#ifndef ROSVIEWER3D_HPP
#define ROSVIEWER3D_HPP

#include <map>

// BOOST
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// ROS
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// OPENCV
#include <opencv2/opencv.hpp>

// Eigen
#include <Eigen/Core>

namespace veeu {

class SpinLockV2 {
  boost::atomic_flag flag;

public:
  void lock() {
    while (flag.test_and_set(boost::memory_order_acquire))
      ;
  }
  bool try_lock() { return !flag.test_and_set(boost::memory_order_acquire); }
  void unlock() { flag.clear(boost::memory_order_release); }
};

typedef SpinLockV2 Lock;

class RosViewer3D {
public:
  RosViewer3D(std::string name = "RosViewer3D") {
    /**
     * ROS
     */
    std::map<std::string, std::string> fakeParameters;
    ros::init(fakeParameters, name, ros::init_options::NoSigintHandler);
    this->nh = new ros::NodeHandle();
    this->br = new tf::TransformBroadcaster();
    this->visualization_publisher_ =
        this->nh->advertise<visualization_msgs::MarkerArray>(
            "visualization_objects", 1);

    /**
     * Main Thread
     */
    main_thread_ = boost::thread(boost::bind(&RosViewer3D::mainLoop, this));
  }
  virtual ~RosViewer3D() {
    printf("DELETEING\n");
    delete this->nh;
    ros::shutdown();
  }

  void mainLoop() {
    while (ros::ok()) {
      printf("Updateing\n");
      update();

      ros::spinOnce();
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
    }
  }

  void update() {

    resources_lock_.lock();

    /**
     * Show Reference Frames
     */
    for (auto it = reference_frames_.begin(); it != reference_frames_.end();
         ++it) {
      tf::Transform camera_pose_tf;
      Eigen::Affine3d camera_affine;
      camera_affine = it->second;
      tf::transformEigenToTF(camera_affine, camera_pose_tf);
      this->br->sendTransform(tf::StampedTransform(
          camera_pose_tf, ros::Time::now(), "world", it->first));
    }

    /**
     * Show objects
     */
    objects_array_.markers.clear();
    for (auto it = markers_map_.begin(); it != markers_map_.end(); ++it) {
      printf("Publishing %s\n", it->first.c_str());
      objects_array_.markers.push_back(it->second);
    }

    resources_lock_.unlock();

    this->visualization_publisher_.publish(objects_array_);
  }

  void storeRF(std::string name, Eigen::Matrix4d rf) {
    resources_lock_.lock();
    reference_frames_[name] = rf;
    resources_lock_.unlock();
  }

  void storeMarker(std::string name, visualization_msgs::Marker marker) {
    resources_lock_.lock();
    markers_map_[name] = marker;
    resources_lock_.unlock();
  }

  void showReferenceFrame(std::string name, Eigen::Matrix4f rf) {
    storeRF(name, rf.cast<double>());
  }

  template <class TYPE>
  void showVoxels(std::string name,
                  Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic> &data,
                  float resolution = 0.1, std::string parent_name = "world") {

    visualization_msgs::Marker marker = this->createVoxelsMarker(
        parent_name, ros::Time(0), name, data, resolution);

    storeMarker(name, marker);
  }

private:
  template <class TYPE>
  visualization_msgs::Marker
  createVoxelsMarker(std::string frame_id, ros::Time time, std::string name,
                     Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic> &data,
                     float size = 0.1, int min_weight_th = 1,
                     cv::Scalar col = cv::Scalar(255, 255, 255)) {

    /**
*Creating Visualization Marker
*/
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = time;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.ns = name;

    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;

    cv::Mat colorSpace(1, data.rows(), CV_32FC3);

    for (int i = 0; i < data.rows(); i++) {

      /**
       * Create 3D Point from 3D Voxel
       */
      geometry_msgs::Point point;
      point.x = data(i, 0);
      point.y = data(i, 1);
      point.z = data(i, 2);

      /**
       * Assign Cube Color from Voxel Color
       */
      std_msgs::ColorRGBA color;

      if (data.cols() >= 6) {
        color.r = data(i, 3) / 255.;
        color.g = data(i, 4) / 255.;
        color.b = data(i, 5) / 255.;
        color.a = 1;
      } else {
        color.r = 255. / 255.;
        color.g = 255. / 255.;
        color.b = 255. / 255.;
        color.a = 1;
      }

      marker.points.push_back(point);
      marker.colors.push_back(color);
    }

    return marker;
  }

  ///
  ros::NodeHandle *nh;
  tf::TransformBroadcaster *br;
  ros::Publisher visualization_publisher_;
  std::map<std::string, visualization_msgs::Marker> markers_map_;
  visualization_msgs::MarkerArray objects_array_;
  //
  boost::thread main_thread_;
  //

  std::map<
      std::string, Eigen::Matrix4d, std::less<std::string>,
      Eigen::aligned_allocator<std::pair<const std::string, Eigen::Matrix4d>>>
      reference_frames_;

  //
  Lock resources_lock_;
};

} // namespace atlas

#endif
