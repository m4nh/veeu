#ifndef ROSVIEWER3D_HPP
#define ROSVIEWER3D_HPP

#include <map>

// BOOST
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>
#include <boost/chrono.hpp>
#include <boost/lexical_cast.hpp>
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

// VEEU
#include <veeu/Logger.hpp>

namespace veeu {

auto logger = veeu::Logger::getSimpleLogger("viewer");

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

struct Command {
  static constexpr const char *NEW_COMMAND = "new";
  static constexpr const char *DELETE_COMMAND = "delete";
  static constexpr const char *VOXELMAP_TYPE = "voxelmap";
  static constexpr const char *FRAME_TYPE = "frame";
  static constexpr const char *FRAMES_TYPE = "frames";
  static constexpr const char *BOXES_TYPE = "boxes";

  bool valid;
  std::string action;
  std::string type;
  std::string name;
  std::map<std::string, std::string> params;

  Command(std::string command) {
    this->valid = false;
    std::vector<std::string> chunks;
    boost::split(chunks, command, boost::is_any_of(" "));

    if (chunks.size() > 0) {
      for (auto c : chunks) {
        logger->debug("CHUNK: {}", c);
      }
      if (chunks[0].compare(Command::NEW_COMMAND) == 0 && chunks.size() >= 3) {
        this->action = Command::NEW_COMMAND;
        this->type = boost::to_lower_copy(chunks[1]);
        this->name = boost::to_lower_copy(chunks[2]);

        if (chunks.size() >= 4)
          this->parseParams(chunks[3]);

        this->valid = true;
      }
      if (chunks[0].compare(Command::DELETE_COMMAND) == 0 &&
          chunks.size() >= 2) {
        this->action = Command::NEW_COMMAND;
        this->type = "";
        this->name = boost::to_lower_copy(chunks[1]);
        this->valid = true;
      }
    }
  }

  void parseParams(std::string query) {
    std::vector<std::string> params_chunks;
    boost::split(params_chunks, query, boost::is_any_of(";"));
    for (std::string param_chunk : params_chunks) {
      std::vector<std::string> pair;
      boost::split(pair, param_chunk, boost::is_any_of("="));
      if (pair.size() == 2) {
        this->params[pair[0]] = pair[1];
      }
    }
  }

  template <class C> C getParam(std::string name) {
    try {
      return boost::lexical_cast<C>(this->params[name]);
    } catch (std::exception e) {
      logger->error("Parsing params: {}", e.what());
      return C();
    }
  }

  /**
   * @brief
   *
   * @param action
   * @return true
   * @return false
   */
  inline bool isAction(std::string action) const {
    return this->action.compare(action) == 0;
  }

  /**
   * @brief
   *
   * @param type
   * @return true
   * @return false
   */
  inline bool isType(std::string type) const {
    return this->type.compare(type) == 0;
  }

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  inline bool isValid() const { return this->valid; }
};

class RosViewer3D {
  static constexpr const char *DEFAULT_BASE_FRAME = "world";

public:
  /**
   * @brief Construct a new Ros Viewer 3 D object
   *
   * @param name
   */
  RosViewer3D(std::string name = "rosviewer3d") {

    std::map<std::string, std::string> fakeParameters;
    ros::init(fakeParameters, name, ros::init_options::NoSigintHandler);
    this->nh = new ros::NodeHandle();
    this->br = new tf::TransformBroadcaster();
    this->visualization_publisher_ =
        this->nh->advertise<visualization_msgs::MarkerArray>(
            "visualization_objects", 1);

    main_thread_ = boost::thread(boost::bind(&RosViewer3D::mainLoop, this));
  }

  /**
   * @brief Destroy the Ros Viewer 3 D object
   *
   */
  virtual ~RosViewer3D() {
    printf("DELETEING\n");
    delete this->nh;
    ros::shutdown();
  }

  /**
   * @brief
   *
   * @param command
   * @return true
   * @return false
   */
  bool parseCommand(std::string command, Eigen::MatrixXf &data) {
    Command cmd(command);
    logger->debug("Parsing: {}", command);
    if (cmd.isValid()) {
      logger->debug("Valid");
      if (cmd.isAction(Command::NEW_COMMAND)) {

        if (cmd.isType(Command::VOXELMAP_TYPE)) {
          this->showVoxels(cmd.name, data, cmd.getParam<float>("resolution"));
        }

        if (cmd.isType(Command::FRAME_TYPE)) {
          this->showReferenceFrame(cmd.name, data);
        }

        if (cmd.isType(Command::FRAMES_TYPE)) {
          this->showReferenceFrames(cmd.name, data);
        }
      }
    }
  }

  /**
   * @brief
   *
   * @param name
   * @param marker
   */
  void storeMarker(std::string name, visualization_msgs::Marker marker) {
    resources_lock_.lock();
    markers_map_[name] = marker;
    changes_map_[name] = true;
    resources_lock_.unlock();
  }

  /**
   * @brief
   *
   * @param name
   * @param rf
   */
  void storeRF(std::string name, Eigen::Matrix4d rf) {
    resources_lock_.lock();
    reference_frames_[name] = rf;
    changes_map_[name] = true;
    resources_lock_.unlock();
  }

  /**
   * @brief
   *
   * @param name
   * @param rf
   */
  void showReferenceFrame(std::string name, Eigen::Matrix4f rf) {
    storeRF(name, rf.cast<double>());
  }

  /**
   * @brief
   *
   * @param name
   * @param rf
   */
  void showReferenceFrames(std::string name, Eigen::MatrixXf rfs) {

    std::stringstream ss;
    for (size_t i = 0; i < rfs.rows(); i++) {
      Eigen::MatrixXf temp = rfs.block(i, 0, 1, 16);
      Eigen::Map<Eigen::MatrixXf> row(temp.data(), 4, 4);
      ss.str("");
      ss << name << "_" << i;
      storeRF(ss.str(), row.cast<double>());
    }
    this->forceResetChanges();
  }

  /**
   * @brief
   *
   */
  void forceResetChanges() {
    resources_lock_.lock();
    for (auto it = changes_map_.begin(); it != changes_map_.end(); ++it) {
      changes_map_[it->first] = true;
    }
    resources_lock_.unlock();
  }

  /**
   * @brief
   *
   * @tparam TYPE
   * @param name
   * @param data
   * @param resolution
   * @param parent_name
   */
  template <class TYPE>
  void showVoxels(std::string name,
                  Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic> &data,
                  float resolution = 0.1, std::string parent_name = "world") {

    visualization_msgs::Marker marker = this->createVoxelsMarker(
        parent_name, ros::Time(0), name, data, resolution);

    storeMarker(name, marker);
  }

  /**
   * @brief
   *
   * @tparam TYPE
   * @param name
   * @param data
   * @param resolution
   * @param parent_name
   */
  void showBox(std::string name, Eigen::Vector3f size, Eigen::Vector3f offset,
               Eigen::Vector4f color = Eigen::Vector4f(1, 1, 1, 1),
               std::string parent_name = "world") {
    visualization_msgs::Marker marker =
        this->createSimpleBoxVisualizationMarker(parent_name, ros::Time(0),
                                                 name, size, color, offset);
    storeMarker(name, marker);
  }

  /**
  * @brief
  */
  void showBoxes(std::string name, Eigen::MatrixXf data) {
    std::stringstream ss;
    for (size_t i = 0; i < data.rows(); i++) {
      Eigen::MatrixXf temp = data.block(i, 0, 1, 10);
      Eigen::Vector3f size = temp.block(i, 0, 1, 3);
      Eigen::Vector3f offset = temp.block(i, 3, 1, 3);
      Eigen::Vector4f color = temp.block(i, 6, 1, 4);

      ss.str("");
      ss << name << "_" << i;
      showBox(ss.str(), size, offset, color);
    }
    this->forceResetChanges();
  }

private:
  void test() {}
  /**
   * @brief
   *
   */
  void mainLoop() {
    while (ros::ok()) {
      printf("Updateing\n");
      update();

      ros::spinOnce();
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
    }
  }

  /**
   * @brief
   *
   */
  void update() {

    resources_lock_.lock();

    for (auto it = reference_frames_.begin(); it != reference_frames_.end();
         ++it) {
      if (!changes_map_[it->first])
        continue;
      changes_map_[it->first] = false;
      tf::Transform camera_pose_tf;
      Eigen::Affine3d camera_affine;
      camera_affine = it->second;
      tf::transformEigenToTF(camera_affine, camera_pose_tf);
      this->br->sendTransform(tf::StampedTransform(
          camera_pose_tf, ros::Time::now(), "world", it->first));
    }

    objects_array_.markers.clear();
    for (auto it = markers_map_.begin(); it != markers_map_.end(); ++it) {
      if (!changes_map_[it->first])
        continue;
      changes_map_[it->first] = false;
      printf("Publishing %s\n", it->first.c_str());
      objects_array_.markers.push_back(it->second);
    }

    resources_lock_.unlock();

    this->visualization_publisher_.publish(objects_array_);
  }

  /**
   * @brief Create a Voxels Marker object
   *
   * @tparam TYPE
   * @param frame_id
   * @param time
   * @param name
   * @param data
   * @param size
   * @param min_weight_th
   * @param col
   * @return visualization_msgs::Marker
   */
  template <class TYPE>
  visualization_msgs::Marker
  createVoxelsMarker(std::string frame_id, ros::Time time, std::string name,
                     Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic> &data,
                     float size = 0.1, int min_weight_th = 1,
                     cv::Scalar col = cv::Scalar(255, 255, 255)) {

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

      geometry_msgs::Point point;
      point.x = data(i, 0);
      point.y = data(i, 1);
      point.z = data(i, 2);

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

  visualization_msgs::Marker createSimpleBoxVisualizationMarker(
      std::string frame_id, ros::Time time, std::string name,
      Eigen::Vector3f size, Eigen::Vector4f color,
      Eigen::Vector3f offset = Eigen::Vector3f(0, 0, 0),
      int method = visualization_msgs::Marker::ADD) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = time;
    marker.action = method;
    marker.id = 0;
    marker.ns = name;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = size(0);
    marker.scale.y = size(1);
    marker.scale.z = size(2);

    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.color.a = color(3);

    marker.pose.position.x = offset(0);
    marker.pose.position.y = offset(1);
    marker.pose.position.z = offset(2);
    return marker;
  }

  ros::NodeHandle *nh;
  tf::TransformBroadcaster *br;
  ros::Publisher visualization_publisher_;
  std::map<std::string, visualization_msgs::Marker> markers_map_;
  visualization_msgs::MarkerArray objects_array_;

  boost::thread main_thread_;

  std::map<
      std::string, Eigen::Matrix4d, std::less<std::string>,
      Eigen::aligned_allocator<std::pair<const std::string, Eigen::Matrix4d>>>
      reference_frames_;

  std::map<std::string, bool> changes_map_;

  Lock resources_lock_;
};

} // namespace veeu

#endif
