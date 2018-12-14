#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <vector>

#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

#include <veeu/Logger.hpp>
#include <veeu/Postcard.hpp>
#include <veeu/Viewer3D.hpp>

#include <sstream>

auto logger = veeu::Logger::getSimpleLogger();

veeu::RosViewer3D *viewer;

veeu::postcard::Message newMessage(std::string session_id,
                                   veeu::postcard::Message message) {
  logger->debug("New Message: {},{},{},{}", message.header.width,
                message.header.height, message.header.depth,
                message.header.byte_per_element);

  Eigen::MatrixXf mat;
  message.fetchMatrix(mat);

  viewer->parseCommand(message.getCommand(), mat);

  veeu::postcard::Message response("DONE");
  return response;
}

void newSession(veeu::postcard::Session::Ptr session) {

  logger->debug("New Session: {}", session->getUUID());
  session->registerNewIdentifiedMessageCallback(newMessage);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {

  // srand((unsigned int)time(0));
  // Eigen::MatrixXf mat;
  // mat = Eigen::MatrixXf::Random(300, 6);
  // mat = mat * 100.0;

  viewer = new veeu::RosViewer3D();
  viewer->showReferenceFrame("base", Eigen::Matrix4f::Identity());

  // viewer->parseCommand("new voxelmap gino resolution=1.1", mat);

  // Eigen::MatrixXf rf = Eigen::Matrix4f::Identity();
  // rf.block(0, 3, 3, 1) = Eigen::MatrixXf::Random(3, 1) * 100;

  // viewer->parseCommand("new frame pino", rf);

  veeu::postcard::PostcardServer server(9999);

  server.registerAcceptCallabck(newSession);
  server.run();

  return 0;
}