/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/network.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <string>
#include "../include/transform_publisher/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace transform_publisher
{
QMutex tf_mutex;
tf::Transform transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
std::string frame_id = "/base_link", childframe_id = "/kinect2_link";
bool stopSign = true;
inline tf::Transform readtransform()
{
  QMutexLocker locker(&tf_mutex);
  return transform;
}

inline tf::Transform settransform(tf::Transform _transform)
{
  QMutexLocker locker(&tf_mutex);
  transform = _transform;
}

inline std::string readframe_id()
{
  QMutexLocker locker(&tf_mutex);
  return frame_id;
}

inline void setframe_id(std::string id)
{
  QMutexLocker locker(&tf_mutex);
  frame_id = id;
}
inline std::string readchildframe_id()
{
  QMutexLocker locker(&tf_mutex);
  return childframe_id;
}

inline void setchildframe_id(std::string id)
{
  QMutexLocker locker(&tf_mutex);
  childframe_id = id;
}
inline bool readstopSigen()
{
  QMutexLocker locker(&tf_mutex);
  return stopSign;
}

inline void setstopSign(bool sign)
{
  QMutexLocker locker(&tf_mutex);
  stopSign = sign;
}

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "transform_publisher");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  ros::NodeHandle n;
  br = std::make_shared<tf::TransformBroadcaster>();
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "transform_publisher");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  ros::NodeHandle n;
  br = std::make_shared<tf::TransformBroadcaster>();
  start();
  return true;
}

void QNode::update(std::string _frame_id, std::string _childframe_id, tf::Transform _transform)
{
  frame_id = _frame_id;
  childframe_id = _childframe_id;
  transform = _transform;
}

void QNode::run()
{
  ros::Rate loop_rate(10.0);
  Q_EMIT
  nodeReady();
  while (ros::ok())
  {
    tf::Transform t = readtransform();
    std::string f_id = readframe_id();
    std::string c_id = readchildframe_id();
    if (readstopSigen())
      continue;
    br->sendTransform(tf::StampedTransform(t, ros::Time::now(), f_id, c_id));
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT
  rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

}  // namespace transform_publisher
