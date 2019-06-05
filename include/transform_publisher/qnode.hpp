/**
 * @file /include/transform_publisher/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef transform_publisher_QNODE_HPP_
#define transform_publisher_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QMutex>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace transform_publisher {
extern QMutex tf_mutex;
extern tf::Transform transform;
extern std::string frame_id, childframe_id;
extern bool stopSign;
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
 public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  bool init(const std::string& master_url, const std::string& host_url);
  void run();
  void update(std::string frame_id, std::string childframe_id,
              tf::Transform transform);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void nodeReady();

 private:
  int init_argc;
  char** init_argv;
  std::shared_ptr<tf::TransformBroadcaster> br;
};

}  // namespace transform_publisher

#endif /* transform_publisher_QNODE_HPP_ */
