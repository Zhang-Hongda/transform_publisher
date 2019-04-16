/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include "../include/transform_publisher/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace transform_publisher {

using namespace Qt;
extern QMutex tf_mutex;
extern tf::Transform transform;
extern std::string frame_id, childframe_id;
extern bool stopSign;
inline tf::Transform readtransform() {
  QMutexLocker locker(&tf_mutex);
  return transform;
}

inline tf::Transform settransform(tf::Transform _transform) {
  QMutexLocker locker(&tf_mutex);
  transform = _transform;
}

inline std::string readframe_id() {
  QMutexLocker locker(&tf_mutex);
  return frame_id;
}

inline void setframe_id(std::string id) {
  QMutexLocker locker(&tf_mutex);
  frame_id = id;
}
inline std::string readchildframe_id() {
  QMutexLocker locker(&tf_mutex);
  return childframe_id;
}

inline void setchildframe_id(std::string id) {
  QMutexLocker locker(&tf_mutex);
  childframe_id = id;
}
inline bool readstopSigen() {
  QMutexLocker locker(&tf_mutex);
  return stopSign;
}

inline void setstopSign(bool sign) {
  QMutexLocker locker(&tf_mutex);
  stopSign = sign;
}

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv) {
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to
                     // on_...() callbacks in this class.
  QObject::connect(
      ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
      SLOT(aboutQt()));  // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing -
                                       // qt-designer should have this already
                                       // hardwired, but often loses it
                                       // (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  /*********************
  ** Auto Start
  **********************/
  if (ui.checkbox_remember_settings->isChecked()) {
    on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check) {
  if (ui.checkbox_use_environment->isChecked()) {
    if (!qnode.init()) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
    }
  } else {
    if (!qnode.init(ui.line_edit_master->text().toStdString(),
                    ui.line_edit_host->text().toStdString())) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
    }
  }
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if (state == 0) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(
      this, tr("About ..."),
      tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin "
         "Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "transform_publisher");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url =
      settings.value("master_url", QString("http://192.168.1.2:11311/"))
          .toString();
  QString host_url =
      settings.value("host_url", QString("192.168.1.3")).toString();
  ui.line_edit_master->setText(master_url);
  ui.line_edit_host->setText(host_url);
  // ui.line_edit_topic->setText(topic_name);
  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked);
  if (checked) {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
  }
  ui.doubleSpinBox_x->setValue(settings.value("x", 0).toDouble());
  ui.doubleSpinBox_y->setValue(settings.value("y", 0).toDouble());
  ui.doubleSpinBox_z->setValue(settings.value("z", 0).toDouble());
  ui.doubleSpinBox_r->setValue(settings.value("r", 0).toDouble());
  ui.doubleSpinBox_p->setValue(settings.value("p", 0).toDouble());
  ui.doubleSpinBox_y_2->setValue(settings.value("y_2", 0).toDouble());
  ui.lineEdit_frame_id->setText(
      settings.value("f_id", QString("/base_link")).toString());
  ui.lineEdit_child_frame_id->setText(
      settings.value("c_id", QString("/kinect2_link")).toString());
  updateTransform();
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "transform_publisher");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  settings.setValue("use_environment_variables",
                    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
                    QVariant(ui.checkbox_remember_settings->isChecked()));
  settings.setValue("x", QVariant(ui.doubleSpinBox_x->value()));
  settings.setValue("y", QVariant(ui.doubleSpinBox_y->value()));
  settings.setValue("z", QVariant(ui.doubleSpinBox_z->value()));
  settings.setValue("r", QVariant(ui.doubleSpinBox_r->value()));
  settings.setValue("p", QVariant(ui.doubleSpinBox_p->value()));
  settings.setValue("y_2", QVariant(ui.doubleSpinBox_y_2->value()));
  settings.setValue("f_id", ui.lineEdit_frame_id->text());
  settings.setValue("c_id", ui.lineEdit_child_frame_id->text());
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace transform_publisher

void transform_publisher::MainWindow::updateTransform() {
  tf::Transform t;
  t.setOrigin(tf::Vector3(ui.doubleSpinBox_x->value(),
                          ui.doubleSpinBox_y->value(),
                          ui.doubleSpinBox_z->value()));
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(ui.doubleSpinBox_y_2->value() / 180.0 * M_PI,
                        Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(ui.doubleSpinBox_p->value() / 180.0 * M_PI,
                        Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(ui.doubleSpinBox_r->value() / 180.0 * M_PI,
                        Eigen::Vector3f::UnitX());
  float qw, qx, qy, qz;
  qw = q.w();
  qx = q.x();
  qy = q.y();
  qz = q.z();
  t.setRotation(tf::Quaternion(qx, qy, qz, qw));

  ui.lineEdit_qx->setText(QString::fromStdString(std::to_string(qx)));
  ui.lineEdit_qy->setText(QString::fromStdString(std::to_string(qy)));
  ui.lineEdit_qz->setText(QString::fromStdString(std::to_string(qz)));
  ui.lineEdit_qw->setText(QString::fromStdString(std::to_string(qw)));
  settransform(t);
  setframe_id(ui.lineEdit_frame_id->text().toStdString());
  setchildframe_id(ui.lineEdit_child_frame_id->text().toStdString());
}

void transform_publisher::MainWindow::on_lineEdit_frame_id_editingFinished() {
  setframe_id(ui.lineEdit_frame_id->text().toStdString());
}

void transform_publisher::MainWindow::
    on_lineEdit_child_frame_id_editingFinished() {
  setchildframe_id(ui.lineEdit_child_frame_id->text().toStdString());
}

void transform_publisher::MainWindow::on_pushButton_publish_toggled(
    bool checked) {
  if (checked) {
    setstopSign(false);
    ui.pushButton_publish->setText("stop");
  } else {
    setstopSign(true);
    ui.pushButton_publish->setText("publish");
  }
}

void transform_publisher::MainWindow::on_doubleSpinBox_x_editingFinished() {
  updateTransform();
}

void transform_publisher::MainWindow::on_doubleSpinBox_y_editingFinished() {
  updateTransform();
}

void transform_publisher::MainWindow::on_doubleSpinBox_z_editingFinished() {
  updateTransform();
}

void transform_publisher::MainWindow::on_doubleSpinBox_r_editingFinished() {
  updateTransform();
}

void transform_publisher::MainWindow::on_doubleSpinBox_p_editingFinished() {
  updateTransform();
}

void transform_publisher::MainWindow::on_doubleSpinBox_y_2_editingFinished() {
  updateTransform();
}
