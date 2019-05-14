/*****************************************************************************
** Includes
*****************************************************************************/

#include <QApplication>
#include <QtGui>
#include "../include/transform_publisher/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);
  transform_publisher::MainWindow w(argc, argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
