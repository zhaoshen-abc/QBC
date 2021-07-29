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
#include "../include/ros_qt_demo/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace class1_ros_qt_demo {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  , qnode(argc,argv,&my_logging_model)
  , service_pose(argc,argv,&my_logging_model)
  , service_pose_2(argc,argv,&my_logging_model)
  , dialog(new ConnectDialog(this))
{
    QBCui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QBCui.Label_page5_5->setStyleSheet("QLabel{background-color:rgb(204, 255, 204);border-bottom:2px groove rgba(0, 0, 0, 0.25);border-radius: 18px;padding:2px 2px;border-style: outset;}");
    QObject::connect(dialog, &ConnectDialog::on_button_connect_clicked,this, &MainWindow::on_button_connect_clicked);
    QObject::connect(dialog, &ConnectDialog::on_checkbox_use_environment_stateChanged,this, &MainWindow::on_checkbox_use_environment_stateChanged);

    ReadSettings();
    setWindowIcon(QIcon("://images/icon.png"));
    QBCui.stackedWidget->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
//    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Logging
    **********************/
    QBCui.ListView_page3->setModel(&my_logging_model);
    QBCui.ListView_page4->setModel(&my_logging_model);
    QBCui.ListView_page5->setModel(&my_logging_model);
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&service_pose, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    if ( dialog->ui->checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked();
    }

    QObject::connect(&qnode, SIGNAL(getdata()), this, SLOT(DisplayData()));

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked() {
  if ( dialog->ui->checkbox_use_environment->isChecked() ) {
    if ( !qnode.init() ) {
      showNoMasterMessage();
    } else {
      dialog->ui->button_connect->setEnabled(false);
    }
  } else {
    if ( ! qnode.init(dialog->ui->line_edit_master->text().toStdString(),
           dialog->ui->line_edit_host->text().toStdString()) ) {
      showNoMasterMessage();
    } else {
      dialog->ui->button_connect->setEnabled(false);
      dialog->ui->line_edit_master->setReadOnly(true);
      dialog->ui->line_edit_host->setReadOnly(true);
      dialog->ui->line_edit_topic->setReadOnly(true);
    }
  }
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if ( state == 0 ) {
    enabled = true;
  } else {
    enabled = false;
  }
  dialog->ui->line_edit_master->setEnabled(enabled);
  dialog->ui->line_edit_host->setEnabled(enabled);
  //ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        QBCui.ListView_page3->scrollToBottom();
        QBCui.ListView_page4->scrollToBottom();
        QBCui.ListView_page5->scrollToBottom();
}


void MainWindow::DisplayData()
{
//    ui.test_Edit->setText(qnode.msg.mode.c_str());

    std::ostringstream ss;
    ss<< std::setiosflags(std::ios::fixed)<<std::setprecision(2);
    ss << qnode.msg.pose.position.x << " , "
       << qnode.msg.pose.position.y << " , "
       << qnode.msg.pose.position.z ;
    QBCui.Edit_page3_2_pos->setText(ss.str().c_str());
//    ui.Edit_roll_1_pos->setText(ss.str().c_str());
//    ui.Edit_switch_1_pos->setText(ss.str().c_str());
//    ss << qnode.msg.pose.position.x;
//    ui.Edit_roll_2_pos->setText(ss.str().c_str());
//    ui.Edit_flight_2_pos->setText(ss.str().c_str());


//    ss << qnode.msg.pose.position.y;
//    ui.Edit_roll_2_connected->setText(ss.str().c_str());
//    ui.Edit_flight_2_connected->setText(ss.str().c_str());

//    ss.str("");
//    ss << unsigned(qnode.msg.connected);
//    ui.Edit_flight_1_connected->setText(ss.str().c_str());
//    ui.Edit_roll_1_connected->setText(ss.str().c_str());
//    ui.Edit_switch_1_connected->setText(ss.str().c_str());

//    ss.str("");
//    ss << unsigned(qnode.msg.armed);
//    ui.Edit_flight_1_armed->setText(ss.str().c_str());
//    ui.Edit_roll_1_armed->setText(ss.str().c_str());
//    ui.Edit_switch_1_armed->setText(ss.str().c_str());

//    ss.str("");
//    ss << unsigned(qnode.msg.manual_input);
//    ui.Edit_flight_1_input->setText(ss.str().c_str());
//    ui.Edit_switch_1_input->setText(ss.str().c_str());
//    ui.Edit_roll_1_input->setText(ss.str().c_str());

//    ss.str("");
//    ss << unsigned(qnode.msg.adhere[0])
//       << unsigned(qnode.msg.adhere[1])
//       << unsigned(qnode.msg.adhere[2])
//       << unsigned(qnode.msg.adhere[3]);
//    ui.Edit_flight_1_status->setText(ss.str().c_str());
//    ui.Edit_roll_1_status->setText(ss.str().c_str());
//    ui.Edit_switch_1_status->setText(ss.str().c_str());
//    +","+qnode.msg.pose.position.y+","+qnode.msg.pose.position.z
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    dialog->ui->line_edit_master->setText(master_url);
    dialog->ui->line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    dialog->ui->checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    dialog->ui->checkbox_use_environment->setChecked(checked);
    if ( checked ) {
      dialog->ui->line_edit_master->setEnabled(false);
      dialog->ui->line_edit_host->setEnabled(false);
      //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    settings.setValue("master_url",dialog->ui->line_edit_master->text());
    settings.setValue("host_url",dialog->ui->line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(dialog->ui->checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(dialog->ui->checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace class1_ros_qt_demo


void class1_ros_qt_demo::MainWindow::on_Button_Quit_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(0);
}

void class1_ros_qt_demo::MainWindow::on_Button_ChooseType_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(1);
}

void class1_ros_qt_demo::MainWindow::on_Button_page2_type1_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(2);
}

void class1_ros_qt_demo::MainWindow::on_Button_page3_switch_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(3);
}

void class1_ros_qt_demo::MainWindow::on_Button_page4_quit_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(1);
}

void class1_ros_qt_demo::MainWindow::on_Button_page5_quit_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(1);
}

void class1_ros_qt_demo::MainWindow::on_Button_page3_quit_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(1);
}

void class1_ros_qt_demo::MainWindow::on_Button_page4_gather_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(4);
}

void class1_ros_qt_demo::MainWindow::on_Button_page4_disperse_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(2);
}


void class1_ros_qt_demo::MainWindow::on_Button_page5_switch_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(3);
}

void class1_ros_qt_demo::MainWindow::on_Button_page2_connect_clicked()
{

    dialog->show();
}
