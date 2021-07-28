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
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <iostream>
#include <iomanip>
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
    , qnode(argc,argv)
    , service_pose(argc,argv,&my_logging_model)
    , service_pose_2(argc,argv,&my_logging_model)
    , scene(new my_graphicsScene)
    , pixmapItem(new QGraphicsPixmapItem)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

//    ui.HomePage_background->setPixmap(QPixmap("://images/background.png"));
    QObject::connect(&qnode,SIGNAL(test()),this,SLOT(TEST()));
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

//    ui.flight_map->setPixmap(QPixmap(":/images/234.jpg"));

//    QGraphicsScene scene;   // 定义一个场景，设置背景色为红色.setBackgroundBrush(Qt::red);
    image_roll.load(":/images/234.jpg");
    std::cout << "1" << std::endl;


    ui.graphicsView_flight->setScene(scene);
    ui.graphicsView_flight->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    QPixmap image(":/images/icon.png");
    pixmapItem->setPixmap(image.scaled(50,50));
    pixmapItem->setFlag(QGraphicsItem::ItemIsMovable,true);
    scene->addItem(pixmapItem);


    ui.roll_map->setPixmap(QPixmap::fromImage(image_roll));
    QObject::connect(&qnode,SIGNAL(getdata()),this,SLOT(updatemap()));

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    ui.view_logging_2->setModel(&my_logging_model);
    ui.view_logging_flight->setModel(&my_logging_model);
    ui.view_logging_roll->setModel(&my_logging_model);
    ui.view_logging_switch->setModel(&my_logging_model);
    QObject::connect(&service_pose, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/


void MainWindow::updatemap()
{

//    pixmapItem->setPos(QPointF(qnode.msg.pose.position.x*100.0,qnode.msg.pose.position.y*100.0));
//    pixmapItem->setPos(100,100);
    pixmapItem->setPos(qnode.msg.pose.position.x*500,qnode.msg.pose.position.y*500);
    pixmapItem->update();
}

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::TEST()
{
    ui.test_Edit->setText(qnode.msg.mode.c_str());

    std::ostringstream ss;
    ss<< std::setiosflags(std::ios::fixed)<<std::setprecision(2);
    ss << qnode.msg.pose.position.x << " , "
       << qnode.msg.pose.position.y << " , "
       << qnode.msg.pose.position.z ;
    ui.Edit_flight_1_pos->setText(ss.str().c_str());
    ui.Edit_roll_1_pos->setText(ss.str().c_str());
    ui.Edit_switch_1_pos->setText(ss.str().c_str());
    ss << qnode.msg.pose.position.x;
    ui.Edit_roll_2_pos->setText(ss.str().c_str());
    ui.Edit_flight_2_pos->setText(ss.str().c_str());


    ss << qnode.msg.pose.position.y;
    ui.Edit_roll_2_connected->setText(ss.str().c_str());
    ui.Edit_flight_2_connected->setText(ss.str().c_str());

    ss.str("");
    ss << unsigned(qnode.msg.connected);
    ui.Edit_flight_1_connected->setText(ss.str().c_str());
    ui.Edit_roll_1_connected->setText(ss.str().c_str());
    ui.Edit_switch_1_connected->setText(ss.str().c_str());

    ss.str("");
    ss << unsigned(qnode.msg.armed);
    ui.Edit_flight_1_armed->setText(ss.str().c_str());
    ui.Edit_roll_1_armed->setText(ss.str().c_str());
    ui.Edit_switch_1_armed->setText(ss.str().c_str());

    ss.str("");
    ss << unsigned(qnode.msg.manual_input);
    ui.Edit_flight_1_input->setText(ss.str().c_str());
    ui.Edit_switch_1_input->setText(ss.str().c_str());
    ui.Edit_roll_1_input->setText(ss.str().c_str());

    ss.str("");
    ss << unsigned(qnode.msg.adhere[0])
       << unsigned(qnode.msg.adhere[1])
       << unsigned(qnode.msg.adhere[2])
       << unsigned(qnode.msg.adhere[3]);
    ui.Edit_flight_1_status->setText(ss.str().c_str());
    ui.Edit_roll_1_status->setText(ss.str().c_str());
    ui.Edit_switch_1_status->setText(ss.str().c_str());
//    +","+qnode.msg.pose.position.y+","+qnode.msg.pose.position.z
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
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
        ui.view_logging->scrollToBottom();
        ui.view_logging_2->scrollToBottom();
        ui.view_logging_flight->scrollToBottom();
        ui.view_logging_roll->scrollToBottom();
        ui.view_logging_switch->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
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
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace class1_ros_qt_demo

void class1_ros_qt_demo::MainWindow::on_button_connect_clicked(bool checked)
{

    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString()) )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void class1_ros_qt_demo::MainWindow::on_button_connect_clicked()
{

}

void class1_ros_qt_demo::MainWindow::on_button_Mission_clicked(bool checked)
{
    ros_qt_demo::MissionCommand mc;
    mc.request.state = atoi(ui.Mission_Edit->text().toStdString().c_str()) ;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   mc ) )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void class1_ros_qt_demo::MainWindow::on_button_Formation_clicked(bool checked)
{
    ros_qt_demo::FormationCommand fc;
    fc.request.formation = atoi(ui.Formation_Edit->text().toStdString().c_str()) ;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   fc ) )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void class1_ros_qt_demo::MainWindow::on_button_Mode_clicked(bool checked)
{
    ros_qt_demo::ModeCommand mdc;
    mdc.request.mode = atoi(ui.Mode_Edit->text().toStdString().c_str()) ;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   mdc ) )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void class1_ros_qt_demo::MainWindow::on_button_Pose_clicked(bool checked)
{
    ros_qt_demo::PoseCommand pc;
    pc.request.waypoint_number = atoi(ui.Pose_Edit->text().toStdString().c_str()) ;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   pc ) )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void class1_ros_qt_demo::MainWindow::on_Button_flight_Takeoff_clicked(bool checked)
{
    ros_qt_demo::MissionCommand mc;
    mc.request.state = 1 ;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   mc ) )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void class1_ros_qt_demo::MainWindow::on_Button_flight_Landing_clicked(bool checked)
{
    ros_qt_demo::MissionCommand mc;
    mc.request.state = 2 ;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   mc ) )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void class1_ros_qt_demo::MainWindow::on_Button_flight_Formation_clicked(bool checked)
{
    ros_qt_demo::MissionCommand mc;
    ros_qt_demo::FormationCommand fc;
    mc.request.state = 3 ;
    fc.request.formation=45;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   mc ) ||
             ! service_pose_2.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   fc )
             )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }


}

void class1_ros_qt_demo::MainWindow::on_Button_flight_Mission_clicked(bool checked)
{
    ros_qt_demo::MissionCommand mc;
    mc.request.state = 6 ;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   mc ) )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void class1_ros_qt_demo::MainWindow::on_Button_flight_Switch_clicked(bool checked)
{
    ros_qt_demo::MissionCommand mc;
    ros_qt_demo::PoseCommand pc;
    mc.request.state = 17 ;
    pc.request.mode=1;
    pc.request.waypoint_number=1;
    pc.request.pose[0].position.x=1.2;
    pc.request.pose[0].position.y=3.4;
    pc.request.pose[0].position.z=5.6;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   mc ) ||
             ! service_pose_2.init(ui.line_edit_master->text().toStdString(),
                                ui.line_edit_host->text().toStdString(),
                                pc )

             )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void class1_ros_qt_demo::MainWindow::on_Button_flight_trajmode_clicked(bool checked)
{
    ros_qt_demo::MissionCommand mc;
    ros_qt_demo::ModeCommand mdc;
    mc.request.state = 4 ;
    mdc.request.mode=45;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   mc ) ||
             ! service_pose_2.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   mdc )
             )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void class1_ros_qt_demo::MainWindow::on_Button_flight_hit_clicked(bool checked)
{
    ros_qt_demo::MissionCommand mc;
    ros_qt_demo::PoseCommand pc;
    mc.request.state = 5;
    pc.request.mode=1;
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !service_pose.init() )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! service_pose.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   mc ) ||
             ! service_pose_2.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString(),
                   pc )
             )
        {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}



void class1_ros_qt_demo::MainWindow::on_pushButton_clicked()
{
    ui.stackedWidget->setCurrentIndex(1);
}

void class1_ros_qt_demo::MainWindow::on_pushButton_2_clicked()
{
    ui.stackedWidget->setCurrentIndex(0);
}

void class1_ros_qt_demo::MainWindow::on_Button_Quit_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(0);
}

void class1_ros_qt_demo::MainWindow::on_Button_ChooseType_clicked()
{
    QBCui.stackedWidget->setCurrentIndex(1);
}
