#include "../include/ros_qt_demo/connectdialog.h"

ConnectDialog::ConnectDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ConnectDialog)
{
  ui->setupUi(this);
}

ConnectDialog::~ConnectDialog()
{
  delete ui;
}

void ConnectDialog::on_button_connect_clicked()
{
}

void ConnectDialog::on_checkbox_use_environment_stateChanged(int arg1)
{

}
