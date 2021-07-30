#ifndef CONNECTDIALOG_H
#define CONNECTDIALOG_H

#include <QDialog>
#include "ui_connectdialog.h"

namespace Ui {
class ConnectDialog;
}

class ConnectDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ConnectDialog(QWidget *parent = nullptr);
  ~ConnectDialog();

public slots:
  void on_button_connect_clicked();
  void on_checkbox_use_environment_stateChanged(int arg1);

public:
  Ui::ConnectDialog *ui;


};

#endif // CONNECTDIALOG_H
