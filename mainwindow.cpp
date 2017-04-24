#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  ui->peersTable->setColumnCount(3);

  sockMain = new QUdpSocket(this);
  sockMain->bind(QHostAddress::AnyIPv4, 4242, QUdpSocket::ShareAddress);
  sockMain->joinMulticastGroup(QHostAddress("233.0.0.1"));
  connect(sockMain, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::readPendingDatagrams()
{
  while(sockMain->hasPendingDatagrams()) {
    QByteArray datagram;
    datagram.resize(sockMain->pendingDatagramSize());
    sockMain->readDatagram(datagram.data(), datagram.size());

    bool found = false;
    for(int i = 0 ; i < ui->peersTable->rowCount() ; i++) {
      if(ui->peersTable->item(i, 0)->text() == QString(datagram)) {
        found = true;
        int count = ui->peersTable->item(i,1)->text().toInt();
        ui->peersTable->item(i,1)->setText(QString("%1").arg(count+1));
      }
    }
    if(!found) {
      ui->peersTable->setRowCount(ui->peersTable->rowCount()+1);
      ui->peersTable->setItem(ui->peersTable->rowCount()-1, 0, new QTableWidgetItem(QString(datagram)));
      ui->peersTable->setItem(ui->peersTable->rowCount()-1, 1, new QTableWidgetItem(QString("1")));
    }
  }
}
