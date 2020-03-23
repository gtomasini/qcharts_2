#ifndef UDPSERVER_H
#define UDPSERVER_H

#include <QObject>
#include <QtNetwork/QUdpSocket>
#include "daa_datatypes_icd/trackICD.h"

class udpServer:public QObject{
    Q_OBJECT

public:
    udpServer(int port=2200);

    void initSocket();
    int getPort(){ return m_port; }
    virtual ~udpServer(){delete m_udpSocket;}

private slots:
    void readPendingDatagrams();
    void setPort(int port);

signals:
    void newDatagramSignal(QByteArray &, int port);

private:
    int m_port=2200;

    QUdpSocket *m_udpSocket;
};

#endif // TRACKLISTENER_H
