#include <QtNetwork\QNetworkDatagram>
#include <QByteArray>
#include <iostream>
#include "udpServer.h"
#include "daa_datatypes_icd/trackICD.h"

udpServer::udpServer(int port):m_port(port){
     m_udpSocket = new QUdpSocket(this);
}

//std::make_pair("adsb", "2200"),
//std::make_pair("camera", "2202"),
//std::make_pair("radar", "2201"),
//std::make_pair("clean", "2300"),
//std::make_pair("ownship", "2100")

void
udpServer::initSocket(){
    m_udpSocket->bind(QHostAddress::LocalHost, m_port);

    connect(m_udpSocket, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));
}

void
udpServer::setPort(int port){
    std::cout<<"udpServer::setPort("<<port<<")"<<std::endl;
    m_port=port;
    m_udpSocket->disconnectFromHost();
    m_udpSocket->close();

    disconnect(m_udpSocket, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));
    delete(m_udpSocket);
    m_udpSocket=new QUdpSocket(this);
    initSocket();
}

void
udpServer::readPendingDatagrams(){
    for (;m_udpSocket->hasPendingDatagrams(); ) {
           QNetworkDatagram datagram = m_udpSocket->receiveDatagram();
           QByteArray buffer=datagram.data();
           std::cout<<"datagram("<<buffer.size()<<") received in port: "
                   <<datagram.destinationPort()<<std::endl;
           emit newDatagramSignal(buffer, datagram.destinationPort());//signal another member class
    }
}
