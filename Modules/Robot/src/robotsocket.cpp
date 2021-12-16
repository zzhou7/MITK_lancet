#include "robotsocket.h"

#ifdef Q_OS_WIN
#pragma execution_character_set("utf-8")
#endif

RobotSocket::RobotSocket(QObject *parent,qintptr p):QTcpSocket(0)
{
    qDebug()<<"RobotSocket::RobotSocket";
    qDebug()<<__FUNCTION__<<"ThreadId: "<<thread()->currentThreadId();
    socket_state = QAbstractSocket::UnconnectedState;
    this->setSocketDescriptor(p);

    this->connect(this, SIGNAL(error(QAbstractSocket::SocketError)),this,SLOT(displayError(QAbstractSocket::SocketError)));
    this->connect(this, SIGNAL(stateChanged(QAbstractSocket::SocketState)),this,SLOT(socketStateChanged(QAbstractSocket::SocketState)));
    this->connect(this, SIGNAL(readyRead()),this, SLOT(socketPopData()));
}

RobotSocket::~RobotSocket()
{
    qDebug()<<"RobotSocket::~RobotSocket";
    this->disconnectFromHost();
}

void RobotSocket::socketPopData()
{
    qDebug()<<"RobotSocket::socketPopData emit socket_data";
    emit socket_data(this->readAll());
}

void RobotSocket::socketWriteData(QString dstr)
{
    qDebug()<<"RobotSocket::socketWriteData";
    this->write(dstr.toLatin1());
    qDebug()<< "Socket send:" <<dstr.toLatin1();
}

// Socket State Handle
void RobotSocket::displayError(QAbstractSocket::SocketError e)
{
    qDebug()<<"RobotSocket::displayError";
    qDebug() << "error:"<< e;
    qDebug()<<__FUNCTION__<<e;
}

void RobotSocket::socketStateChanged(QAbstractSocket::SocketState s)
{
    qDebug()<<"RobotSocket::socketStateChanged";
    socket_state = s;
    qDebug()<<__FUNCTION__<<socket_state;
}

// Timer Handle
void RobotSocket::handleTimeout()
{
    qDebug()<<"RobotSocket::handleTimeout";
    qDebug()<<"TimeOut";
}
