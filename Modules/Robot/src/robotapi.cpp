#include "robotapi.h"

RobotApi::RobotApi()
{
    robot_state.clear();
    newThread = new QThread();
    robotControler = new RobotControler(0, socketDescriptor);
    robotControler->moveToThread(newThread);
    this->connect(newThread, SIGNAL(started()), robotControler, SLOT(RunRobotControler()));
    this->connect(this, SIGNAL(signal_api_connectrobot()),robotControler, SLOT(connectrobot()));
    this->connect(this, SIGNAL(signal_api_disconnectrobot()),robotControler, SLOT(disconnectrobot()));
    this->connect(this, SIGNAL(signal_api_poweron()),robotControler, SLOT(poweron()));
    this->connect(this, SIGNAL(signal_api_poweroff()),robotControler, SLOT(poweroff()));
    this->connect(this, SIGNAL(signal_api_setio(int,int)),robotControler, SLOT(setio(int,int)));
    this->connect(this, SIGNAL(signal_api_settcpinfo(TcpInfo)),robotControler, SLOT(settcpinfo(TcpInfo)), Qt::DirectConnection);
    this->connect(this, SIGNAL(signal_api_gettcpinfo()),robotControler, SLOT(gettcpinfo()));
    this->connect(this, SIGNAL(signal_api_movej(double,double,double,double,double,double)),robotControler, SLOT(movej(double,double,double,double,double,double)));
    this->connect(this, SIGNAL(signal_api_movep(double,double,double,double,double,double)),robotControler, SLOT(movep(double,double,double,double,double,double)));
    this->connect(this, SIGNAL(signal_api_movel(double,double,double,double,double,double)), robotControler, SLOT(movel(double,double,double,double,double,double)));
    this->connect(this, SIGNAL(signal_api_stop()),robotControler, SLOT(stop()));
    this->connect(this, SIGNAL(signal_api_setspeed(double)),robotControler, SLOT(setspeed(double)));
    this->connect(this, SIGNAL(signal_api_setworkmode(int)),robotControler, SLOT(setworkmode(int)));
    this->connect(this, SIGNAL(signal_api_setlimitdata(TractionLimit)),robotControler,SLOT(setlimitdata(TractionLimit)), Qt::DirectConnection);
    this->connect(this, SIGNAL(signal_api_getlimitdata()), robotControler,SLOT(getlimitdata()));
    this->connect(this, SIGNAL(signal_api_atizero()), robotControler, SLOT(atizero()));
    this->connect(this, SIGNAL(signal_api_atimode(int,double,double,double,double,double,double,double)),robotControler, SLOT(atimode(int,double,double,double,double,double,double,double)));
    this->connect(this, SIGNAL(signal_api_RobotMove(int,int)),robotControler,SLOT(Robothandlemode(int,int)));
    this->connect(this, SIGNAL(signal_api_requestrealtimedata()),robotControler, SLOT(response_realtimedata()));
    this->connect(this, SIGNAL(signal_api_requestcommandfeedbackdata()),robotControler, SLOT(response_commanddata()));
    connect(robotControler,SIGNAL(signal_isRobotBlocked(bool)),this, SLOT(api_isRobotBlocked(bool)));
    connect(robotControler,SIGNAL(signal_isRobotConnected(bool)),this, SLOT(api_isRobotConnected(bool)));
    connect(robotControler,SIGNAL(signal_isRobotPowerOn(bool)),this, SLOT(api_isRobotPowerOn(bool)));
    connect(robotControler,SIGNAL(signal_isRobotError(bool)),this, SLOT(api_isRobotError(bool)));
    connect(robotControler,SIGNAL(signal_RobotWorkState(RobotWorkState)),this, SLOT(api_RobotWorkState(RobotWorkState)), Qt::DirectConnection);
    connect(robotControler,SIGNAL(signal_RobotWorkMode(RobotWorkMode)),this, SLOT(api_RobotWorkMode(RobotWorkMode)), Qt::DirectConnection);
    connect(robotControler,SIGNAL(signal_update_commanddata(RobotCommandFeedbackData)),this, SLOT(api_getcommandfeedbackdata(RobotCommandFeedbackData)), Qt::DirectConnection);
    connect(robotControler,SIGNAL(signal_update_realtimedata(RobotRealtimeData)),this, SLOT(api_getrealtimedata(RobotRealtimeData)), Qt::DirectConnection);

    //realtimedata_timer = new QTimer(this);
    //connect(realtimedata_timer, SIGNAL(timeout()), this, SLOT(startpolling()));

    newThread->start();
    qDebug()<<" --- ROBOTAPI START --- ";
}

RobotApi::~RobotApi()
{
    qDebug()<<"RobotApi::~RobotApi";
    newThread->requestInterruption();
    newThread->quit();
    newThread->wait();
    delete newThread;
    delete robotControler;
    //delete realtimedata_timer;
}

QString RobotApi::Version()
{
    qDebug()<<"RobotApi::Version()";
    qDebug()<< VERSION;
    return VERSION;
}

void RobotApi::api_isRobotBlocked(bool s)
{
    qDebug()<<"RobotApi::api_isRobotBlocked";
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    qDebug()<<current_time<<"api get RobotBlocked: "<<s;
    if(robot_state.isRobotBlocked != s)
    {
        emit signal_api_isRobotBlocked(s);
        robot_state.isRobotBlocked = s;
    }
}

void RobotApi::api_isRobotConnected(bool s)
{
    qDebug()<<"RobotApi::api_isRobotConnected";
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    qDebug()<<current_time<<"api get RobotConnect: "<<s;
    if(robot_state.isRobotConnected != s)
    {
        emit signal_api_isRobotConnected(s);
        robot_state.isRobotConnected = s;
    }
}

void RobotApi::api_isRobotPowerOn(bool s)
{
    qDebug()<<"RobotApi::api_isRobotPowerOn";
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    qDebug()<<current_time<<"api get PowerOnRobot: "<<s;
    if(robot_state.isRobotPowerOn != s)
    {
        emit signal_api_isRobotPowerOn(s);
        robot_state.isRobotPowerOn = s;
    }
}

void RobotApi::api_isRobotError(bool s)
{
    qDebug()<<"RobotApi::api_isRobotError";
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    qDebug()<<current_time<<"api get isRobotError: "<<s;
    if(robot_state.isRobotError != s)
    {
        emit signal_api_isRobotError(s);
        robot_state.isRobotError = s;
    }
}

void RobotApi::api_RobotWorkState(RobotWorkState ws)
{
    qDebug()<<"RobotApi::api_RobotWorkState";
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    qDebug()<<current_time<<"api get RobotWorkState: "<<ws;
    if(robot_state.robot_work_state != ws)
    {
        emit signal_api_RobotWorkState(ws);
        robot_state.robot_work_state = ws;
    }
}

void RobotApi::api_RobotWorkMode(RobotWorkMode wm)
{
    qDebug()<<"RobotApi::api_RobotWorkMode";
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    qDebug()<<current_time<<"api get RobotWorkState: "<<wm;
    if(robot_state.robot_work_mode != wm)
    {
        emit signal_api_RobotWorkMode(wm);
        robot_state.robot_work_mode = wm;
    }
}

void RobotApi::api_getrealtimedata(RobotRealtimeData d)
{
    qDebug()<<"RobotApi::api_getrealtimedata";
    //    QDateTime current_date_time = QDateTime::currentDateTime();
    //    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    apiRealtimedataMutex.lock();
    realtime_data = d;
    //    qDebug()<< "getrealtimedata time: " << current_time;
    apiRealtimedataMutex.unlock();
}

void RobotApi::api_getcommandfeedbackdata(RobotCommandFeedbackData sd)
{
    qDebug()<<"RobotApi::api_getcommandfeedbackdata";
    //    QDateTime current_date_time = QDateTime::currentDateTime();
    //    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    apiCommanddataMutex.lock();
    cammand_feedback_data = sd;
    //qDebug()<< "getcommandfeedbackdata: ";
    apiCommanddataMutex.unlock();
}

void RobotApi::connectrobot()
{
    qDebug()<<"RobotApi::connectrobot emit signal_api_connectrobot";
    emit signal_api_connectrobot();
}

void RobotApi::disconnectrobot()
{
    qDebug()<<"RobotApi::disconnectrobot emit signal_api_disconnectrobot";
    emit signal_api_disconnectrobot();
}

void RobotApi::poweron()
{
    qDebug()<<"RobotApi::poweron emit signal_api_poweron";
    emit signal_api_poweron();
}

void RobotApi::poweroff()
{
    qDebug()<<"RobotApi::poweroff emit signal_api_poweroff";
    emit signal_api_poweroff();
}

void RobotApi::setio(int b, int v)
{
    qDebug()<<"RobotApi::setio emit signal_api_setio";
    emit signal_api_setio(b,v);
}

void RobotApi::settcpinfo(TcpInfo t)
{
    qDebug()<<"RobotApi::settcpinfo emit signal_api_settcpinfo";
    emit signal_api_settcpinfo(t);
}

void RobotApi::gettcpinfo()
{
    qDebug()<<"RobotApi::gettcpinfo emit signal_api_gettcpinfo";
    emit signal_api_gettcpinfo();
}

void RobotApi::movej(double a, double b, double c, double d, double e, double f)
{
    qDebug()<<"RobotApi::movej emit signal_api_movej";
    emit signal_api_movej(a,b,c,d,e,f);
}

void RobotApi::movep(double a, double b, double c, double d, double e, double f)
{
    qDebug()<<"RobotApi::movep emit signal_api_movep";
    emit signal_api_movep(a,b,c,d,e,f);
}

void RobotApi::movel(double a, double b, double c, double d, double e, double f)
{
    qDebug()<<"RobotApi::movel emit signal_api_movel";
    emit signal_api_movel(a,b,c,d,e,f);
}

void RobotApi::stop()
{
    qDebug()<<"RobotApi::stop emit signal_api_stop";
    emit signal_api_stop();
}

void RobotApi::setspeed(double s)
{
    qDebug()<<"RobotApi::setspeed emit signal_api_setspeed";
    emit signal_api_setspeed(s);
}

void RobotApi::setworkmode(int s)
{
    qDebug()<<"RobotApi::setworkmode emit signal_api_setworkmode";
    emit signal_api_setworkmode(s);
}

void RobotApi::setlimitdata(TractionLimit tl)
{
    qDebug()<<"RobotApi::setlimitdata emit signal_api_setlimitdata";
    emit signal_api_setlimitdata(tl);
}

void RobotApi::getlimitdata()
{
    qDebug()<<"RobotApi::getlimitdata emit signal_api_getlimitdata";
    emit signal_api_getlimitdata();
}

void RobotApi::atizero()
{
    qDebug()<<"RobotApi::atizero emit signal_api_atizero";
    emit signal_api_atizero();
}

void RobotApi::atimode(int m, double para1, double para2, double para3,double para4,double para5 ,double para6,double para7)
{
    qDebug()<<"RobotApi::atimode emit signal_api_atimode";
    emit signal_api_atimode(m,para1,para2,para3,para4,para5,para6,para7);
}

void RobotApi::setTcpNum(int Joint, int Direction)
{
    qDebug()<<"RobotApi::atimode emit signal_api_atimode";
    emit signal_api_RobotMove(Joint,Direction);
}

void RobotApi::requestrealtimedata()
{
    qDebug()<<"RobotApi::requestrealtimedata emit signal_api_requestrealtimedata";
    //    QDateTime current_date_time = QDateTime::currentDateTime();
    //    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    emit signal_api_requestrealtimedata();
}

void RobotApi::requestcommandfeedbackdata()
{
    qDebug()<<"RobotApi::requestcommandfeedbackdata emit signal_api_requestcommandfeedbackdata";
    //    QDateTime current_date_time = QDateTime::currentDateTime();
    //    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    //    qDebug()<<"requestsecondarydata time: " << current_time;
    emit signal_api_requestcommandfeedbackdata();
}
