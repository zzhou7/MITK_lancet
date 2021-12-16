#include "robotcontroler.h"


const QString CMD_ERRORHANDLE = "$cmd,error,%1*";       // Handle error $res,error$  $res,error,-1$
const QString CMD_SETWM = "$cmd,setwm,%1*";               // $res,wmd,0$  $res,wmd,-1$
const QString CMD_POWERON = "$cmd,pon*";                // Powers on the robot arm, return: succ: $res,pon,0$ failed: $res,pon,-1$
const QString CMD_POWEROFF = "$cmd,off*";               // Powers off the robot arm, return:succ: $res,off,0$ failed: $res,off,-1$
const QString CMD_SETSPEED = "$cmd,spd,%1*";            // succ: $res,spd,para$ failed: $res,spd,-1$
const QString CMD_SETIO = "$cmd,sIO,%1,%2*";            // $res,sIO,0$ $res,sIO,-1$
const QString CMD_MOVEJ = "$cmd,mj,%1*";                // $res,mj,0$ $res,mj,-1$
const QString CMD_MOVEP = "$cmd,mp,%1*";                // $res,mp,0$ $res,mp,-1$
const QString CMD_MOVEL = "$cmd,ml,%1*";                // $res,ml,0$ $res,ml,-1$
const QString CMD_STOP = "$cmd,stop*";                  // $res,stop,0$ $res,stop,-1$
const QString CMD_SETTCP = "$cmd,stcp,%1*";             // $res,stcp,0$ $res,stcp,-1$
const QString CMD_GETTCP = "$cmd,qtcp*";                // $res,qtcp,type,weight,gcx,gcy,gcz,x,y,z,rx,ry,rz$ $res,qtcp,-1$
const QString CMD_SETLD = "$cmd,stld,%1*";              // $res,stld,0$ $res,stld,-1$
const QString CMD_GETLD = "$cmd,qtld*";                 // $res,qtld,angle,x1,y1,z1,x2,y2,z2$ $res,qtld,-1$
const QString CMD_ATIZERO = "$cmd,atiz*";               // $res,atiz,0$ $res,atiz,-1$
const QString CMD_ATIMODE = "$cmd,atiType,%1*";         // $res,atiType,0$ $res,atiType,-1$
const QString CMD_RobotMove="$cmd,RobotMove,%1*";

//const QString POLLINGDATA = "$";
const QString BREAKRELEASE = "brake release";       //Releases the brakes, return:"Brake releasing"
const QString SAFETYMODE = "safetymode";            //Safety mode enquiry, return:"Safetymode: <Safety_Mode>" @ robotdata.h
const QString UNLOCK = "unlock protective stop";    //Closes the current popup and unlocks protective stop, return:"Protective stop releasing"
const QString CLOSESAFEPOP = "close safety popup";  //Closes a safety popup, return:"closing safety popup"
const QString SHUTDOWN = "shutdown";                //Shuts down and turns off robot and controller, return:"Shutting down"
const QString QUIT = "quit";                        //Closes connection, return:"Disconnected"
const QString Joint2Str="%1,%2,%3,%4,%5,%6";
const QString Pose2Str="%1,%2,%3,%4,%5,%6";
const QString staubliPose2Str="%1,%2,%3,%4,%5,%6";
const QString staubliJoint2Str="%1,%2,%3,%4,%5,%6";
const QString CoG2Str = "%1,%2,%3";
const QString SetDigitalOutTrue2Str = "sec set_digital_out_true():\n set_digital_out(%1,True)\nend";
const QString SetDigitalOutFalse2Str = "sec set_digital_out_false():\n set_digital_out(%1,False)\nend";
const QString SetPayload2Str="set_payload(%1,%2)";
const QString SetPayloadCog2Str="set_payload_cog(%1)";
const QString SetPayloadMass2Str="set_payload_mass(%1)";
const QString SetGravity2Str="set_gravity(%1)";
const QString SetTcp2Str = "[%1,%2,%3,%4,%5,%6,%7]";
const QString Speed2Str = "%1,%2,%3"; // set speed
const QString StopJ2Str="stopj(%1)";
const QString StopL2Str="stopl(%1)";
const QString MoveC2Str="movec(%1,%2,a=%3,v=%4,r=%5)";
const QString MoveJWithRT2Str="movej(%1,a=%2,v=%3,t=%4,r=%5)";
const QString staubliMoveJWithRT2Str="(%1,%2)";
const QString MoveLWithRT2Str="(%1,%2)";
const QString MoveJWithoutRT2Str="movej(%1,a=%2,v=%3)";
const QString MoveLWithoutRT2Str="movel(%1,a=%2,v=%3)";
const QString MoveP2Str = "movep(%1,a=%2,v=%3,r=%4)";
const QString SpeedJ2Str="speedj(%1,a=%2,t=%3)";
const QString ForcdMode2Str = "force mode(1%,2%,3%,4%,5%)";
const QString SpeedL2Str="speedl(1%,2%,3%,aRot=’a’)";
const QString Sleep2Str="sleep(1%)";
const QString RT_EndFcMode="end_force_mode()";
const QString RT_EndFdMode="end_freedrive_mode()";
const QString RT_EndTcMode="end_teach_mode()";
const QString RT_SrtFdMode="freedrive_mode()";
const QString RT_SrtTcMode="teach_mode()";
const QString RT_GetForce="force()";
const QString RT_PwrDown="powerdown()";
const QString RT_Sync="sync()";


RobotControler::RobotControler(QObject *parent, qintptr p) : QThread(parent)
{
    qDebug()<<"RobotControler::RobotControler";
    qDebug()<<__FUNCTION__<<"ThreadId: "<<thread()->currentThreadId();
    this->ptr=p;
    this->connect(this,SIGNAL(signal_commanddata(DataMap)),this,SLOT(handle_command_data(DataMap)));
    this->connect(this,SIGNAL(signal_realtimedata(DataMap)),this,SLOT(handle_realtime_data(DataMap)));
    robot_state_pop.clear();
    newcommandsk = false;
    newrealtimesk = false;
}

RobotControler::~RobotControler()
{
    qDebug()<<"RobotControler::~RobotControler";
    disconnectrobot();
    this->requestInterruption();
    this->quit();
    this->wait();
    robot_state_pop.clear();
    newcommandsk = false;
    newrealtimesk = false;
    //    qDebug()<<"release urcontroler";
}

void RobotControler::RunRobotControler()
{
    qDebug()<<"RobotControler::RunRobotControler";
    qDebug()<<__FUNCTION__<< "Controler: Hello Lancet, ThreadId: "<<thread()->currentThreadId();
}

void RobotControler::run_robot_command(bool t)
{
    qDebug()<<"RobotControler::run_robot_command";
    //    qDebug()<<__FUNCTION__<<thread()->currentThreadId();
    if(t)
    {
        newcommandsk = true;
        robot_command=new RobotSocket(0,this->ptr);
        connect(robot_command, SIGNAL(connected()),this,SLOT(isCConnected()));
        connect(robot_command, SIGNAL(disconnected()),this,SLOT(disConnected()));
        connect(robot_command, SIGNAL(socket_data(QByteArray)),this, SLOT(readPendingCmdFeedbackData(QByteArray)));
        connect(this, SIGNAL(signal_sendcommand(QString)),robot_command, SLOT(socketWriteData(QString)));
        robot_command->connectToHost(ROBOT_SERVER_ADDRESS, ROBOT_COMMAND_PORT);
        //robot_command->waitForConnected(1000);
    }
    else
    {
        if(newcommandsk)
        {
            robot_command->disconnectFromHost();
            qDebug()<<"Command socket disconnected";
            delete robot_command;
            newcommandsk = false;
        }
        else
            qDebug()<<"There is no command socket";
    }
}

void RobotControler::run_robot_realtimedata(bool t)
{
    qDebug()<<"RobotControler::run_robot_realtimedata";
    //    qDebug()<<__FUNCTION__<<thread()->currentThreadId();
    if(t)
    {
        newrealtimesk = true;
        robot_realtimedata=new RobotSocket(0,this->ptr);
        connect(robot_realtimedata, SIGNAL(connected()),this,SLOT(isRConnected()));
        connect(robot_realtimedata, SIGNAL(disconnected()),this,SLOT(disConnected()));
        connect(robot_realtimedata, SIGNAL(socket_data(QByteArray)),this, SLOT(readPendingRealtimeData(QByteArray)));
        //connect(this, SIGNAL(signal_datapolling(QString)),robot_realtimedata, SLOT(socketWriteData(QString)));
        robot_realtimedata->connectToHost(ROBOT_SERVER_ADDRESS, ROBOT_DATA_PORT);
        //robot_realtimedata->waitForConnected(1000);
    }
    else
    {
        if(newrealtimesk)
        {
            robot_realtimedata->disconnectFromHost();
            delete robot_realtimedata;
            newrealtimesk = false;
            qDebug()<<"Realtimedata socket disconnected";
        }
        else
            qDebug()<<"There is no realtimedata socket";
    }
}

void RobotControler::isConnected()
{
    qDebug()<<"RobotControler::isConnected";
    robot_state_pop.socketnum ++;
    if(robot_state_pop.socketnum == 2)
        update_isRobotConnected(true);
}

void RobotControler::isCConnected()
{
    qDebug()<<"RobotControler::isCConnected";
    robot_state_pop.isCommandConnected = true;
    if(robot_state_pop.isRealtimedataConnected)
        update_isRobotConnected(true);
    else
        qDebug() << "Realtimedata Socket is not Connected yet";
}
void RobotControler::isRConnected()
{
    qDebug()<<"RobotControler::isRConnected";
    qDebug()<<"Realtime Socket Connected";
    robot_state_pop.isRealtimedataConnected = true;
    if(robot_state_pop.isCommandConnected)
        update_isRobotConnected(true);
    else
        qDebug() << "Command Socket is not Connected yet";
}

void RobotControler::disConnected()
{
    qDebug()<<"RobotControler::disConnected";
    update_isRobotConnected(false);
    update_isRobotBlocked(false);
    update_isRobotPowerOn(false);
    update_isRobotError(false);
    robot_state_pop.clear();
}

void RobotControler::readPendingCmdFeedbackData(QByteArray r)
{
    qDebug()<<"RobotControler::readPendingCmdFeedbackData emit signal_commanddata";
    datamap_command.data_array.clear();
    datamap_command.data_array.operator +=(r);
    qDebug()<< __FUNCTION__ << r;
    emit signal_commanddata(datamap_command);
}

void RobotControler::readPendingRealtimeData(QByteArray r)
{
    qDebug()<<"RobotControler::readPendingRealtimeData";
    //realtime_mutex.lock();
    datamap_realtime.data_array.clear();
    datamap_realtime.data_array.operator +=(r);
    //qDebug()<< __FUNCTION__ << r;
    emit signal_realtimedata(datamap_realtime);
    //realtime_mutex.unlock();
}

bool RobotControler::getIsRobotBlocked()
{
    qDebug()<<"RobotControler::getIsRobotBlocked";
    return this->robot_state_pop.isRobotBlocked;
}

bool RobotControler::getIsRobotConnected()
{
    qDebug()<<"RobotControler::getIsRobotConnected";
    return this->robot_state_pop.isRobotConnected;
}

bool RobotControler::getIsRobotPowerOn()
{
    qDebug()<<"RobotControler::getIsRobotPowerOn";
    return this->robot_state_pop.isRobotPowerOn;
}

bool RobotControler::getIsRobotError()
{
    qDebug()<<"RobotControler::getIsRobotError";
    return this->robot_state_pop.isRobotError;
}

RobotWorkState RobotControler::getRobotWorkState()
{
    qDebug()<<"RobotControler::getRobotWorkState";
    return this->robot_state_pop.robot_work_state;
}

RobotWorkMode RobotControler::getRobotWorkMode()
{
    qDebug()<<"RobotControler::getRobotWorkMode";
    return this->robot_state_pop.robot_work_mode;
}

void RobotControler::SendCommand(RobotCommandSendData m_commanddata)
{
    qDebug()<<"RobotControler::SendCommand";
    if(robot_state_pop.isRobotConnected)
    {
        switch (m_commanddata.robot_command_type)
        {
        case POWERON:
            qDebug() << "emit Power On";
            emit signal_sendcommand(CMD_POWERON);
            break;
        case POWEROFF:
            qDebug() << "emit Power Off";
            emit signal_sendcommand(CMD_POWEROFF);
            break;
        case SETSPEED:
            qDebug() << "emit Set speed";
            emit signal_sendcommand(CMD_SETSPEED.arg(m_commanddata.robot_speed));
            break;
        case SETWM:
            qDebug() << "emit Set workmode";
            emit signal_sendcommand(CMD_SETWM.arg(m_commanddata.robot_workmode));
            break;
        case SETIO:
            qDebug() << "emit Set IO";
            emit signal_sendcommand(CMD_SETIO.arg(m_commanddata.robot_io.bit).arg(m_commanddata.robot_io.valu1e));
            break;
        case MOVEJ:
            qDebug() << "emit MoveJ";
            emit signal_sendcommand(CMD_MOVEJ.arg(m_commanddata.JointString));
            break;
        case MOVEP:
            qDebug() << "emit MoveP";
            emit signal_sendcommand(CMD_MOVEP.arg(m_commanddata.PointString));
            break;
        case MOVEL:
            qDebug() << "emit MoveL";
            emit signal_sendcommand(CMD_MOVEL.arg(m_commanddata.PointString));
            break;
        case STOP:
            qDebug() << "emit Stop!";
            emit signal_sendcommand(CMD_STOP);
            break;
        case SETTCPINFO:
            qDebug() << "emit Set Tcp";
            emit signal_sendcommand(CMD_SETTCP.arg(m_commanddata.TcpString));
            break;
        case GETTCPINFO:
            qDebug() << "emit Get Tcp";
            emit signal_sendcommand(CMD_GETTCP);
            break;
        case SETLD:
            qDebug() << "emit Set Traction Limit Data";
            emit signal_sendcommand(CMD_SETLD.arg(m_commanddata.LimitdataString));
            break;
        case GETLD:
            qDebug() << "emit Get Traction Limit Data";
            emit signal_sendcommand(CMD_GETLD);
            break;
        case ATIZERO:
            qDebug() << "emit ATI Zero Calibration";
            emit signal_sendcommand(CMD_ATIZERO);
            break;
        case ATIMODE:
            qDebug() << "emit SET ATI MODE";
            emit signal_sendcommand(CMD_ATIMODE.arg(m_commanddata.AtiModeParaString));
            break;
        case RobotMove:
            qDebug()<<"emit SET Direaction and jointNum";
            emit signal_sendcommand(CMD_RobotMove.arg(m_commanddata.RobotMoveString));
            break;
        default:
            break;
        }
    }
    else
        qDebug() << "Connect Socket First.";
}

void RobotControler::movej(double a, double b, double c, double d, double e, double f)
{
    qDebug()<<"RobotControler::movej";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::MOVEJ;
    command_send_data.robot_joint.j1 = a;command_send_data.robot_joint.j2 = b;command_send_data.robot_joint.j3 = c;
    command_send_data.robot_joint.j4 = d;command_send_data.robot_joint.j5 = e;command_send_data.robot_joint.j6 = f;
    command_send_data.set_joints_string();
    SendCommand(command_send_data);
}

void RobotControler::movep(double a, double b, double c, double d, double e, double f)
{
    qDebug()<<"RobotControler::movep";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::MOVEP;
    command_send_data.robot_point.x = a;command_send_data.robot_point.y = b;command_send_data.robot_point.z = c;
    command_send_data.robot_point.a = d;command_send_data.robot_point.b = e;command_send_data.robot_point.c = f;
    command_send_data.set_points_string();
    SendCommand(command_send_data);
}

void RobotControler::movel(double a, double b, double c, double d, double e, double f)
{
    qDebug()<<"RobotControler::movel";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::MOVEL;
    command_send_data.robot_point.x = a;command_send_data.robot_point.y = b;command_send_data.robot_point.z = c;
    command_send_data.robot_point.a = d;command_send_data.robot_point.b = e;command_send_data.robot_point.c = f;
    command_send_data.set_points_string();
    SendCommand(command_send_data);
}

void RobotControler::stop()
{
    qDebug()<<"RobotControler::stop";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::STOP;
    SendCommand(command_send_data);
}

void RobotControler::settcpinfo(TcpInfo t)
{
    qDebug()<<"RobotControler::settcpinfo";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::SETTCPINFO;
    command_send_data.robot_tcp = t;
    command_send_data.set_tcpinfo_string();
    SendCommand(command_send_data);
}

void RobotControler::gettcpinfo()
{
    qDebug()<<"RobotControler::gettcpinfo";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::GETTCPINFO;
    SendCommand(command_send_data);
}

void RobotControler::setspeed(double s)
{
    qDebug()<<"RobotControler::setspeed";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::SETSPEED;
    command_send_data.robot_speed = s;
    SendCommand(command_send_data);
}

void RobotControler::setworkmode(int s)
{
    qDebug()<<"RobotControler::setworkmode";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::SETWM;
    command_send_data.robot_workmode = s;
    SendCommand(command_send_data);
}

void RobotControler::setlimitdata(TractionLimit tl)
{
    qDebug()<<"RobotControler::setlimitdata";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::SETLD;
    command_send_data.robot_limitdata = tl;
    command_send_data.set_limitdata_string();
    SendCommand(command_send_data);
}

void RobotControler::getlimitdata()
{
    qDebug()<<"RobotControler::getlimitdata";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::GETLD;
    SendCommand(command_send_data);
}

void RobotControler::atizero()
{
    qDebug()<<"RobotControler::atizero";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::ATIZERO;
    SendCommand(command_send_data);
}

void RobotControler::atimode(int m, double para1, double para2, double para3,double para4,double para5,double para6,double para7)
{
    qDebug()<<"RobotControler::atimode";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::ATIMODE;
    command_send_data.robot_ati_mode.atimode = m;
    command_send_data.robot_ati_mode.para1 = para1;
    command_send_data.robot_ati_mode.para2 = para2;
    command_send_data.robot_ati_mode.para3 = para3;
    command_send_data.robot_ati_mode.para4 = para4;
    command_send_data.robot_ati_mode.para5 = para5;
    command_send_data.robot_ati_mode.para6 = para6;
    command_send_data.robot_ati_mode.para7 = para7;
    command_send_data.set_atimode_string();
    qDebug()<<"get ati mode data";
    SendCommand(command_send_data);
}

void RobotControler::Robothandlemode(int joint, int Direction)
{
    qDebug()<<"RobotControler::Robothandlemode";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::RobotMove;
   // command_send_data.robot_ati_mode.atimode = m;
    command_send_data.robot_handle_move.Jointmode=joint;
    command_send_data.robot_handle_move.para1=Direction;
    command_send_data.set_RobotMove_string();
    SendCommand(command_send_data);
}


void RobotControler::connectrobot()
{
    qDebug()<<"RobotControler::connectrobot";
    if(robot_state_pop.isRobotConnected)
        qDebug()<<"All Socket is Connected";
    else
    {
        qDebug()<<"Start Socket Connecting...";
        run_robot_command(true);
        run_robot_realtimedata(true);
    }
}

void RobotControler::disconnectrobot()
{
    qDebug()<<"RobotControler::disconnectrobot";
    run_robot_command(false);
    run_robot_realtimedata(false);
    robot_state_pop.clear();
}

void RobotControler::poweron()
{
    qDebug()<<"RobotControler::poweron";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::POWERON;
    SendCommand(command_send_data);
}

void RobotControler::poweroff()
{
    qDebug()<<"RobotControler::poweroff";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::POWEROFF;
    SendCommand(command_send_data);
}

void RobotControler::setio(int b, int v)
{
    qDebug()<<"RobotControler::setio";
    command_send_data.init();
    command_send_data.robot_command_type = CommandType::SETIO;
    command_send_data.robot_io.bit = b;
    command_send_data.robot_io.valu1e = v;
    SendCommand(command_send_data);
}

void RobotControler::handle_command_data(DataMap cmdresponse)
{
    qDebug()<<"RobotControler::handle_command_data";
    //qDebug() << __FUNCTION__ ;
    update_isRobotBlocked(false);
    c_command_feedback_data.clear();
    c_command_feedback_data = cmdresponse;
    //command_mutex.lock();
    c_command_feedback_data.data_string.prepend(c_command_feedback_data.data_array);
    //command_mutex.unlock();
    //qDebug()<< c_command_feedback_data.data_string.left(1);
    c_command_feedback_data.data_string.remove(0,5); // remove '$res,'
    //qDebug() << c_staubli_data_command.data_string;
    c_command_feedback_data.data_string.remove(c_command_feedback_data.data_string.size()-2,2); //remove '$\r'
    //TODO
    //qDebug() << c_staubli_data_command.data_string;
    QStringList tmpList = c_command_feedback_data.data_string.split(",");
    //    //int listnum = tmpList.size();
    qDebug() << tmpList;
    if(tmpList.at(0).toStdString() == "qtcp")
    {
        if( tmpList.size() == 12)
        {
            qDebug() << "get tcpinfo Successfully";
            temp_command_feedback_data.tcpinfo.tcptype = TcpType(tmpList.at(1).toInt());
            temp_command_feedback_data.tcpinfo.cog.weight = tmpList.at(2).toDouble();
            temp_command_feedback_data.tcpinfo.cog.x = tmpList.at(3).toDouble();
            temp_command_feedback_data.tcpinfo.cog.y = tmpList.at(4).toDouble();
            temp_command_feedback_data.tcpinfo.cog.z = tmpList.at(5).toDouble();
            temp_command_feedback_data.tcpinfo.tcp.x = tmpList.at(6).toDouble();
            temp_command_feedback_data.tcpinfo.tcp.y = tmpList.at(7).toDouble();
            temp_command_feedback_data.tcpinfo.tcp.z = tmpList.at(8).toDouble();
            temp_command_feedback_data.tcpinfo.tcp.a = tmpList.at(9).toDouble();
            temp_command_feedback_data.tcpinfo.tcp.b = tmpList.at(10).toDouble();
            temp_command_feedback_data.tcpinfo.tcp.c = tmpList.at(11).toDouble();
        }
        else
            qDebug() << "get tcpinfo Failed";
    }

    if(tmpList.at(0).toStdString() == "mj" && tmpList.size() == 2)
    {
        if(tmpList.at(1) == "0")
            qDebug() << "MoveJ Successfully!";
        else
            qDebug() << "MoveJ Failed!";
    }

    if(tmpList.at(0).toStdString() == "mp" && tmpList.size() == 2)
    {
        if(tmpList.at(1) == "0")
            qDebug() << "MoveP Successfully!";
        else
            qDebug() << "MoveP Failed!";
    }

    if(tmpList.at(0).toStdString() == "ml" && tmpList.size() == 2)
    {
        if(tmpList.at(1) == "0")
            qDebug() << "MoveL Successfully!";
        else
            qDebug() << "MoveL Failed!";
    }

    if(tmpList.at(0).toStdString() == "qtld")
    {
        if(tmpList.size() == 8)
        {
            qDebug() << "get traction limit data Successfully";
            temp_command_feedback_data.limitdata.angle = tmpList.at(1).toDouble();
            temp_command_feedback_data.limitdata.p1.x = tmpList.at(2).toDouble();
            temp_command_feedback_data.limitdata.p1.y = tmpList.at(3).toDouble();
            temp_command_feedback_data.limitdata.p1.z = tmpList.at(4).toDouble();
            temp_command_feedback_data.limitdata.p2.x = tmpList.at(5).toDouble();
            temp_command_feedback_data.limitdata.p2.y = tmpList.at(6).toDouble();
            temp_command_feedback_data.limitdata.p2.z = tmpList.at(7).toDouble();
        }
        else
            qDebug() << "get traction limit data Failed";
    }
    //qDebug() << tmpList;    // todo

    //    command_mutex.lock();
    command_controler_feedback_data = temp_command_feedback_data;
    //    command_mutex.unlock();
}

//    if(c_command_feedback_data.data_type == 1 && tmpList.at(2).toInt() == 0)
//        update_isRobotBlocked(true);
//    if(c_command_feedback_data.data_type == 1 && tmpList.at(2).toInt() == 1)
//        update_isRobotBlocked(false);
//    if(c_command_feedback_data.data_type == 2 && tmpList.at(0).toInt() == 1 && tmpList.at(1).toInt() == 1 && tmpList.at(2).toInt() == 0)
//        update_isRobotBlocked(true);
//    if(c_command_feedback_data.data_type == 2 && tmpList.at(0).toInt() == 1 && tmpList.at(1).toInt() == 1 && tmpList.at(2).toInt() == 1)
//    {
//        update_isRobotBlocked(false);
//    }
//    if(c_command_feedback_data.data_type == 2 && tmpList.at(0).toInt() == 1 && tmpList.at(1).toInt() == 2 && tmpList.at(2).toInt() == 0)
//        update_isRobotBlocked(true);
//    if(c_command_feedback_data.data_type == 2 && tmpList.at(0).toInt() == 1 && tmpList.at(1).toInt() == 2 && tmpList.at(2).toInt() == 1)
//    {
//        update_isRobotBlocked(false);
//    }
//    //return command_controler_feedback_data;

void RobotControler::handle_realtime_data(DataMap rtdata)
{
    qDebug()<<"RobotControler::handle_realtime_data";
    //qDebug() << __FUNCTION__ ;
    //realtime_mutex.lock();
    c_realtime_data.clear();
    c_realtime_data = rtdata;
    c_realtime_data.data_string.prepend(c_realtime_data.data_array);
    //realtime_mutex.unlock();
    c_realtime_data.data_string.remove(0,1); //remove first $
    //qDebug() << c_realtime_data.data_string;
    c_realtime_data.data_string.remove(c_realtime_data.data_string.size()-2,2); // remove last $ and \r
    //qDebug() << c_staubli_data_realtime.data_string;
    QStringList tmpList = c_realtime_data.data_string.split(",");
    if(tmpList.at(0).toInt() == 0 && tmpList.size() >= 19)
    {
        temp_realtime_data.datatype = ROBOTDATA;
        temp_realtime_data.errorcode = RobotErrorCode(tmpList.at(1).toInt());
        temp_realtime_data.workmode = RobotWorkMode(tmpList.at(2).toInt());
        update_RobotWorkMode(temp_realtime_data.workmode);
        update_isRobotPowerOn(bool(tmpList.at(3).toInt()));
        temp_realtime_data.speed = tmpList.at(4).toDouble();
        temp_realtime_data.io.valu1e = tmpList.at(5).toInt();
        temp_realtime_data.joints.j1 = tmpList.at(6).toDouble();
        temp_realtime_data.joints.j2 = tmpList.at(7).toDouble();
        temp_realtime_data.joints.j3 = tmpList.at(8).toDouble();
        temp_realtime_data.joints.j4 = tmpList.at(9).toDouble();
        temp_realtime_data.joints.j5 = tmpList.at(10).toDouble();
        temp_realtime_data.joints.j6 = tmpList.at(11).toDouble();
        temp_realtime_data.joints.j7 = tmpList.at(12).toDouble();
        temp_realtime_data.pose.x = tmpList.at(13).toDouble();
        temp_realtime_data.pose.y = tmpList.at(14).toDouble();
        temp_realtime_data.pose.z = tmpList.at(15).toDouble();
        temp_realtime_data.pose.a = tmpList.at(16).toDouble();
        temp_realtime_data.pose.b = tmpList.at(17).toDouble();
        temp_realtime_data.pose.c = tmpList.at(18).toDouble();
        //temp_realtime_data.io.bit1 = bool(tmpList.at(17).toInt());
        //temp_realtime_data.io.bit2 = bool(tmpList.at(18).toInt());
    }
    else if(tmpList.size() >= 7)
    {
        temp_realtime_data.datatype = ATIDATA;
        temp_realtime_data.ati.status = true;
        //temp_realtime_data.ati.status = bool(tmpList.at(1).toInt());
        temp_realtime_data.ati.fx = tmpList.at(1).toDouble();
        temp_realtime_data.ati.fy = tmpList.at(2).toDouble();
        temp_realtime_data.ati.fz = tmpList.at(3).toDouble();
        temp_realtime_data.ati.mx = tmpList.at(4).toDouble();
        temp_realtime_data.ati.my = tmpList.at(5).toDouble();
        temp_realtime_data.ati.mz = tmpList.at(6).toDouble();
    }
    else
        qDebug() << tmpList;

    realtime_mutex.lock();
    controler_realtime_data = temp_realtime_data;
    realtime_mutex.unlock();

    // todo
    //return controler_realtime_data;
}

void RobotControler::update_isRobotBlocked(bool s)
{
    qDebug()<<"RobotControler::update_isRobotBlocked emit signal_isRobotBlocked";
    if(robot_state_pop.isRobotBlocked != s)
    {
        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
        qDebug()<<current_time<<"Controler pop isRobotBlocked: "<<s;
        robot_state_pop.isRobotBlocked = s;
        emit signal_isRobotBlocked(s);
    }
}

void RobotControler::update_isRobotConnected(bool s)
{
    qDebug()<<"RobotControler::update_isRobotConnected emit signal_isRobotConnected";
    if(robot_state_pop.isRobotConnected != s)
    {
        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
        qDebug()<<current_time<<"Controler pop isRobotConnected: "<<s;
        robot_state_pop.isRobotConnected = s;
        emit signal_isRobotConnected(s);
    }
}

void RobotControler::update_isRobotPowerOn(bool s)
{
    qDebug()<<"RobotControler::update_isRobotPowerOn emit signal_isRobotPowerOn";
    if(robot_state_pop.isRobotPowerOn != s)
    {
        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
        robot_state_pop.isRobotPowerOn = s;
        emit signal_isRobotPowerOn(s);
        qDebug()<<current_time<<"Controler pop isRobotPowerOn: "<<s;
    }
}

void RobotControler::update_isRobotError(bool s)
{
    qDebug()<<"RobotControler::update_isRobotError emit signal_isRobotError";
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
    if(robot_state_pop.isRobotError != s)
    {
        robot_state_pop.isRobotError = s;
        emit signal_isRobotError(s);
        qDebug()<<current_time<<"Controler pop isRobotError: "<<s;
    }
}

void RobotControler::update_RobotWorkState(RobotWorkState ws)
{
    qDebug()<<"RobotControler::update_isRobotError";
    if(robot_state_pop.robot_work_state != ws)
    {
        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
        qDebug()<<current_time<<"Controler pop RobotWorkState: "<<ws;
        robot_state_pop.robot_work_state = ws;
        qDebug()<<"emit signal_RobotWorkState";
        emit signal_RobotWorkState(ws);
    }
}

void RobotControler::update_RobotWorkMode(RobotWorkMode wm)
{
    qDebug()<<"RobotControler::update_RobotWorkMode";
    if(robot_state_pop.robot_work_mode != wm)
    {
        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_time = current_date_time.toString("hh:mm:ss.zzz ");
        qDebug()<<current_time<<"Controler pop RobotWorkMode: "<<wm;
        robot_state_pop.robot_work_mode = wm;
        qDebug()<<"emit signal_RobotWorkMode";
        emit signal_RobotWorkMode(wm);
    }
}

void RobotControler::response_commanddata()
{
    qDebug()<<"RobotControler::response_commanddata";
    //temp_command_data = handle_command_data(datamap_command);
    //emit signal_commanddata(temp_command_data);
    command_mutex.lock();
    emit signal_update_commanddata(command_controler_feedback_data);
    command_mutex.unlock();
}

void RobotControler::response_realtimedata()
{
    qDebug()<<"RobotControler::response_realtimedata emit signal_update_realtimedata";
    //temp_realtime_data = handle_realtime_data();
    //    qDebug()<< __FUNCTION__ ;
    //emit signal_realtimedata(temp_realtime_data);
    realtime_mutex.lock();
    emit signal_update_realtimedata(controler_realtime_data);
    realtime_mutex.unlock();
}

//void RobotControler::update_commanddata()
//{
////    qDebug()<<"update_commanddata";
//    handle_command_data(datamap_command);
//}

//void RobotControler::update_realtimedata()
//{
////    qDebug()<<"update_realtimedata";
//    handle_realtime_data();
//}

