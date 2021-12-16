/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "RobotView.h"

// Qt
#include <QMessageBox>
#include <QPushButton>
// mitk image
#include <mitkImage.h>

const std::string RobotView::VIEW_ID = "org.mitk.views.robotview";

void RobotView::SetFocus()
{
  m_Controls.SocketConnect->setFocus();
}

RobotView::~RobotView()
{
    m_runTread = false;
    m_robotApi->disconnectrobot();
    //delete m_robotApi;
}

void RobotView::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);
  // timer
  m_robotApi = new RobotApi;
  ui_realtimedata_timer = new QTimer(this);
  connect(ui_realtimedata_timer, SIGNAL(timeout()), this, SLOT(update_realtime_data()));

  ui_automove_timer = new QTimer(this);
  connect(ui_automove_timer, SIGNAL(timeout()), this, SLOT(robot_auto_move()));

  m_Controls.robotconnectstate->setText("Robot Disconnect");
  //ui->powerstate->setText("PowerOff");
  m_Controls.errorstate->setText("NO ERROR");
  m_Controls.robotworkmode->setText("UNKNOWN");

  connect(m_Controls.SocketConnect, SIGNAL(clicked()), this, SLOT(on_SocketConnect_clicked()));
  connect(m_Controls.SocketDisConnect, SIGNAL(clicked()), this, SLOT(on_SocketDisConnect_clicked()));
  //igt = IGTController::getInStance();
  connect(m_robotApi, SIGNAL(signal_api_isRobotConnected(bool)), this, SLOT(ui_isRobotConnected(bool)));
  connect(m_robotApi, SIGNAL(signal_api_isRobotPowerOn(bool)), this, SLOT(ui_isRobotPowerOn(bool)));
  connect(m_robotApi, SIGNAL(signal_api_isRobotError(bool)), this, SLOT(ui_isRobotError(bool)));
  connect(m_robotApi, SIGNAL(signal_api_isRobotBlocked(bool)), this, SLOT(ui_isRobotBlocked(bool)));
  //connect(staubli, SIGNAL(signal_api_RobotWorkState(RobotWorkState)), this, SLOT(ui_RobotWorkState(RobotWorkState)), Qt::DirectConnection);
  connect(m_robotApi,
          SIGNAL(signal_api_RobotWorkMode(RobotWorkMode)),
          this,
          SLOT(ui_RobotWorkMode(RobotWorkMode)),
          Qt::DirectConnection);
  connect(m_robotApi,
          SIGNAL(signal_api_atimode(int, double, double, double)),
          this,
          SLOT(ui_ATIMode(RobotWorkMode)),
          Qt::DirectConnection);
  cubeSidelength = 50.0;
  cube_point_number = 0;
  initial_point_ready = false;
  move_rand_index = 0;

  //islimit = false;
  //isspeed = false;
  isIO = false;
  settcp = 1;

  ATIValue.push_back(0);
  JointValue.push_back(0);
  TCPValue.push_back(0);

  m_MultiThreader = itk::MultiThreader::New();
  m_runTread = true;
  m_ThreadID = m_MultiThreader->SpawnThread(this->HeartBeatThread, this);
}

void RobotView::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
                                   const QList<mitk::DataNode::Pointer> &nodes)
{
  // iterate all selected objects, adjust warning visibility
  foreach(mitk::DataNode::Pointer node, nodes)
  {
    if (node.IsNotNull() && dynamic_cast<mitk::Image *>(node->GetData()))
    {
      //m_Controls.labelWarning->setVisible(false);
      //m_Controls.buttonPerformImageProcessing->setEnabled(true);
      return;
    }
  }

  //m_Controls.labelWarning->setVisible(true);
  //m_Controls.buttonPerformImageProcessing->setEnabled(false);
}

/*void RobotView::DoImageProcessing()
{
  QList<mitk::DataNode::Pointer> nodes = this->GetDataManagerSelection();
  if (nodes.empty())
    return;

  mitk::DataNode *node = nodes.front();

  if (!node)
  {
    // Nothing selected. Inform the user and return
    QMessageBox::information(nullptr, "Template", "Please load and select an image before starting image processing.");
    return;
  }

  // here we have a valid mitk::DataNode

  // a node itself is not very useful, we need its data item (the image)
  mitk::BaseData *data = node->GetData();
  if (data)
  {
    // test if this data item is an image or not (could also be a surface or something totally different)
    mitk::Image *image = dynamic_cast<mitk::Image *>(data);
    if (image)
    {
      std::stringstream message;
      std::string name;
      message << "Performing image processing for image ";
      if (node->GetName(name))
      {
        // a property called "name" was found for this DataNode
        message << "'" << name << "'";
      }
      message << ".";
      MITK_INFO << message.str();

      // actually do something here...
    }
  }
}*/

void RobotView::update_realtime_data()
{
  m_robotApi->requestrealtimedata();
  ui_realtime_data = m_robotApi->realtime_data;
  m_Controls.j0->setText(QString::number(ui_realtime_data.joints.j1, 10, 3));
  m_Controls.j1->setText(QString::number(ui_realtime_data.joints.j2, 10, 3));
  m_Controls.j2->setText(QString::number(ui_realtime_data.joints.j3, 10, 3));
  m_Controls.j3->setText(QString::number(ui_realtime_data.joints.j4, 10, 3));
  m_Controls.j4->setText(QString::number(ui_realtime_data.joints.j5, 10, 3));
  m_Controls.j5->setText(QString::number(ui_realtime_data.joints.j6, 10, 3));
  m_Controls.j6->setText(QString::number(ui_realtime_data.joints.j7, 10, 3));

  m_Controls.x->setText(QString::number(ui_realtime_data.pose.x, 10, 3));
  m_Controls.y->setText(QString::number(ui_realtime_data.pose.y, 10, 3));
  m_Controls.z->setText(QString::number(ui_realtime_data.pose.z, 10, 3));
  m_Controls.a->setText(QString::number(ui_realtime_data.pose.a, 10, 3));
  m_Controls.b->setText(QString::number(ui_realtime_data.pose.b, 10, 3));
  m_Controls.c->setText(QString::number(ui_realtime_data.pose.c, 10, 3));
  //QString::number(ui_realtime_data.pose.c, 10, 3);
}

void RobotView::robot_auto_move()
{
  double para = qrand() % 30 - 15;
  m_robotApi->movej(-45.0 + para, 60.0 + para, 30.0 + para, 0.0 + para, 90.0 + para, 0.0 + para);
}

void RobotView::on_SocketConnect_clicked()
{
  MITK_INFO << "socketConnect clicked";
  m_robotApi->connectrobot();
  cubeSidelength = 50.0;
  cube_point_number = 0;
  initial_point_ready = false;
}

void RobotView::on_SocketDisConnect_clicked()
{
  ui_realtimedata_timer->stop();
  ui_automove_timer->stop();
  m_robotApi->disconnectrobot();
  cubeSidelength = 50.0;
  cube_point_number = 0;
  initial_point_ready = false;
}

void RobotView::on_PowerOn_clicked()
{
  m_robotApi->poweron();
}

void RobotView::on_PowerOff_clicked()
{
  m_robotApi->poweroff();
}

void RobotView::on_BackTestPose_clicked()
{
  m_robotApi->requestrealtimedata();
  ui_moveInitial_realtime_data = m_robotApi->realtime_data;
  ui_moveInitial_realtime_data.pose.x += 7;
  m_robotApi->movel(ui_moveInitial_realtime_data.pose.x,
                    ui_moveInitial_realtime_data.pose.y,
                    ui_moveInitial_realtime_data.pose.z,
                    ui_moveInitial_realtime_data.pose.a,
                    ui_moveInitial_realtime_data.pose.b,
                    ui_moveInitial_realtime_data.pose.c);
}

void RobotView::on_Testpose_clicked()
{
  m_robotApi->requestrealtimedata();
  ui_moveInitial_realtime_data = m_robotApi->realtime_data;
  ui_moveInitial_realtime_data.pose.x -= 7;
  m_robotApi->movel(ui_moveInitial_realtime_data.pose.x,
                    ui_moveInitial_realtime_data.pose.y,
                    ui_moveInitial_realtime_data.pose.z,
                    ui_moveInitial_realtime_data.pose.a,
                    ui_moveInitial_realtime_data.pose.b,
                    ui_moveInitial_realtime_data.pose.c);
}

void RobotView::ui_isRobotConnected(bool p)
{
  if (p)
  {
    m_Controls.robotconnectstate->setText("Robot Connected");
    qDebug() << "Robot Connected";
    ui_realtimedata_timer->start(100);
    m_heartBeat = true;
  }
  else
  {
    m_Controls.robotconnectstate->setText("Robot Disconnect");
    ui_realtimedata_timer->stop();
    qDebug() << "Robot Disconnect";
    m_heartBeat = false;
  }
}

void RobotView::ui_isRobotError(bool p) const
{
  if (p)
    m_Controls.errorstate->setText("Error!!");
  else
    m_Controls.errorstate->setText("NO Error");
}

void RobotView::ui_isRobotBlocked(bool p)
{
  //if(p)
  //    m_Controls.blockedstate->setText("Blocked: YES");
  //else
  //    m_Controls.blockedstate->setText("Blocked: NO");
}

void RobotView::ui_RobotWorkMode(RobotWorkMode wm)
{
  switch (wm)
  {
    case CONTROL_MODE_UNKNOWN:
      emit signalRobotWorkMode(CONTROL_MODE_UNKNOWN);
      m_Controls.robotworkmode->setText("WorkMode: UNKNOWN");
      break;
    case CONTROL_MODE_AUTO:
      emit signalRobotWorkMode(CONTROL_MODE_AUTO);
      //IGTController::staubilMode = CONTROL_MODE_AUTO;
      m_Controls.robotworkmode->setText("WorkMode: NORMAL");
      break;
    case CONTROL_MODE_FORCE_ATI_MODE1:
      emit signalRobotWorkMode(CONTROL_MODE_FORCE_ATI_MODE1);
      //IGTController::staubilMode = CONTROL_MODE_FORCE_ATI_MODE1;
      m_Controls.robotworkmode->setText("WorkMode: ATI_MODE");
      break;
    default:
      break;
  }
}

void RobotView::ui_ATIMode(int i, double rx, double ry, double rz)
{
  switch (i)
  {
    case 1:
      emit signalRobotWorkMode(CONTROL_MODE_FORCE_ATI_MODE1);
      m_Controls.robotworkmode->setText("WorkMode: ATI_MODE1");
      break;
    case 2:
      emit signalRobotWorkMode(CONTROL_MODE_FORCE_ATI_MODE2);
      m_Controls.robotworkmode->setText("WorkMode: ATI_MODE2");
      break;
    default:
      break;
  }
}

void RobotView::on_StartPollingData_clicked()
{
  if (m_robotApi->robot_state.isRobotConnected)
  {
    if (m_robotApi->robot_state.isRobotPowerOn)
    {
      m_robotApi->movej(-45.0, 60.0, 30.0, 0.0, 90.0, 0.0);
      qDebug() << "Set Initial Pose, Start Move Random!";
      ui_automove_timer->start(7000);
    }
    else
      qDebug() << "Power On Robot First!";
  }
  else
    qDebug() << "Connect Robot first";
}

void RobotView::on_StopPollingData_clicked()
{
  move_rand_index = 1;
  //staubli->stoppollingdata();
  qDebug() << "Stop Move Random!";
  ui_automove_timer->stop();
}

void RobotView::on_GetTcpInfo_clicked()
{
  m_robotApi->gettcpinfo();
  m_robotApi->requestcommandfeedbackdata();
  ui_commandfeedback_data = m_robotApi->cammand_feedback_data;
  qDebug() << "Tcp Type: " << ui_commandfeedback_data.tcpinfo.tcptype;
}

void RobotView::on_SetTcpInfo_clicked()
{
  ui_tcp.tcptype = TcpType::TCP_ROD_DRILL;
  m_robotApi->settcpinfo(ui_tcp);
}

void RobotView::on_setCubesidelength_valueChanged(double arg1)
{
  cubeSidelength = arg1 * 10.0;
  qDebug() << "Set Cubesidelength " << arg1 << "cm";
}

void RobotView::on_setInitialPoint_clicked()
{
  //m_robotApi->setspeed(70);
  //m_robotApi->requestrealtimedata();
  //ui_moveInitial_realtime_data = m_robotApi->realtime_data;
  //initial_point_ready = true;
  //cube_point_number = 0;
  //qDebug() << "Set InitialPoint finished!";
  m_robotApi->setworkmode(2);
  m_robotApi->movep(1, 0, 0, 0, 0, 0);
  qDebug() << "on_setInitialPoint_clicked";
}

void RobotView::on_nextPoint_clicked()
{
  if (!initial_point_ready)
    qDebug() << "SetInitialPoint First!!!";
  else
  {
    switch (cube_point_number)
    {
      case 0: //PC1
        qDebug() << ++cube_point_number;
        m_robotApi->movej(m_robotApi->realtime_data.joints.j1, 60.0, 30.0, 0.0, 90.0, 0.0);
        //            staubli->movej(ui_moveInitial_realtime_data.joints.j1+qrand()%10-5,
        //                           ui_moveInitial_realtime_data.joints.j2+qrand()%10-5,
        //                           ui_moveInitial_realtime_data.joints.j3+qrand()%10-5,
        //                           ui_moveInitial_realtime_data.joints.j4+qrand()%10-5,
        //                           ui_moveInitial_realtime_data.joints.j5+qrand()%10-5,
        //                           ui_moveInitial_realtime_data.joints.j6+qrand()%10-5);
        break;
      case 1: //PC2
        qDebug() << ++cube_point_number;
        //staubli->movej(true, ui_moveInitial_realtime_data.pose.x+qrand()%100-50,
        m_robotApi->movel(ui_moveInitial_realtime_data.pose.x + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.y + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.z + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.a,
                          ui_moveInitial_realtime_data.pose.b,
                          ui_moveInitial_realtime_data.pose.c);
        break;
      case 2: //PC3
        qDebug() << ++cube_point_number;
        //staubli->movej(true, ui_moveInitial_realtime_data.pose.x+cubeSidelength,
        m_robotApi->movel(ui_moveInitial_realtime_data.pose.x + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.y + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.z + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.a,
                          ui_moveInitial_realtime_data.pose.b,
                          ui_moveInitial_realtime_data.pose.c);
        break;
      case 3: //PC4
        qDebug() << ++cube_point_number;
        //staubli->movej(true, ui_moveInitial_realtime_data.pose.x,
        m_robotApi->movel(ui_moveInitial_realtime_data.pose.x + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.y + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.z + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.a,
                          ui_moveInitial_realtime_data.pose.b,
                          ui_moveInitial_realtime_data.pose.c);
        break;
      case 4: //PC5
        qDebug() << ++cube_point_number;
        m_robotApi->movep(ui_moveInitial_realtime_data.pose.x + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.y + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.z + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.a + qrand() % 10 - 5,
                          ui_moveInitial_realtime_data.pose.b + qrand() % 10 - 5,
                          ui_moveInitial_realtime_data.pose.c + qrand() % 10 - 5);
        break;
      case 5:
        qDebug() << ++cube_point_number;
        m_robotApi->movep(ui_moveInitial_realtime_data.pose.x + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.y + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.z + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.a + qrand() % 10 - 5,
                          ui_moveInitial_realtime_data.pose.b + qrand() % 10 - 5,
                          ui_moveInitial_realtime_data.pose.c + qrand() % 10 - 5);
        break;
      case 6:
        qDebug() << ++cube_point_number;
        m_robotApi->movep(ui_moveInitial_realtime_data.pose.x + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.y + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.z + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.a + qrand() % 10 - 5,
                          ui_moveInitial_realtime_data.pose.b + qrand() % 10 - 5,
                          ui_moveInitial_realtime_data.pose.c + qrand() % 10 - 5);
        break;
      case 7:
        qDebug() << ++cube_point_number;
        m_robotApi->movep(ui_moveInitial_realtime_data.pose.x + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.y + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.z + qrand() % 50 - 25,
                          ui_moveInitial_realtime_data.pose.a + qrand() % 10 - 5,
                          ui_moveInitial_realtime_data.pose.b + qrand() % 10 - 5,
                          ui_moveInitial_realtime_data.pose.c + qrand() % 10 - 5);
        cube_point_number = 0;
        qDebug() << "reset cube_point_number: " << cube_point_number;
        break;
      default:
        qDebug() << "wrong cube_point_number: " << cube_point_number << ", and reset cube_point_number: 0";
        cube_point_number = 0;
        break;
    }
  }
}

void RobotView::on_GetLimitData_clicked()
{
  qDebug() << "Get Traction Limit Data";
  m_robotApi->getlimitdata();
}

void RobotView::on_SetLimitData_clicked()
{
  m_robotApi->requestrealtimedata();
  RobotRealtimeData l_realtime_data = m_robotApi->realtime_data;
  TractionLimit l;
  l.init();
  l.angle = 30;
  l.p1 = l_realtime_data.pose;
  l.p2 = l_realtime_data.pose;
  l.p1.z -= 50;
  l.p2.z += 50;
  m_robotApi->setlimitdata(l);
}

void RobotView::on_Foldrobot_clicked()
{
  m_robotApi->movej(120.0, 0.0, 135.0, 0.0, 45.0, 0.0);
}

void RobotView::on_Unfolrobot_clicked()
{
  m_robotApi->movej(-45.0, 0.0, 135.0, 0.0, 45.0, 0.0);
}

void RobotView::on_positionInit_left_clicked()
{
  qDebug() << "on_positionInit_left_clicked";
  m_robotApi->setworkmode(2);
  m_robotApi->movep(2, 0, 0, 0, 0, 0);
}

void RobotView::on_positionInit_right_clicked()
{
  qDebug() << "on_positionInit_right_clicked";
  m_robotApi->setworkmode(2);
  m_robotApi->movep(3, 0, 0, 0, 0, 0);
}

void RobotView::on_kuka_mode_1_clicked()
{
  m_robotApi->setworkmode(1);
  MITK_INFO << "on_kuka_mode_1_clicked";
}

void RobotView::on_kuka_mode_3_clicked()
{
  m_robotApi->setworkmode(3);
  MITK_INFO << "on_kuka_mode_3_clicked";
}

void RobotView::on_SetSpeed_clicked()
{
}

void RobotView::on_setIO_clicked()
{
  if (!isIO)
  {
    m_robotApi->setio(1, 1);
    isIO = true;
    this->m_Controls.setIO->setText("设置骨钻(关)");
    //todo
    //IGTController::issetio = false;
  }
  else
  {
    m_robotApi->setio(1, 0);
    isIO = false;
    this->m_Controls.setIO->setText("设置骨钻(开)");
    //todo
    //IGTController::issetio = true;
  }
}

unsigned RobotView::HeartBeatThread(void *pInfoStruct)
{
  MITK_INFO << "threadStartTracking Called";
  /* extract this pointer from Thread Info structure */
  struct itk::MultiThreader::ThreadInfoStruct *pInfo = (struct itk::MultiThreader::ThreadInfoStruct *)pInfoStruct;

  if (pInfo == nullptr)
  {
    return ITK_THREAD_RETURN_VALUE;
  }
  if (pInfo->UserData == nullptr)
  {
    return ITK_THREAD_RETURN_VALUE;
  }
  RobotView *robot_view = (RobotView *)pInfo->UserData;
  if (robot_view != nullptr)
  {
    while (robot_view->m_runTread)
    {
      if (robot_view->m_heartBeat)
      {
          robot_view->m_robotApi->setspeed(90);
          Sleep(2000);
          MITK_INFO << "beat!";
      } 
    }
  }
  robot_view->m_ThreadID = 0; // erase thread id, now that this thread will end.
  return ITK_THREAD_RETURN_VALUE;
}
