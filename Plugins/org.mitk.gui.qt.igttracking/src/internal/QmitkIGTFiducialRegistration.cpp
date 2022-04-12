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

//Mitk
#include <mitkDataNode.h>
#include <mitkNodePredicateNot.h>
#include <mitkNodePredicateProperty.h>
#include <mitkNodePredicateOr.h>
#include <mitkNodePredicateDataType.h>
#include <mitkIRenderingManager.h>
#include <mitkImageGenerator.h>

// Qmitk
#include "QmitkIGTFiducialRegistration.h"
#include <QmitkRenderWindow.h>

// Qt
#include <QMessageBox>
#include <QSettings>
#include <QTimer>

// MicroServices
#include <usModuleContext.h>
#include <usGetModuleContext.h>
#include "usServiceReference.h"

#include <vtkQuaternion.h>
#include <eigen3/Eigen/Eigen>
const std::string QmitkIGTFiducialRegistration::VIEW_ID = "org.mitk.views.IGTFiducialRegistration";

void QmitkIGTFiducialRegistration::SetFocus()
{
}

void QmitkIGTFiducialRegistration::CreateQtPartControl( QWidget *parent )
{
  m_ToolNDPointer = mitk::NavigationData::New();
  m_ProbeNDPointer = mitk::NavigationData::New();
  m_ReferenceNDPointer = mitk::NavigationData::New();
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi( parent );

  //Connect signals and slots
  connect(m_Controls.m_ChooseSelectedPointer, SIGNAL(clicked()), this, SLOT(PointerSelectionChanged()));
  connect(m_Controls.m_ChooseSelectedImage, SIGNAL(clicked()), this, SLOT(ImageSelectionChanged()));

  connect(m_Controls.m_ChooseSelectedPointer_probe, SIGNAL(clicked()), this, SLOT(ProbeSelect()));
  connect(m_Controls.m_ChooseSelectedPointer_tool, SIGNAL(clicked()), this, SLOT(ToolSelect()));
  connect(m_Controls.m_ChooseSelectedPointer_reference, SIGNAL(clicked()), this, SLOT(ReferenceSelect()));

  connect(m_Controls.pushButton_snapshotTool, SIGNAL(clicked()), this, SLOT(SnapShotTool()));
  connect(m_Controls.pushButton_probeSawPit, SIGNAL(clicked()), this, SLOT(ProbeSawPit()));
  connect(m_Controls.pushButton_probeSawPlane, SIGNAL(clicked()), this, SLOT(ProbeSawPlane()));
  //connect(m_Controls.pushButton_snapshotTool, SIGNAL(clicked()), this, SLOT(SnapShotTool()));
  connect(m_Controls.pushButton_calibarate, SIGNAL(clicked()), this, SLOT(Calibrate()));
  //Initialize Combobox
  m_Controls.m_DataStorageComboBox->SetDataStorage(this->GetDataStorage());
  m_Controls.m_DataStorageComboBox->SetAutoSelectNewItems(false);
  m_Controls.m_DataStorageComboBox->SetPredicate(mitk::NodePredicateOr::New(mitk::NodePredicateDataType::New("Surface"), mitk::NodePredicateDataType::New("Image")));

  //Initialize Fiducial Registration Widget
  m_Controls.m_FiducialRegistrationWidget->setDataStorage(this->GetDataStorage());
  m_Controls.m_FiducialRegistrationWidget->HideStaticRegistrationRadioButton(true);
  m_Controls.m_FiducialRegistrationWidget->HideContinousRegistrationRadioButton(true);
  m_Controls.m_FiducialRegistrationWidget->HideUseICPRegistrationCheckbox(true);

}

vtkMatrix4x4 * QmitkIGTFiducialRegistration::getVtkMatrix4x4(mitk::NavigationData::Pointer nd)
{
  auto o = nd->GetOrientation();
  double R[3][3];
  double* V = {nd->GetPosition().GetDataPointer()};
  vtkQuaterniond quaterniond{o.r(), o.x(), o.y(), o.z()};
  quaterniond.ToMatrix3x3(R);

  vtkMatrix4x4* matrix = vtkMatrix4x4::New();
  matrix->SetElement(0, 0, R[0][0]);
  matrix->SetElement(0, 1, R[0][1]);
  matrix->SetElement(0, 2, R[0][2]);
  matrix->SetElement(1, 0, R[1][0]);
  matrix->SetElement(1, 1, R[1][1]);
  matrix->SetElement(1, 2, R[1][2]);
  matrix->SetElement(2, 0, R[2][0]);
  matrix->SetElement(2, 1, R[2][1]);
  matrix->SetElement(2, 2, R[2][2]);

  matrix->SetElement(0, 3, V[0]);
  matrix->SetElement(1, 3, V[1]);
  matrix->SetElement(2, 3, V[2]);

  matrix->Print(std::cout);
  return matrix;
}

void QmitkIGTFiducialRegistration::InitializeRegistration()
{
  foreach(QmitkRenderWindow* renderWindow, this->GetRenderWindowPart(mitk::WorkbenchUtil::OPEN)->GetQmitkRenderWindows().values())
  {
    this->m_Controls.m_FiducialRegistrationWidget->AddSliceNavigationController(renderWindow->GetSliceNavigationController());
  }
}

void QmitkIGTFiducialRegistration::PointerSelectionChanged()
{
  InitializeRegistration();
  int toolID = m_Controls.m_TrackingDeviceSelectionWidget->GetSelectedToolID();
  m_TrackingPointer = m_Controls.m_TrackingDeviceSelectionWidget->GetSelectedNavigationDataSource()->GetOutput(toolID);
  m_Controls.m_FiducialRegistrationWidget->setTrackerNavigationData(m_TrackingPointer);
  m_Controls.m_PointerLabel->setText(m_Controls.m_TrackingDeviceSelectionWidget->GetSelectedNavigationTool()->GetToolName().c_str());
}

void QmitkIGTFiducialRegistration::ImageSelectionChanged()
{
  InitializeRegistration();
  m_Controls.m_ImageLabel->setText(m_Controls.m_DataStorageComboBox->GetSelectedNode()->GetName().c_str());
  m_Controls.m_FiducialRegistrationWidget->setImageNode(m_Controls.m_DataStorageComboBox->GetSelectedNode());
}

void QmitkIGTFiducialRegistration::ProbeSelect()
{
  int toolID = m_Controls.m_TrackingDeviceSelectionWidget_sawcalib->GetSelectedToolID();
  m_ProbeNDPointer = m_Controls.m_TrackingDeviceSelectionWidget_sawcalib->GetSelectedNavigationDataSource()->GetOutput(toolID);
  //m_Controls.m_FiducialRegistrationWidget->setTrackerNavigationData(m_TrackingPointer);
  m_Controls.m_PointerLabel_probe->setText(
    m_Controls.m_TrackingDeviceSelectionWidget_sawcalib->GetSelectedNavigationTool()->GetToolName().c_str());
}

void QmitkIGTFiducialRegistration::ReferenceSelect()
{
  int toolID = m_Controls.m_TrackingDeviceSelectionWidget_sawcalib->GetSelectedToolID();
  m_ReferenceNDPointer =
    m_Controls.m_TrackingDeviceSelectionWidget_sawcalib->GetSelectedNavigationDataSource()->GetOutput(toolID);
  // m_Controls.m_FiducialRegistrationWidget->setTrackerNavigationData(m_TrackingPointer);
  m_Controls.m_PointerLabel_reference->setText(
    m_Controls.m_TrackingDeviceSelectionWidget_sawcalib->GetSelectedNavigationTool()->GetToolName().c_str());
}

void QmitkIGTFiducialRegistration::ToolSelect()
{
  int toolID = m_Controls.m_TrackingDeviceSelectionWidget_sawcalib->GetSelectedToolID();
  m_ToolNDPointer =
    m_Controls.m_TrackingDeviceSelectionWidget_sawcalib->GetSelectedNavigationDataSource()->GetOutput(toolID);
  // m_Controls.m_FiducialRegistrationWidget->setTrackerNavigationData(m_TrackingPointer);
  m_Controls.m_PointerLabel_tool->setText(
    m_Controls.m_TrackingDeviceSelectionWidget_sawcalib->GetSelectedNavigationTool()->GetToolName().c_str());
}

void QmitkIGTFiducialRegistration::SnapShotTool()
{
  if (m_ToolNDPointer->IsDataValid() && m_ReferenceNDPointer->IsDataValid())
  {
    auto toolMatrix = getVtkMatrix4x4(m_ToolNDPointer);
    auto ReferenceMatrix = getVtkMatrix4x4(m_ReferenceNDPointer);

    m_toolMatrixInRef = vtkMatrix4x4::New();
    getReferenceMatrix4x4(toolMatrix, ReferenceMatrix, m_toolMatrixInRef);

    m_toolMatrixInRef->Print(std::cout);
  }
  else
  {
    m_Controls.textBrowser_sawCalibRes->setText("tool is not valid");
  }
  Eigen::Vector3d x{
    m_toolMatrixInRef->GetElement(0, 0), m_toolMatrixInRef->GetElement(1, 0), m_toolMatrixInRef->GetElement(2, 0)};

  Eigen::Vector3d y{
    m_toolMatrixInRef->GetElement(0, 1), m_toolMatrixInRef->GetElement(1, 1), m_toolMatrixInRef->GetElement(2, 1)};

  Eigen::Vector3d z{
    m_toolMatrixInRef->GetElement(0, 2), m_toolMatrixInRef->GetElement(1, 2), m_toolMatrixInRef->GetElement(2, 2)};

  Eigen::Vector3d o{
    m_toolMatrixInRef->GetElement(0, 3), m_toolMatrixInRef->GetElement(1, 3), m_toolMatrixInRef->GetElement(2, 3)};

  mitk::Point3D Origin{o.data()};

  Eigen::Vector3d vx = o + x * 50;
  Eigen::Vector3d vy = o + y * 50;
  Eigen::Vector3d vz = o + z * 50;

  mitk::Point3D A {vx.data()};
  mitk::Point3D B{vy.data()};
  mitk::Point3D C{vz.data()};

  //show node
  auto toolnode = GetDataStorage()->GetNamedNode("toolInRef");
  if (toolnode == nullptr)
  {
    mitk::DataNode::Pointer toolnode = mitk::DataNode::New();

    auto pSet = mitk::PointSet::New();
    pSet->InsertPoint(Origin);
    pSet->InsertPoint(A);
    pSet->InsertPoint(B);
    pSet->InsertPoint(C);
    toolnode->SetData(pSet);
    toolnode->SetName("toolInRef");
    toolnode->SetFloatProperty("pointsize", 5);
    GetDataStorage()->Add(toolnode);
  }
  else
  {
    auto pSet = mitk::PointSet::New();
    pSet->InsertPoint(Origin);
    pSet->InsertPoint(A);
    pSet->InsertPoint(B);
    pSet->InsertPoint(C);
    toolnode->SetData(pSet);
  }
}

void QmitkIGTFiducialRegistration::ProbeSawPit()
{
  if (m_ProbeNDPointer->IsDataValid() && m_ReferenceNDPointer->IsDataValid())
  {
    auto probeMatrix = getVtkMatrix4x4(m_ProbeNDPointer);
    auto ReferenceMatrix = getVtkMatrix4x4(m_ReferenceNDPointer);

    vtkMatrix4x4 *probeInRef = vtkMatrix4x4::New();
    getReferenceMatrix4x4(probeMatrix, ReferenceMatrix, probeInRef);

    double p[3]{probeInRef->GetElement(0, 3), probeInRef->GetElement(1, 3), probeInRef->GetElement(2, 3)};
    mitk::Point3D point{p};
    // for showing the polish result
    auto SawPitnode = GetDataStorage()->GetNamedNode("SawPit");

    if (SawPitnode == nullptr)
    {
      mitk::DataNode::Pointer SawPitnode = mitk::DataNode::New();

      m_ProbePitPointInRef = mitk::PointSet::New();

      
      m_ProbePitPointInRef->InsertPoint(point);
      SawPitnode->SetData(m_ProbePitPointInRef);
      SawPitnode->SetName("SawPit");
      SawPitnode->SetFloatProperty("pointsize", 3);
      SawPitnode->SetColor(0,1,0);
      GetDataStorage()->Add(SawPitnode);
    }
    else
    {
      dynamic_cast<mitk::PointSet *>(SawPitnode->GetData())->InsertPoint(point);
    }
  }
  else
  {
    m_Controls.textBrowser_sawCalibRes->setText("tool is not valid");
  }
}

void QmitkIGTFiducialRegistration::ProbeSawPlane()
{
  if (m_ProbeNDPointer->IsDataValid() && m_ReferenceNDPointer->IsDataValid())
  {
    auto probeMatrix = getVtkMatrix4x4(m_ProbeNDPointer);
    auto ReferenceMatrix = getVtkMatrix4x4(m_ReferenceNDPointer);

    vtkMatrix4x4 *probeInRef = vtkMatrix4x4::New();
    getReferenceMatrix4x4(probeMatrix, ReferenceMatrix, probeInRef);

    double p[3]{probeInRef->GetElement(0, 3), probeInRef->GetElement(1, 3), probeInRef->GetElement(2, 3)};
    mitk::Point3D point{p};

    auto SawPlanenode = GetDataStorage()->GetNamedNode("SawPlane");
    if (SawPlanenode == nullptr)
    {
      mitk::DataNode::Pointer SawPlanenode = mitk::DataNode::New();

      m_probePlanePointInRef = mitk::PointSet::New();

      m_probePlanePointInRef->InsertPoint(point);
      SawPlanenode->SetData(m_probePlanePointInRef);
      SawPlanenode->SetName("SawPlane");
      SawPlanenode->SetFloatProperty("pointsize", 2);
      GetDataStorage()->Add(SawPlanenode);
    }
    else
    {
      dynamic_cast<mitk::PointSet *>(SawPlanenode->GetData())->InsertPoint(point);
    }
  }
  else
  {
    m_Controls.textBrowser_sawCalibRes->setText("tool is not valid");
  }
}

void QmitkIGTFiducialRegistration::Calibrate()
{
  MITK_INFO << m_ProbePitPointInRef;
  MITK_INFO << m_probePlanePointInRef;
  //MITK_INFO << m_toolMatrixInRef;
  auto pd = m_ProbePitPointInRef->GetPoint(0).GetDataPointer();
  auto pp = m_probePlanePointInRef->GetPoint(0).GetDataPointer();
  auto pq = m_probePlanePointInRef->GetPoint(1).GetDataPointer();
  auto ps = m_probePlanePointInRef->GetPoint(2).GetDataPointer();
  calibrateGooseSaw(m_toolMatrixInRef->GetData(), pd, pp, pq, ps);
}

void QmitkIGTFiducialRegistration::calibrateGooseSaw(double MatrixRefToPointAcoordinate[16],
                                                     double sawPointD[3],
                                                     double sawPlanePointP[3],
                                                     double sawPlanePointQ[3],
                                                     double sawPlanePointS[3])
{
  Eigen::Vector3d PQ(sawPlanePointQ[0] - sawPlanePointP[0],
                     sawPlanePointQ[1] - sawPlanePointP[1],
                     sawPlanePointQ[2] - sawPlanePointP[2]);
  //double normPQ = PQ.norm();
  PQ.normalize();
  Eigen::Vector3d PS(sawPlanePointS[0] - sawPlanePointP[0],
                     sawPlanePointS[1] - sawPlanePointP[1],
                     sawPlanePointS[2] - sawPlanePointP[2]);
  PS.normalize();
  //double normPS = PS.norm();

  // 3 unit vectors of the coordinate system at Point A
  Eigen::Matrix4d matrixRefToPointACoordinate{MatrixRefToPointAcoordinate};
  matrixRefToPointACoordinate.transposeInPlace();
  MITK_INFO << matrixRefToPointACoordinate;
  Eigen::Vector3d x(matrixRefToPointACoordinate(0), matrixRefToPointACoordinate(1), matrixRefToPointACoordinate(2));
  Eigen::Vector3d y(matrixRefToPointACoordinate(4), matrixRefToPointACoordinate(5), matrixRefToPointACoordinate(6));
  Eigen::Vector3d z(matrixRefToPointACoordinate(8), matrixRefToPointACoordinate(9), matrixRefToPointACoordinate(10));

  MITK_INFO <<"x before: " << x;

  Eigen::Matrix3d matrixA;
  matrixA.col(0) = x;
  matrixA.col(1) = y;
  matrixA.col(2) = z;
  Eigen::Matrix3d inverseMatrixA = matrixA.inverse();

  MITK_INFO << "x after: " << x;

  // 3 unit vectors of the coordinate system at Point D
  Eigen::Vector3d X;
  X = PQ.cross(PS);
  if (X.dot(x) < 0)
  {
    X = -X;
  }

  Eigen::Vector3d Y;
  Y = y - X * (y.dot(X));
  Y.normalize();
  //double normY = Y.norm();
  //Y = Y / normY;

  Eigen::Vector3d Z;
  Z = X.cross(Y);

  double simpleZangle = acos(abs(x.dot(X)));

  Eigen::Matrix3d matrixD;
  matrixD.col(0) = X;
  matrixD.col(1) = Y;
  matrixD.col(2) = Z;

  // Obtain the rotation angles
  Eigen::Matrix3d matrixR;
  matrixR = inverseMatrixA * matrixD;

  // double rx, ry, rz, r_x, r_y, r_z;
  // double piParameter = 180 / 3.1415926;
  //
  // if (matrixR(0, 2) < 1)
  // {
  //   if (matrixR(0, 2) > -1)
  //   {
  //     ry = asin(matrixR(0, 2));
  //     rx = atan2(-matrixR(1, 2), matrixR(2, 2));
  //     rz = atan2(-matrixR(0, 1), matrixR(0, 0));
  //   }
  //   else
  //   {
  //     ry = -3.1415926 / 2;
  //     rx = -atan2(matrixR(1, 0), matrixR(1, 1));
  //     rz = 0;
  //   }
  // }
  // else
  // {
  //   ry = 3.1415926 / 2;
  //   rx = atan2(matrixR(1, 0), matrixR(1, 1));
  //   rz = 0;
  // }
  // r_x = rx * piParameter;
  // r_y = ry * piParameter;
  // r_z = rz * piParameter;
  Eigen::Vector3d eulerAngles = matrixR.eulerAngles(0, 1, 2); // ZYX rotation
  MITK_INFO << matrixR;
  double r_x = eulerAngles[0];
  double r_y = eulerAngles[1];
  double r_z = eulerAngles[2];
  

  // Obtain the translation (D's position under A's coordinate system)
  Eigen::Vector3d AD(sawPointD[0] - matrixRefToPointACoordinate(12),
                     sawPointD[1] - matrixRefToPointACoordinate(13),
                     sawPointD[2] - matrixRefToPointACoordinate(14));

  double x_d = AD.dot(x);
  double y_d = AD.dot(y);
  double z_d = AD.dot(z);

  // retrieve the member variables
  //m_Controls.textBrowser_sawCalibRes->append("")
  MITK_INFO << "RX:" << r_x;
  MITK_INFO << "RY:" << r_y;
  MITK_INFO << "RZ:" << r_z;

  MITK_INFO << "SimpleZ:" << simpleZangle;
  MITK_INFO << "X:" << x_d;
  MITK_INFO << "Y:" << y_d;
  MITK_INFO << "Z:" << z_d;
}

QmitkIGTFiducialRegistration::QmitkIGTFiducialRegistration()
{

}

QmitkIGTFiducialRegistration::~QmitkIGTFiducialRegistration()
{

}
