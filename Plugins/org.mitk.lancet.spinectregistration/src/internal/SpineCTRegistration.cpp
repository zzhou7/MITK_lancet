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
#include "SpineCTRegistration.h"

// Qt
#include "leastsquaresfit.h"
#include <QMessageBox>
#include <QPushButton>

#include "mitkImageToSurfaceFilter.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateOr.h"
#include "mitkNodePredicateProperty.h"
#include "mitkPointSet.h"
#include "mitkSurface.h"
#include "surfaceregistraion.h"
#include "vtkConnectivityFilter.h"
#include <QPushButton>
#include <mitkImage.h>
#include <vtkSphereSource.h>

const std::string SpineCTRegistration::VIEW_ID = "org.mitk.views.spinectregistration";

//---------------------------Below: QT slots------------------------------//

// Extract steelball centers as a pointset
void SpineCTRegistration::GetSteelballCenters()
{
  auto inputCtImage = dynamic_cast<mitk::Image *>(m_CtImageDataNode->GetData());

  // The isosurface of all steelballs as into a single polydata
  double threshold = m_Controls.lineEdit_SteelballThreshold->text().toDouble();
  auto mitkSteelBallSurfaces = mitk::Surface::New();
  mitk::ImageToSurfaceFilter::Pointer imageToSurfaceFilter = mitk::ImageToSurfaceFilter::New();

  imageToSurfaceFilter->SetInput(inputCtImage);
  imageToSurfaceFilter->SetThreshold(threshold);
  mitkSteelBallSurfaces = imageToSurfaceFilter->GetOutput();

  // Separate steelball surface by examining their connectivity
  vtkNew<vtkConnectivityFilter> vtkConnectivityFilter;
  vtkConnectivityFilter->SetInputData(mitkSteelBallSurfaces->GetVtkPolyData());

  vtkConnectivityFilter->SetExtractionModeToAllRegions();
  vtkConnectivityFilter->Update();
  int numberOfTotalSteelBalls = vtkConnectivityFilter->GetNumberOfExtractedRegions();

  auto mitkSingleSteelballCenterPointset = mitk::PointSet::New(); // store each steelball's center
  double centerOfAllSteelballs[3]{0, 0, 0};                       // the center of all steel balls

  vtkConnectivityFilter->SetExtractionModeToSpecifiedRegions();
  for (int m = 0; m < numberOfTotalSteelBalls; m++)
  {
    vtkConnectivityFilter->InitializeSpecifiedRegionList();
    vtkConnectivityFilter->AddSpecifiedRegion(m);
    vtkConnectivityFilter->Update();

    auto vtkSingleSteelBallSurface = vtkConnectivityFilter->GetPolyDataOutput();

    auto numberOfCells =
      vtkSingleSteelBallSurface->GetNumberOfCells(); // the total number of cells of a single mesh surface; each cell
                                                     // stores one facet of the mesh surface

    std::vector<double> inp_x(
      numberOfCells); // inp_x, inp_y and inp_z store one point of each facet on the mesh surface
    std::vector<double> inp_y(
      numberOfCells); // inp_x, inp_y and inp_z store one point of each facet on the mesh surface
    std::vector<double> inp_z(
      numberOfCells); // inp_x, inp_y and inp_z store one point of each facet on the mesh surface

    for (int n = 0; n < numberOfCells; n++)
    {
      auto tmpPoint = vtkSingleSteelBallSurface->GetCell(n)->GetPoints()->GetPoint(0);

      inp_x[n] = tmpPoint[0];
      inp_y[n] = tmpPoint[1];
      inp_z[n] = tmpPoint[2];
    }

    // use inp_x, inp_y and inp_z to simulate a sphere
    double cx, cy, cz;
    double R;

    lancetAlgorithm::fit_sphere(inp_x, inp_y, inp_z, cx, cy, cz, R);

    mitk::Point3D mitkTmpCenterPoint3D;
    mitkTmpCenterPoint3D[0] = cx;
    mitkTmpCenterPoint3D[1] = cy;
    mitkTmpCenterPoint3D[2] = cz;
    mitkSingleSteelballCenterPointset->InsertPoint(mitkTmpCenterPoint3D);

    centerOfAllSteelballs[0] = centerOfAllSteelballs[0] + cx;
    centerOfAllSteelballs[1] = centerOfAllSteelballs[1] + cy;
    centerOfAllSteelballs[2] = centerOfAllSteelballs[2] + cz;

    // // Draw simulated spheres
    // auto vtkBallSource0 = vtkSmartPointer<vtkSphereSource>::New();
    // vtkBallSource0->SetCenter(cx, cy, cz);
    // vtkBallSource0->SetRadius(R);
    // vtkBallSource0->Update();
    //
    // auto tmpNode = mitk::DataNode::New();
    //
    // tmpNode->SetName("Single steelball sphere");
    // auto mitkSteelBallSurfacesNew1 = mitk::Surface::New();
    // mitkSteelBallSurfacesNew1->SetVtkPolyData(vtkBallSource0->GetOutput());
    // tmpNode->SetData(mitkSteelBallSurfacesNew1);
    // GetDataStorage()->Add(tmpNode);
  }

  centerOfAllSteelballs[0] = centerOfAllSteelballs[0] / numberOfTotalSteelBalls;
  centerOfAllSteelballs[1] = centerOfAllSteelballs[1] / numberOfTotalSteelBalls;
  centerOfAllSteelballs[2] = centerOfAllSteelballs[2] / numberOfTotalSteelBalls;

  // Sort the centers of the separate steelballs according to their distances to the group center
  std::vector<double> distancesToPointSetCenter(numberOfTotalSteelBalls);
  std::vector<int> distanceRanks(numberOfTotalSteelBalls);

  for (int i = 0; i < numberOfTotalSteelBalls; i++)
  {
    distancesToPointSetCenter[i] =
      sqrt(pow(centerOfAllSteelballs[0] - mitkSingleSteelballCenterPointset->GetPoint(i)[0], 2) +
           pow(centerOfAllSteelballs[1] - mitkSingleSteelballCenterPointset->GetPoint(i)[1], 2) +
           pow(centerOfAllSteelballs[2] - mitkSingleSteelballCenterPointset->GetPoint(i)[2], 2));

    distanceRanks[i] = i;
  }

  for (int i = 0; i < numberOfTotalSteelBalls; i++)
  {
    MITK_INFO << "Distance before sorting: " << distancesToPointSetCenter[i];
  }

  for (int i = 0; i < numberOfTotalSteelBalls - 2; i++)
  {
    for (int j = 0; j < numberOfTotalSteelBalls - 1 - i; j++)
    {
      double temp = 0;
      double temp2 = 0;
      if (distancesToPointSetCenter[j] > distancesToPointSetCenter[j + 1])
      {
        temp = distancesToPointSetCenter[j];
        distancesToPointSetCenter[j] = distancesToPointSetCenter[j + 1];
        distancesToPointSetCenter[j + 1] = temp;

        temp2 = distanceRanks[j];
        distanceRanks[j] = distanceRanks[j + 1];
        distanceRanks[j + 1] = temp2;
      }
    }
  }

  for (int i = 0; i < numberOfTotalSteelBalls; i++)
  {
    MITK_INFO << "Distance after sorting: " << distancesToPointSetCenter[i];
  }

  auto mitkSortedSingleSteelballCenterPointset = mitk::PointSet::New();
  for (int i = 0; i < numberOfTotalSteelBalls; i++)
  {
    mitkSortedSingleSteelballCenterPointset->InsertPoint(mitkSingleSteelballCenterPointset->GetPoint(distanceRanks[i]));
  }

  // draw extracted  steel ball surfaces
  auto nodeSteelballSurfaces = mitk::DataNode::New();
  nodeSteelballSurfaces->SetName("Steelball surfaces");
  // add new node
  nodeSteelballSurfaces->SetData(mitkSteelBallSurfaces);
  GetDataStorage()->Add(nodeSteelballSurfaces);

  // add steel ball centers
  auto nodeSteelballCenters = mitk::DataNode::New();
  nodeSteelballCenters->SetName("Steelball centers");
  // add new node
  nodeSteelballCenters->SetData(mitkSingleSteelballCenterPointset);
  GetDataStorage()->Add(nodeSteelballCenters);

  // add sorted steel ball centers
  auto nodeSortedSteelballCenters = mitk::DataNode::New();
  nodeSortedSteelballCenters->SetName("Sorted Steelball centers");
  // add new node
  nodeSortedSteelballCenters->SetData(mitkSortedSingleSteelballCenterPointset);
  GetDataStorage()->Add(nodeSortedSteelballCenters);
}

// Reset the origin of an mitk::Image to (0, 0, 0), realign the image's axes to the standard xyz axes
void SpineCTRegistration::ResetImage()
{
  // Image origin to (0, 0, 0)
  auto inputImage = dynamic_cast<mitk::Image *>(m_CtImageDataNode->GetData());

  mitk::Point3D imageOrigin;
  imageOrigin[0] = 0.0;
  imageOrigin[1] = 0.0;
  imageOrigin[2] = 0.0;

  inputImage->SetOrigin(imageOrigin);

  // Align the image's axes to the standard xyz axes
  auto tmpVtkTransform = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> tmpVtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
  tmpVtkTransform->Identity();
  tmpVtkTransform->GetMatrix(tmpVtkMatrix);

  inputImage->GetGeometry(0)->SetIndexToWorldTransformByVtkMatrixWithoutChangingSpacing(
    tmpVtkMatrix); // SetIndexToWorldTransformByVtkMatrix(tmpVtkMatrix) will set the spacing as (1, 1, 1)
}

// Landmark registration (pointset to pointset)
void SpineCTRegistration::LandmarkRegistration()
{
  auto landmarkRegistrator = mitk::SurfaceRegistration::New();

  MITK_INFO << "Proceedinng Landmark registration";
  if (m_LandmarkSrcPointsetDataNode != nullptr && m_LandmarkTargetPointsetDataNode != nullptr)
  {
    auto sourcePointset = dynamic_cast<mitk::PointSet *>(m_LandmarkSrcPointsetDataNode->GetData());
    auto targetPointset = dynamic_cast<mitk::PointSet *>(m_LandmarkTargetPointsetDataNode->GetData());
    landmarkRegistrator->SetLandmarksSrc(sourcePointset);
    landmarkRegistrator->SetLandmarksTarget(targetPointset);
    landmarkRegistrator->ComputeLandMarkResult();

    Eigen::Matrix4d tmpRegistrationResult{landmarkRegistrator->GetResult()->GetData()};
    tmpRegistrationResult.transposeInPlace();
    m_TmpRegistrationResult = tmpRegistrationResult;
    MITK_INFO << m_TmpRegistrationResult;
    UpdateRegistrationMatrixInUI();
  }

  
  std::ostringstream os;
  landmarkRegistrator->GetResult()->Print(os);
  
  m_Controls.textBrowser_Registration->append("-------------Start landmark registration----------");
  m_Controls.textBrowser_Registration->append(QString::fromStdString(os.str()));
  
};

// ICP registration (surface to pointset)
void SpineCTRegistration::IcpRegistration(){
  auto icpRegistrator = mitk::SurfaceRegistration::New();

  MITK_INFO << "Proceedinng ICP registration";

  if (m_IcpSrcSurfaceDataNode != nullptr && m_IcpTargetPointsetDataNode != nullptr)
  {
    auto icpTargetPointset = dynamic_cast<mitk::PointSet *>(m_IcpTargetPointsetDataNode->GetData());
    auto icpSrcSurface = dynamic_cast<mitk::Surface *>(m_IcpSrcSurfaceDataNode->GetData());
    icpRegistrator->SetIcpPoints(icpTargetPointset);
    icpRegistrator->SetSurfaceSrc(icpSrcSurface);
    icpRegistrator->ComputeIcpResult();

    Eigen::Matrix4d tmpRegistrationResult{icpRegistrator->GetResult()->GetData()};
    tmpRegistrationResult.transposeInPlace();

    m_TmpRegistrationResult = tmpRegistrationResult;
    MITK_INFO << m_TmpRegistrationResult;

    UpdateRegistrationMatrixInUI();
  }

  std::ostringstream os;
  icpRegistrator->GetResult()->Print(os);
  m_Controls.textBrowser_Registration->append("-------------Start ICP registration----------");
  m_Controls.textBrowser_Registration->append(QString::fromStdString(os.str()));

};

// Update the registration result in the UI
void SpineCTRegistration::UpdateRegistrationMatrixInUI()
{
  m_Controls.lineEdit_RegistrationMatrix_0->setText(QString::number(m_TmpRegistrationResult(0)));
  m_Controls.lineEdit_RegistrationMatrix_1->setText(QString::number(m_TmpRegistrationResult(1)));
  m_Controls.lineEdit_RegistrationMatrix_2->setText(QString::number(m_TmpRegistrationResult(2)));
  m_Controls.lineEdit_RegistrationMatrix_3->setText(QString::number(m_TmpRegistrationResult(3)));
  m_Controls.lineEdit_RegistrationMatrix_4->setText(QString::number(m_TmpRegistrationResult(4)));
  m_Controls.lineEdit_RegistrationMatrix_5->setText(QString::number(m_TmpRegistrationResult(5)));
  m_Controls.lineEdit_RegistrationMatrix_6->setText(QString::number(m_TmpRegistrationResult(6)));
  m_Controls.lineEdit_RegistrationMatrix_7->setText(QString::number(m_TmpRegistrationResult(7)));
  m_Controls.lineEdit_RegistrationMatrix_8->setText(QString::number(m_TmpRegistrationResult(8)));
  m_Controls.lineEdit_RegistrationMatrix_9->setText(QString::number(m_TmpRegistrationResult(9)));
  m_Controls.lineEdit_RegistrationMatrix_10->setText(QString::number(m_TmpRegistrationResult(10)));
  m_Controls.lineEdit_RegistrationMatrix_11->setText(QString::number(m_TmpRegistrationResult(11)));
  m_Controls.lineEdit_RegistrationMatrix_12->setText(QString::number(m_TmpRegistrationResult(12)));
  m_Controls.lineEdit_RegistrationMatrix_13->setText(QString::number(m_TmpRegistrationResult(13)));
  m_Controls.lineEdit_RegistrationMatrix_14->setText(QString::number(m_TmpRegistrationResult(14)));
  m_Controls.lineEdit_RegistrationMatrix_15->setText(QString::number(m_TmpRegistrationResult(15)));

};

//---------------------------Above: QT slots------------------------------//
//
//
//
//---------------------------Below: QT UI initialization------------------------------//

void SpineCTRegistration::SetFocus()
{
  // m_Controls.buttonPerformImageProcessing->setFocus();
}
void SpineCTRegistration::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);

  // Initialize mitkNodeSelectionWidget
  InitNodeSelector(m_Controls.mitkNodeSelectWidget_CTimage);
  InitNodeSelector(m_Controls.mitkNodeSelectWidget_MovingSurface);
  InitNodeSelector(m_Controls.mitkNodeSelectWidget_LandmarkSrcPointset);
  InitNodeSelector(m_Controls.mitkNodeSelectWidget_LandmarkTargetPointset);
  InitNodeSelector(m_Controls.mitkNodeSelectWidget_IcpSrcSurface);
  InitNodeSelector(m_Controls.mitkNodeSelectWidget_IcpTargetPointset);

  // Connect the signals and slots
  connect(m_Controls.mitkNodeSelectWidget_CTimage,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &SpineCTRegistration::ChangeCtImage);

  connect(m_Controls.mitkNodeSelectWidget_MovingSurface,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &SpineCTRegistration::ChangeMovingSurface);

  connect(m_Controls.mitkNodeSelectWidget_LandmarkSrcPointset,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &SpineCTRegistration::ChangeLandmarkSrcPointset);

  connect(m_Controls.mitkNodeSelectWidget_LandmarkTargetPointset,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &SpineCTRegistration::ChangeLandmarkTargetPointset);

  connect(m_Controls.mitkNodeSelectWidget_IcpSrcSurface,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &SpineCTRegistration::ChangeIcpSrcSurface);

  connect(m_Controls.mitkNodeSelectWidget_IcpTargetPointset,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &SpineCTRegistration::ChangeIcpTargetPointset);

  connect(m_Controls.pushButton_ResetImageOrigin, &QPushButton::clicked, this, &SpineCTRegistration::ResetImage);

  connect(m_Controls.pushButton_Landmark, &QPushButton::clicked, this, &SpineCTRegistration::LandmarkRegistration);

  connect(m_Controls.pushButton_Icp, &QPushButton::clicked, this, &SpineCTRegistration::IcpRegistration);
}

void SpineCTRegistration::InitNodeSelector(QmitkSingleNodeSelectionWidget *widget)
{
  widget->SetDataStorage(GetDataStorage());
  widget->SetNodePredicate(mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(
    mitk::NodePredicateProperty::New("helper object"), mitk::NodePredicateProperty::New("hidden object"))));
  widget->SetSelectionIsOptional(true);
  widget->SetAutoSelectNewNodes(true);
  widget->SetEmptyInfo(QString("Please select a node"));
  widget->SetPopUpTitel(QString("Select node"));
}

inline void SpineCTRegistration::ChangeCtImage(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_CtImageDataNode = m_Controls.mitkNodeSelectWidget_CTimage->GetSelectedNode();
}

inline void SpineCTRegistration::ChangeMovingSurface(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_MovingSurfaceDataNode = m_Controls.mitkNodeSelectWidget_MovingSurface->GetSelectedNode();
}

inline void SpineCTRegistration::ChangeLandmarkSrcPointset(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_LandmarkSrcPointsetDataNode = m_Controls.mitkNodeSelectWidget_LandmarkSrcPointset->GetSelectedNode();
}

inline void SpineCTRegistration::ChangeLandmarkTargetPointset(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_LandmarkTargetPointsetDataNode = m_Controls.mitkNodeSelectWidget_LandmarkTargetPointset->GetSelectedNode();
}

inline void SpineCTRegistration::ChangeIcpSrcSurface(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_IcpSrcSurfaceDataNode = m_Controls.mitkNodeSelectWidget_IcpSrcSurface->GetSelectedNode();
}

inline void SpineCTRegistration::ChangeIcpTargetPointset(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_IcpTargetPointsetDataNode = m_Controls.mitkNodeSelectWidget_IcpTargetPointset->GetSelectedNode();
}

//---------------------------Above: QT UI initialization------------------------------//
