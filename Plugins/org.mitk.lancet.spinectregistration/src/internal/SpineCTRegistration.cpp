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
    tmpVtkMatrix);
    // SetIndexToWorldTransformByVtkMatrix(tmpVtkMatrix) will set the spacing as (1, 1, 1),
    //because the spacing is determined by the matrix diagonal
}

void SpineCTRegistration::ReconstructSpineSurface()
{
  auto inputCtImage = dynamic_cast<mitk::Image *>(m_CtImageDataNode->GetData());

  // The isosurface of all steelballs as into a single polydata
  double threshold = m_Controls.lineEdit_BoneSurfaceThreshold->text().toDouble();
  auto mitkSteelBallSurfaces = mitk::Surface::New();
  mitk::ImageToSurfaceFilter::Pointer imageToSurfaceFilter = mitk::ImageToSurfaceFilter::New();

  imageToSurfaceFilter->SetInput(inputCtImage);
  imageToSurfaceFilter->SetThreshold(threshold);
  mitkSteelBallSurfaces = imageToSurfaceFilter->GetOutput();

  // // Separate steelball surface by examining their connectivity
  // vtkNew<vtkConnectivityFilter> vtkConnectivityFilter;
  // vtkConnectivityFilter->SetInputData(mitkSteelBallSurfaces->GetVtkPolyData());
  //
  // vtkConnectivityFilter->SetExtractionModeToAllRegions();
  // vtkConnectivityFilter->Update();
  // int numberOfTotalSteelBalls = vtkConnectivityFilter->GetNumberOfExtractedRegions();
  //
  // auto mitkSingleSteelballCenterPointset = mitk::PointSet::New(); // store each steelball's center
  // double centerOfAllSteelballs[3]{0, 0, 0};                       // the center of all steel balls
  //
  // vtkConnectivityFilter->SetExtractionModeToSpecifiedRegions();
  // for (int m = 0; m < numberOfTotalSteelBalls; m++)
  // {
  //   vtkConnectivityFilter->InitializeSpecifiedRegionList();
  //   vtkConnectivityFilter->AddSpecifiedRegion(m);
  //   vtkConnectivityFilter->Update();
  //
  //   auto vtkSingleSteelBallSurface = vtkConnectivityFilter->GetPolyDataOutput();
  //
  //   auto numberOfCells =
  //     vtkSingleSteelBallSurface->GetNumberOfCells(); // the total number of cells of a single mesh surface; each cell
  //                                                    // stores one facet of the mesh surface
  //
  //   std::vector<double> inp_x(
  //     numberOfCells); // inp_x, inp_y and inp_z store one point of each facet on the mesh surface
  //   std::vector<double> inp_y(
  //     numberOfCells); // inp_x, inp_y and inp_z store one point of each facet on the mesh surface
  //   std::vector<double> inp_z(
  //     numberOfCells); // inp_x, inp_y and inp_z store one point of each facet on the mesh surface
  //
  //   for (int n = 0; n < numberOfCells; n++)
  //   {
  //     auto tmpPoint = vtkSingleSteelBallSurface->GetCell(n)->GetPoints()->GetPoint(0);
  //
  //     inp_x[n] = tmpPoint[0];
  //     inp_y[n] = tmpPoint[1];
  //     inp_z[n] = tmpPoint[2];
  //   }
  //
  //   // use inp_x, inp_y and inp_z to simulate a sphere
  //   double cx, cy, cz;
  //   double R;
  //
  //   lancetAlgorithm::fit_sphere(inp_x, inp_y, inp_z, cx, cy, cz, R);
  //
  //   mitk::Point3D mitkTmpCenterPoint3D;
  //   mitkTmpCenterPoint3D[0] = cx;
  //   mitkTmpCenterPoint3D[1] = cy;
  //   mitkTmpCenterPoint3D[2] = cz;
  //   mitkSingleSteelballCenterPointset->InsertPoint(mitkTmpCenterPoint3D);
  //
  //   centerOfAllSteelballs[0] = centerOfAllSteelballs[0] + cx;
  //   centerOfAllSteelballs[1] = centerOfAllSteelballs[1] + cy;
  //   centerOfAllSteelballs[2] = centerOfAllSteelballs[2] + cz;
  //
  //   // // Draw simulated spheres
  //   // auto vtkBallSource0 = vtkSmartPointer<vtkSphereSource>::New();
  //   // vtkBallSource0->SetCenter(cx, cy, cz);
  //   // vtkBallSource0->SetRadius(R);
  //   // vtkBallSource0->Update();
  //   //
  //   // auto tmpNode = mitk::DataNode::New();
  //   //
  //   // tmpNode->SetName("Single steelball sphere");
  //   // auto mitkSteelBallSurfacesNew1 = mitk::Surface::New();
  //   // mitkSteelBallSurfacesNew1->SetVtkPolyData(vtkBallSource0->GetOutput());
  //   // tmpNode->SetData(mitkSteelBallSurfacesNew1);
  //   // GetDataStorage()->Add(tmpNode);
  // }
  //
  // centerOfAllSteelballs[0] = centerOfAllSteelballs[0] / numberOfTotalSteelBalls;
  // centerOfAllSteelballs[1] = centerOfAllSteelballs[1] / numberOfTotalSteelBalls;
  // centerOfAllSteelballs[2] = centerOfAllSteelballs[2] / numberOfTotalSteelBalls;
  //
  // // Sort the centers of the separate steelballs according to their distances to the group center
  // std::vector<double> distancesToPointSetCenter(numberOfTotalSteelBalls);
  // std::vector<int> distanceRanks(numberOfTotalSteelBalls);
  //
  // for (int i = 0; i < numberOfTotalSteelBalls; i++)
  // {
  //   distancesToPointSetCenter[i] =
  //     sqrt(pow(centerOfAllSteelballs[0] - mitkSingleSteelballCenterPointset->GetPoint(i)[0], 2) +
  //          pow(centerOfAllSteelballs[1] - mitkSingleSteelballCenterPointset->GetPoint(i)[1], 2) +
  //          pow(centerOfAllSteelballs[2] - mitkSingleSteelballCenterPointset->GetPoint(i)[2], 2));
  //
  //   distanceRanks[i] = i;
  // }
  //
  // for (int i = 0; i < numberOfTotalSteelBalls; i++)
  // {
  //   MITK_INFO << "Distance before sorting: " << distancesToPointSetCenter[i];
  // }
  //
  // for (int i = 0; i < numberOfTotalSteelBalls - 2; i++)
  // {
  //   for (int j = 0; j < numberOfTotalSteelBalls - 1 - i; j++)
  //   {
  //     double temp = 0;
  //     double temp2 = 0;
  //     if (distancesToPointSetCenter[j] > distancesToPointSetCenter[j + 1])
  //     {
  //       temp = distancesToPointSetCenter[j];
  //       distancesToPointSetCenter[j] = distancesToPointSetCenter[j + 1];
  //       distancesToPointSetCenter[j + 1] = temp;
  //
  //       temp2 = distanceRanks[j];
  //       distanceRanks[j] = distanceRanks[j + 1];
  //       distanceRanks[j + 1] = temp2;
  //     }
  //   }
  // }
  //
  // for (int i = 0; i < numberOfTotalSteelBalls; i++)
  // {
  //   MITK_INFO << "Distance after sorting: " << distancesToPointSetCenter[i];
  // }
  //
  // auto mitkSortedSingleSteelballCenterPointset = mitk::PointSet::New();
  // for (int i = 0; i < numberOfTotalSteelBalls; i++)
  // {
  //   mitkSortedSingleSteelballCenterPointset->InsertPoint(mitkSingleSteelballCenterPointset->GetPoint(distanceRanks[i]));
  // }

  // draw extracted  steel ball surfaces
  auto nodeSteelballSurfaces = mitk::DataNode::New();
  nodeSteelballSurfaces->SetName("Bone surface");
  // add new node
  nodeSteelballSurfaces->SetData(mitkSteelBallSurfaces);
  GetDataStorage()->Add(nodeSteelballSurfaces);

}










