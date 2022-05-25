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

  inputImage->GetGeometry(0)->SetIndexToWorldTransformByVtkMatrixWithoutChangingSpacing(tmpVtkMatrix);
  // SetIndexToWorldTransformByVtkMatrix(tmpVtkMatrix) will set the spacing as (1, 1, 1),
  // because the spacing is determined by the matrix diagonal
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

  // draw extracted  steel ball surfaces
  auto nodeSteelballSurfaces = mitk::DataNode::New();
  nodeSteelballSurfaces->SetName("Bone surface");
  // add new node
  nodeSteelballSurfaces->SetData(mitkSteelBallSurfaces);
  GetDataStorage()->Add(nodeSteelballSurfaces);
}

void SpineCTRegistration::CheckToolValidity() // regard it as the metric
{
  m_Controls.textBrowser_checkNDI->append("~~~~~~ Checking tool validity ~~~~~~");
  m_Controls.textBrowser_checkNDI->append(" ");

  // Checkpoint 0: The distance between any 2 markers on the same tool should be longer than 50 mm
  m_Controls.textBrowser_checkNDI->append("--------- Entered Check point 0 -----------");

  metric[0] = 1; // a metric to measure whether the new tool meets all 3 design requirements ( {1, 1, 1} )
  for (int m = 0; m < 4; m++)
  {
    for (int n = 0; n < 4; n++)
    {
      if (m < n)
      {
        // The distance between marker m and marker n
        double marker_m[2]{inputTool[m], inputTool[m + 4]};
        double marker_n[2]{inputTool[n], inputTool[n + 4]};

        double internalSegDistance = sqrt(pow(marker_m[0] - marker_n[0], 2) + pow(marker_m[1] - marker_n[1], 2));

        if (internalSegDistance < 50)
        {
          m_Controls.textBrowser_checkNDI->append("!!Warning!! On the new tool, the distance between Point_" +
                                                  QString::number(m) + " and Point_" + QString::number(n) + " is " +
                                                  QString::number(internalSegDistance) + " < 50");

          metric[0] = 0;
        }
      }
    }
  }

  if (metric[0] == 1) // The distance between any 2 markers on the same tool is longer than 50 mm
  {
    m_Controls.textBrowser_checkNDI->append("--------- Passed check point 0 (50 mm) ----------");
  }
  else
  {
    m_Controls.textBrowser_checkNDI->append("--------- Failed check point 0 (50 mm) ---------");
    return;
  }

  // Checkpoint 2: The length difference of any 2 segments on the same tool should be more than 3.5 mm
  metric[1] = 1;
  m_Controls.textBrowser_checkNDI->append("--------- Entered Check point 1 -----------");

  for (int m = 0; m < 4; m++)
  {
    for (int n = 0; n < 4; n++)
    {
      if (m < n)
      {
        // The distance between marker m and marker n
        double marker_m[2]{inputTool[m], inputTool[m + 4]};
        double marker_n[2]{inputTool[n], inputTool[n + 4]};

        double internalSegDistance_0 = sqrt(pow(marker_m[0] - marker_n[0], 2) + pow(marker_m[1] - marker_n[1], 2));

        for (int i = 0; i < 4; i++)
        {
          for (int j = 0; j < 4; j++)
          {
            if (((i < j) && ((m != i) || (n != j))) && ((m <= i) && (n <= j))) // avoid redundant repetition
            {
              // The distance between marker i and marker j
              double marker_i[2]{inputTool[i], inputTool[i + 4]};
              double marker_j[2]{inputTool[j], inputTool[j + 4]};

              double internalSegDistance_1 =
                sqrt(pow(marker_i[0] - marker_j[0], 2) + pow(marker_i[1] - marker_j[1], 2));

              if (abs(internalSegDistance_1 - internalSegDistance_0) < 3.5)
              {
                metric[1] = 0;

                m_Controls.textBrowser_checkNDI->append(
                  "!!Warning!! Found internal segment length difference " +
                  QString::number(abs(internalSegDistance_1 - internalSegDistance_0)) + " < 3.5 mm");

                m_Controls.textBrowser_checkNDI->append("New tool segment_0: Point_" + QString::number(m) +
                                                        " and Point_" + QString::number(n) + " ( " +
                                                        QString::number(internalSegDistance_0) + " )");
                m_Controls.textBrowser_checkNDI->append("New tool segment_1: Point_" + QString::number(i) +
                                                        " and Point_" + QString::number(j) + " ( " +
                                                        QString::number(internalSegDistance_1) + " )");
              }
            }
          }
        }
      }
    }
  }

  if (metric[1] == 1) // The length difference of any 2 segments on the same tool is more than 3.5 mm
  {
    m_Controls.textBrowser_checkNDI->append("--------- Passed check point 1 (3.5 mm) ----------");
  }
  else
  {
    m_Controls.textBrowser_checkNDI->append("--------- Failed check point 1 (3.5 mm) ---------");
    return;
  }

  // Checkpoint 3: The length difference of any 2 segments on the same tool should be more than 3.5 mm
  metric[2] = 1;
  m_Controls.textBrowser_checkNDI->append("--------- Entered Check point 2 -----------");

  for (int m = 0; m < 4; m++)
  {
    for (int n = 0; n < 4; n++)
    {
      if (m < n)
      {
        // The distance between marker m and marker n
        double marker_m[2]{inputTool[m], inputTool[m + 4]};
        double marker_n[2]{inputTool[n], inputTool[n + 4]};

        double internalSegDistance_0 = sqrt(pow(marker_m[0] - marker_n[0], 2) + pow(marker_m[1] - marker_n[1], 2));

        for (int i = 0; i < 4; i++)
        {
          for (int j = 0; j < 4; j++)
          {
            if (((i < j) && ((m != i) || (n != j))) && ((m <= i) && (n <= j))) // avoid redundant repetition
            {
              // The distance between marker m and marker n
              double marker_i[2]{inputTool[i], inputTool[i + 4]};
              double marker_j[2]{inputTool[j], inputTool[j + 4]};

              double internalSegDistance_1 =
                sqrt(pow(marker_i[0] - marker_j[0], 2) + pow(marker_i[1] - marker_j[1], 2));

              // Till now, 1 segment pair on the new tool has been obtained
              // Pair_0 = segment_0 + segment_1
              // segment_0 = marker_m + marker_n
              // segment_1 = marker_i + marker_j

              // Next we want to get another segment pair on 1 existing tool
              for (int existingToolCount = 0; existingToolCount < existingToolNum; existingToolCount++)
              {
                for (int q = 0; q < 4; q++)
                {
                  for (int s = 0; s < 4; s++)
                  {
                    if (q < s)
                    {
                      // The distance between marker q and marker s
                      // double marker_q[2]{inputTool[q], inputTool[q + 4]};
                      // double marker_s[2]{inputTool[s], inputTool[s + 4]};
                      double marker_q[2]{existingTools[existingToolCount * 4 + q],
                                         existingTools[existingToolNum * 4 + existingToolCount * 4 + q]};
                      double marker_s[2]{existingTools[existingToolCount * 4 + s],
                                         existingTools[existingToolNum * 4 + existingToolCount * 4 + s]};

                      double internalSegDistance_2 =
                        sqrt(pow(marker_q[0] - marker_s[0], 2) + pow(marker_q[1] - marker_s[1], 2));

                      for (int d = 0; d < 4; d++)
                      {
                        for (int t = 0; t < 4; t++)
                        {
                          if (((d < t) && ((q != d) || (s != t))) &&
                              ((q <= d) && (s <= t))) // avoid redundant repetition
                          {
                            // The distance between marker d and marker t
                            double marker_d[2]{existingTools[existingToolCount * 4 + d],
                                               existingTools[existingToolNum * 4 + existingToolCount * 4 + d]};
                            double marker_t[2]{existingTools[existingToolCount * 4 + t],
                                               existingTools[existingToolNum * 4 + existingToolCount * 4 + t]};

                            double internalSegDistance_3 =
                              sqrt(pow(marker_d[0] - marker_t[0], 2) + pow(marker_d[1] - marker_t[1], 2));

                            // 1 segment pair on 1 existing tool has been obtained
                            // Pair_1 = segment_2 + segment_3
                            // segment_2 = marker_q + marker_s
                            // segment_3 = marker_d + marker_t
                            if (((abs(internalSegDistance_0 - internalSegDistance_2) < 3.5) &&
                                 (abs(internalSegDistance_1 - internalSegDistance_3) < 3.5)) ||
                                ((abs(internalSegDistance_0 - internalSegDistance_3) < 3.5) &&
                                 (abs(internalSegDistance_1 - internalSegDistance_2) <
                                  3.5))) // check if (segment_0 & segment_1) and (segment_2 & segment_3) are like
                                         // segment pairs
                            {
                              // angle between (segment_0 & segment_1)
                              Eigen::Vector2d vectorSegment_0(marker_m[0] - marker_n[0], marker_m[1] - marker_n[1]);
                              Eigen::Vector2d vectorSegment_1(marker_i[0] - marker_j[0], marker_i[1] - marker_j[1]);
                              vectorSegment_0.normalize();
                              vectorSegment_1.normalize();

                              double angleSeg_0_1 = 180 * acos(abs(vectorSegment_0.dot(vectorSegment_1))) / 3.1415926;

                              // angle between (segment_0 & segment_1)
                              Eigen::Vector2d vectorSegment_2(marker_q[0] - marker_s[0], marker_q[1] - marker_s[1]);
                              Eigen::Vector2d vectorSegment_3(marker_d[0] - marker_t[0], marker_d[1] - marker_t[1]);
                              vectorSegment_2.normalize();
                              vectorSegment_3.normalize();

                              double angleSeg_2_3 = 180 * acos(abs(vectorSegment_2.dot(vectorSegment_3))) / 3.1415926;

                              if (abs(angleSeg_0_1 - angleSeg_2_3) < 2)
                              {
                                m_Controls.textBrowser_checkNDI->append(
                                  "!!Warning!! Found like segment pairs having angle difference in degree: " +
                                  QString::number(abs(angleSeg_0_1 - angleSeg_2_3)) + "( < 2)");

                                m_Controls.textBrowser_checkNDI->append("Pair_0 on the new tool:");
                                m_Controls.textBrowser_checkNDI->append("Pair_0, segment_0: Point_" +
                                                                        QString::number(m) + " and Point_" +
                                                                        QString::number(n));
                                m_Controls.textBrowser_checkNDI->append("Pair_0, segment_1: Point_" +
                                                                        QString::number(i) + " and Point_" +
                                                                        QString::number(j));
                                m_Controls.textBrowser_checkNDI->append("Pair_0 angle: " +
                                                                        QString::number(angleSeg_0_1));


                                m_Controls.textBrowser_checkNDI->append("Pair_1 on Tool_" +
                                                                        QString::number(existingToolCount));
                                m_Controls.textBrowser_checkNDI->append("Pair_1, segment_2: Point_" +
                                                                        QString::number(q) + " and Point_" +
                                                                        QString::number(s));
                                m_Controls.textBrowser_checkNDI->append("Pair_1, segment_3: Point_" +
                                                                        QString::number(d) + " and Point_" +
                                                                        QString::number(t));
                                m_Controls.textBrowser_checkNDI->append("Pair_1 angle: " +
                                                                        QString::number(angleSeg_2_3));

                                metric[2] = 0;
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  if (metric[2] == 1) // The length difference of any 2 segments on the same tool is more than 3.5 mm
  {
    m_Controls.textBrowser_checkNDI->append("--------- Passed check point 2 (2 degrees)-----------");
  }
  else
  {
    m_Controls.textBrowser_checkNDI->append("--------- Failed check point 2 (2 degrees) ----------");
    return;
  }


}


void SpineCTRegistration::UpdateTool()
{
  double toolCenter_x = (inputTool[0] + inputTool[1] + inputTool[2] + inputTool[3]) / 4;
  double toolCenter_y = (inputTool[4] + inputTool[5] + inputTool[6] + inputTool[7]) / 4;

  // failed the distance requirement so we span the tool geometry
  if (metric[0] == 0 || metric[1] ==0)
  {
    double grow_factor{0.07};
    inputTool[0] = (inputTool[0] - toolCenter_x) * grow_factor + inputTool[0];
    inputTool[1] = (inputTool[1] - toolCenter_x) * grow_factor + inputTool[1];
    inputTool[2] = (inputTool[2] - toolCenter_x) * grow_factor + inputTool[2];
    inputTool[3] = (inputTool[3] - toolCenter_x) * grow_factor + inputTool[3];

    inputTool[0] = (inputTool[0] - toolCenter_x) * grow_factor + inputTool[0];
    inputTool[1] = (inputTool[1] - toolCenter_x) * grow_factor + inputTool[1];
    inputTool[2] = (inputTool[2] - toolCenter_x) * grow_factor + inputTool[2];
    inputTool[3] = (inputTool[3] - toolCenter_x) * grow_factor + inputTool[3];

    inputTool[4] = (inputTool[4] - toolCenter_y) * grow_factor + inputTool[4];
    inputTool[5] = (inputTool[5] - toolCenter_y) * grow_factor + inputTool[5];
    inputTool[6] = (inputTool[6] - toolCenter_y) * grow_factor + inputTool[6];
    inputTool[7] = (inputTool[7] - toolCenter_y) * grow_factor + inputTool[7];

    return;
  }

  
  if (metric[2] == 0)
  {
    double modulate_factor{0.05};

    for (int n = 0; n < 4; n++)
    {
      srand((int)time(NULL));
      if (rand() % 100 < 50)
      {
        inputTool[n] = (inputTool[n] - toolCenter_x) * modulate_factor + inputTool[n];
        inputTool[n + 4] = (inputTool[n + 4] - toolCenter_x) * modulate_factor + inputTool[n + 4];
      }else
      {
        modulate_factor = -modulate_factor;
        inputTool[n] = (inputTool[n] - toolCenter_x) * modulate_factor + inputTool[n];
        inputTool[n + 4] = (inputTool[n + 4] - toolCenter_x) * modulate_factor + inputTool[n + 4];
      }
      
    }
  }
}

void SpineCTRegistration::OptimizeTool()
{
  
}

