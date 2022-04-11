/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include "itkCastImageFilter.h"
#include "itkCommand.h"
#include "itkNormalizedCorrelationTwoImageToOneImageMetric.h"
#include "itkPowellOptimizer.h"
#include "itkTwoProjectionImageRegistrationMethod.h"

#include "itkEuler3DTransform.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRayCastInterpolateImageFunction.h"
// --------
#include "itkFlipImageFilter.h"
#include "itkImage.h"
#include "itkImageDuplicator.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkSiddonJacobsRayCastInterpolateImageFunction.h"
#include "itkTimeProbesCollectorBase.h"
#include "setuptcpcalibrator.h"


//------------
#include "itkResampleImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "mitkITKImageImport.h"
#include "mitkImageAccessByItk.h"
#include "mitkImageCast.h"
#include <itkShiftScaleImageFilter.h>
#include <mitkImageToItk.h>
#include <ITKOptimizer.h>
#include <eigen3/Eigen/Eigen>


SetUpTcpCalibrator::SetUpTcpCalibrator() = default;
SetUpTcpCalibrator::~SetUpTcpCalibrator() = default;

// void SetUpTcpCalibrator::calibrate(double toolPointA[3],
//                                     double toolPointB[3],
//                                     double toolPointC[3],
//                                     double sawPointD[3],
//                                     double sawPlanePointP[3],
//                                     double sawPlanePointQ[3],
//                                     double sawPlanePointS[3])
// {
//   Eigen::Vector3d AC(toolPointC[0] - toolPointA[0], toolPointC[1] - toolPointA[1], toolPointC[2] - toolPointA[2]);
//   double normAC = AC.norm();
//
//   Eigen::Vector3d AB(toolPointB[0] - toolPointA[0], toolPointB[1] - toolPointA[1], toolPointB[2] - toolPointA[2]);
//   double normAB = AB.norm();
//
//   Eigen::Vector3d AD(sawPointD[0] - toolPointA[0], sawPointD[1] - toolPointA[1], sawPointD[2] - toolPointA[2]);
//   double normAD = AD.norm();
//
//   Eigen::Vector3d PQ(sawPlanePointQ[0] - sawPlanePointP[0],
//                      sawPlanePointQ[1] - sawPlanePointP[1],
//                      sawPlanePointQ[2] - sawPlanePointP[2]);
//   double normPQ = PQ.norm();
//
//   Eigen::Vector3d PS(sawPlanePointS[0] - sawPlanePointP[0],
//                      sawPlanePointS[1] - sawPlanePointP[1],
//                      sawPlanePointS[2] - sawPlanePointP[2]);
//   double normPS = PS.norm();
//
//   // 3 unit vectors of the coordinate system at Point A
//   Eigen::Vector3d y;
//   y = AC / normAC;
//
//   Eigen::Vector3d z;
//   z = (AB.cross(AC)) / (normAB * normAC);
//
//   Eigen::Vector3d x;
//   x = y.cross(z);
//
//   Eigen::Matrix3d matrixA;
//   matrixA.col(0) = x;
//   matrixA.col(1) = y;
//   matrixA.col(2) = z;
//   Eigen::Matrix3d inverseMatrixA = matrixA.inverse();
//
//   // 3 unit vectors of the coordinate system at Point D
//   Eigen::Vector3d X;
//   X = PQ.cross(PS) / (normPQ * normPS);
//   if (X.dot(x) < 0)
//   {
//     X = -X;
//   }
//
//   Eigen::Vector3d Y;
//   Y = y - X * (y.dot(X));
//   Y = Y / Y.norm();
//
//   Eigen::Vector3d Z;
//   Z = X.cross(Y);
//
//   Eigen::Matrix3d matrixD;
//   matrixD.col(0) = X;
//   matrixD.col(1) = Y;
//   matrixD.col(2) = Z;
//
//   // Obtain the rotation angles
//   Eigen::Matrix3d matrixR;
//   matrixR = matrixD * inverseMatrixA;
//   Eigen::Vector3d eulerAngles = matrixR.eulerAngles(2, 1, 0);
//
//   double r_x = eulerAngles[2];
//   double r_y = eulerAngles[1];
//   double r_z = eulerAngles[0];
//
//   // Obtain the translation (D's position under A's coordinate system)
//   double x_d = AD.dot(x);
//   double y_d = AD.dot(y);
//   double z_d = AD.dot(z);
//
//   // print out
//   std::cout << "r_z: " << r_z << std::endl;
//   std::cout << "r_y: " << r_y << std::endl;
//   std::cout << "r_x: " << r_x << std::endl;
//
//   std::cout << "x: " << x_d << std::endl;
//   std::cout << "y: " << y_d << std::endl;
//   std::cout << "z: " << z_d << std::endl;
// }

void SetUpTcpCalibrator::calibrateGooseSaw(double MatrixRefToPointAcoordinate[16],
                 double sawPointD[3],
                 double sawPlanePointP[3],
                 double sawPlanePointQ[3],
                 double sawPlanePointS[3])
{
  Eigen::Vector3d PQ(sawPlanePointQ[0] - sawPlanePointP[0],
                     sawPlanePointQ[1] - sawPlanePointP[1],
                     sawPlanePointQ[2] - sawPlanePointP[2]);
  double normPQ = PQ.norm();

  Eigen::Vector3d PS(sawPlanePointS[0] - sawPlanePointP[0],
                     sawPlanePointS[1] - sawPlanePointP[1],
                     sawPlanePointS[2] - sawPlanePointP[2]);
  double normPS = PS.norm();


  // 3 unit vectors of the coordinate system at Point A
  Eigen::Matrix4d matrixRefToPointACoordinate{MatrixRefToPointAcoordinate};
  matrixRefToPointACoordinate.transposeInPlace();
  Eigen::Vector3d x(matrixRefToPointACoordinate(0), matrixRefToPointACoordinate(1), matrixRefToPointACoordinate(2));
  Eigen::Vector3d y(matrixRefToPointACoordinate(4), matrixRefToPointACoordinate(5), matrixRefToPointACoordinate(6));
  Eigen::Vector3d z(matrixRefToPointACoordinate(8), matrixRefToPointACoordinate(9), matrixRefToPointACoordinate(10));

  Eigen::Matrix3d matrixA;
  matrixA.col(0) = x;
  matrixA.col(1) = y;
  matrixA.col(2) = z;
  Eigen::Matrix3d inverseMatrixA = matrixA.inverse();

  // 3 unit vectors of the coordinate system at Point D
  Eigen::Vector3d X;
  X = PQ.cross(PS) / (normPQ * normPS);
  if (X.dot(x) < 0)
  {
    X = -X;
  }

  Eigen::Vector3d Y;
  Y = y - X * (y.dot(X));
  Y = Y / Y.norm();

  Eigen::Vector3d Z;
  Z = X.cross(Y);

  Eigen::Matrix3d matrixD;
  matrixD.col(0) = X;
  matrixD.col(1) = Y;
  matrixD.col(2) = Z;

  // Obtain the rotation angles
  Eigen::Matrix3d matrixR;
  matrixR = matrixD * inverseMatrixA;
  Eigen::Vector3d eulerAngles = matrixR.eulerAngles(0, 1, 2); // ZYX rotation

  double r_x = eulerAngles[0];
  double r_y = eulerAngles[1];
  double r_z = eulerAngles[2];

  // Obtain the translation (D's position under A's coordinate system)
  Eigen::Vector3d AD
   (sawPointD[0]-matrixRefToPointACoordinate(12),
	  sawPointD[1]-matrixRefToPointACoordinate(13),
	  sawPointD[2]-matrixRefToPointACoordinate(14));

  double x_d = AD.dot(x);
  double y_d = AD.dot(y);
  double z_d = AD.dot(z);


  // retrieve the member variables
  m_Rx = r_x;
  m_Ry = r_y;
  m_Rz = r_z;

  m_Tx = x_d;
  m_Ty = y_d;
  m_Tz = z_d;
}