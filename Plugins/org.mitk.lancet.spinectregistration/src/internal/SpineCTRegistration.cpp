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
#include "mitkApplyTransformMatrixOperation.h"
#include "mitkInteractionConst.h"
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

void SpineCTRegistration::TransformSurface()
{
  auto movingObject = dynamic_cast<mitk::Surface *>(m_MovingSurfaceDataNode->GetData());

  double registrationResult[16];

  for (int m=0; m < 16; m++)
  {
    registrationResult[m] = *(m_TmpRegistrationResult.data() + m);
  }

  vtkSmartPointer<vtkMatrix4x4> tmpVtkMatrixRegistrationResult = vtkSmartPointer<vtkMatrix4x4>::New();
  tmpVtkMatrixRegistrationResult->DeepCopy(registrationResult);
  tmpVtkMatrixRegistrationResult->Transpose();

  vtkTransform *trans = vtkTransform::New();
  trans->PostMultiply();
  trans->SetMatrix(movingObject->GetGeometry()->GetVtkMatrix());
  trans->Concatenate(tmpVtkMatrixRegistrationResult);
  trans->Update();

  mitk::Point3D ref;
  auto *mitkMatrixOperation =
    new mitk::ApplyTransformMatrixOperation(mitk::OpAPPLYTRANSFORMMATRIX, trans->GetMatrix(), ref);

  movingObject->GetGeometry()->ExecuteOperation(mitkMatrixOperation);

  delete mitkMatrixOperation;

  mitk::RenderingManager::GetInstance()->RequestUpdateAll();

}

void SpineCTRegistration::TransformImage()
{
  auto movingObject = dynamic_cast<mitk::Image *>(m_MovingSurfaceDataNode->GetData());

  double registrationResult[16];

  for (int m = 0; m < 16; m++)
  {
    registrationResult[m] = *(m_TmpRegistrationResult.data() + m);
  }

  vtkSmartPointer<vtkMatrix4x4> tmpVtkMatrixRegistrationResult = vtkSmartPointer<vtkMatrix4x4>::New();
  tmpVtkMatrixRegistrationResult->DeepCopy(registrationResult);
  tmpVtkMatrixRegistrationResult->Transpose();

  vtkTransform *trans = vtkTransform::New();
  trans->PostMultiply();
  trans->SetMatrix(movingObject->GetGeometry()->GetVtkMatrix());
  trans->Concatenate(tmpVtkMatrixRegistrationResult);
  trans->Update();

  movingObject->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(trans->GetMatrix());

  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}









