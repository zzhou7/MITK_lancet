/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

/*=============================================================
Below are Headers for the Node Editor plugin
==============================================================*/
#include <sstream>

#include <cmath>
// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "NodeEditor.h"

// Qt
#include <QDoubleSpinBox>
#include <QPushButton>

// mitk image
#include "basic.h"
#include "mitkImagePixelReadAccessor.h"
#include "mitkImagePixelWriteAccessor.h"
#include "mitkNodePredicateAnd.h"
#include "mitkNodePredicateDataType.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateOr.h"
#include "mitkNodePredicateProperty.h"
#include "mitkSurfaceOperation.h"
#include "physioModelFactory.h"
#include "vtkPlaneSource.h"
#include "vtkSphereSource.h"
#include <QComboBox>
#include <itkBSplineInterpolateImageFunction.h>
#include <itkResampleImageFilter.h>
#include <mitkApplyTransformMatrixOperation.h>
#include <mitkBoundingShapeCropper.h>
#include <mitkImage.h>
#include <mitkInteractionConst.h>
#include <mitkPadImageFilter.h>
#include <mitkPointOperation.h>
#include <mitkPointSet.h>
#include <mitkPointSetShapeProperty.h>
#include <mitkRotationOperation.h>
#include <mitkSurface.h>
#include <mitkSurfaceToImageFilter.h>
#include <mitkVector.h>
#include <vtkClipPolyData.h>
#include <vtkImageStencil.h>
#include <vtkLandmarkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkPlane.h>
#include <vtkSTLReader.h>
#include <vtkPolyLine.h>
#include "drrGenerator.h"
#include <drr.h>
#include <drrsidonjacobsraytracing.h>
#include <eigen3/Eigen/Eigen>
#include <mitkPadImageFilter.h>
#include <nodebinder.h>
#include <surfaceregistraion.h>
// registration header
#include "volumeRegistrator.h"
#include <twoprojectionregistration.h>
#include <setuptcpcalibrator.h>

/*=============================================================
Above are Headers for the Node Editor plugin
==============================================================*/

/*=============================================================
Below are Headers for DRR testing
==============================================================*/
#include "itkImage.h"
// #include "itkImageFileReader.h"
#include "itkCenteredEuler3DTransform.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkNearestNeighborInterpolateImageFunction.h"
#include "itkResampleImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
//---

#include "itkRayCastInterpolateImageFunction.h"
#include "mitkImageCast.h"
#include <boost/numeric/conversion/bounds.hpp>

#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <iostream>

inline void NodeEditor::DrrCtImageChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_DrrCtImageDataNode = m_Controls.drrCtImageSingleNodeSelectionWidget->GetSelectedNode();
}

inline void NodeEditor::NewDrrCtImageChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_NewDrrCtImageDataNode = m_Controls.newDrrCtImageSingleNodeSelectionWidget->GetSelectedNode();
}

inline void NodeEditor::RegistrationCtImageChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_RegistrationCtImageDataNode = m_Controls.registrationCtSingleNodeSelectionWidget->GetSelectedNode();
}

inline void NodeEditor::InputDrrImageChanged_1(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_InputDrrImageDataNode_1 = m_Controls.registrationDrr1SingleNodeSelectionWidget->GetSelectedNode();
}

inline void NodeEditor::InputDrrImageChanged_2(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_InputDrrImageDataNode_2 = m_Controls.registrationDrr2SingleNodeSelectionWidget->GetSelectedNode();
}

inline void NodeEditor::InputImageToCropChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_InputImageToCropDataNode = m_Controls.widget_CropImage->GetSelectedNode();
}

inline void NodeEditor::InputSurfaceChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_InputSurfaceDataNode = m_Controls.widget_Poly->GetSelectedNode();
}

void NodeEditor::RawCtImageChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_RawCtImageDataNode = m_Controls.rawCtImageSingleNodeSelectionWidget->GetSelectedNode();
}

void NodeEditor::EvaluationPointsChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_EvaluationPointsDataNode = m_Controls.evaluationPointsSingleNodeSelectionWidget->GetSelectedNode();
}

void NodeEditor::NewRegistrationCtChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_NewRegistrationCtDataNode = m_Controls.newRegistrationCtSingleNodeSelectionWidget->GetSelectedNode();
}

void NodeEditor::RegDrr1Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_RegDrr1DataNode = m_Controls.regDrr1SingleNodeSelectionWidget->GetSelectedNode();
}

void NodeEditor::RegDrr2Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_RegDrr2DataNode = m_Controls.regDrr2SingleNodeSelectionWidget->GetSelectedNode();
}

void NodeEditor::EvaluateRegistration()
{
  auto image_original_ct = dynamic_cast<mitk::Image *>(m_RawCtImageDataNode->GetData());
  auto pointset_register_points = dynamic_cast<mitk::PointSet *>(m_EvaluationPointsDataNode->GetData());
  auto pointset_real_points = mitk::PointSet::New();
  for (unsigned n = 0; n < pointset_register_points->GetSize(); n++)
  {
    pointset_real_points->InsertPoint(pointset_register_points->GetPoint(n));
  }

  mitk::Point3D ct_center = image_original_ct->GetGeometry()->GetCenter();

  double x_axis[3] = {1, 0, 0};
  double y_axis[3] = {0, 1, 0};
  double z_axis[3] = {0, 0, 1};
  mitk::Vector3D axis_z{z_axis};
  mitk::Vector3D axis_y{y_axis};
  mitk::Vector3D axis_x{x_axis};
  mitk::Point3D rotateCenter{ct_center};

  double rotation_x_real = (m_Controls.sampleRotationXLineEdit->text()).toDouble();
  double rotation_y_real = (m_Controls.sampleRotationYLineEdit->text()).toDouble();
  double rotation_z_real = (m_Controls.sampleRotationZLineEdit->text()).toDouble();

  double translation_x_real = (m_Controls.sampleTranslationXLineEdit->text()).toDouble();
  double translation_y_real = (m_Controls.sampleTranslationYLineEdit->text()).toDouble();
  double translation_z_real = (m_Controls.sampleTranslationZLineEdit->text()).toDouble(); // ZYX

  double rotation_x_register = (m_Controls.registrationRotationXLineEdit->text()).toDouble();
  double rotation_y_register = (m_Controls.registrationRotationYLineEdit->text()).toDouble();
  double rotation_z_register = (m_Controls.registrationRotationZLineEdit->text()).toDouble();

  double translation_x_register = (m_Controls.registrationTranslationXLineEdit->text()).toDouble();
  double translation_y_register = (m_Controls.registrationTranslationYLineEdit->text()).toDouble();
  double translation_z_register = (m_Controls.registrationTranslationZLineEdit->text()).toDouble(); // ZYX

  auto *rotate_operation_real_z = new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, axis_z, rotation_z_real);
  pointset_real_points->GetGeometry()->ExecuteOperation(rotate_operation_real_z);
  auto *rotate_operation_real_y = new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, axis_y, rotation_y_real);
  pointset_real_points->GetGeometry()->ExecuteOperation(rotate_operation_real_y);
  auto *rotate_operation_real_x = new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, axis_x, rotation_x_real);
  pointset_real_points->GetGeometry()->ExecuteOperation(rotate_operation_real_x);
  delete rotate_operation_real_z;
  delete rotate_operation_real_y;
  delete rotate_operation_real_x;

  auto *rotate_operation_register_z =
    new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, axis_z, rotation_z_register);
  pointset_register_points->GetGeometry()->ExecuteOperation(rotate_operation_register_z);
  auto *rotate_operation_register_y =
    new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, axis_y, rotation_y_register);
  pointset_register_points->GetGeometry()->ExecuteOperation(rotate_operation_register_y);
  auto *rotate_operation_register_x =
    new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, axis_x, rotation_x_register);
  pointset_register_points->GetGeometry()->ExecuteOperation(rotate_operation_register_x);
  delete rotate_operation_register_z;
  delete rotate_operation_register_y;
  delete rotate_operation_register_x;

  double direction_real[3] = {translation_x_real, translation_y_real, translation_z_real};
  mitk::Point3D translation_dir_real{direction_real};
  auto *translation_real = new mitk::PointOperation(mitk::OpMOVE, 0, translation_dir_real, 0);
  pointset_real_points->GetGeometry()->ExecuteOperation(translation_real);
  delete translation_real;
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();

  double direction_register[3] = {translation_x_register, translation_y_register, translation_z_register};
  mitk::Point3D translation_dir_register{direction_register};
  auto *translation_register = new mitk::PointOperation(mitk::OpMOVE, 0, translation_dir_register, 0);
  pointset_register_points->GetGeometry()->ExecuteOperation(translation_register);
  delete translation_register;
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();

  for (unsigned n = 0; n < pointset_register_points->GetSize(); n++)
  {
    mitk::Point3D points_register = pointset_register_points->GetPoint(n);
    mitk::Point3D point_real = pointset_real_points->GetPoint(n);
    double p1[3]{points_register[0], points_register[1], points_register[2]};
    double p2[3]{point_real[0], point_real[1], point_real[2]};
    double distance = lancetAlgorithm::DistanceOfTwoPoints(p1, p2);
    m_Controls.evaluationTextBrowser->append("Deviation of point " + QString::number(n) + " is " +
                                             QString::number(distance));
  }
}

void NodeEditor::ConvertPolyDataToImage()
{
  auto imageToCrop = dynamic_cast<mitk::Image *>(m_InputImageToCropDataNode->GetData());
  auto objectSurface = dynamic_cast<mitk::Surface *>(m_InputSurfaceDataNode->GetData());
  // imageToCrop->GetPixelType()
  mitk::Point3D imageCenter = imageToCrop->GetGeometry()->GetCenter();
  mitk::Point3D surfaceCenter = objectSurface->GetGeometry()->GetOrigin();
  double direction[3]{
    surfaceCenter[0] - imageCenter[0], surfaceCenter[1] - imageCenter[1], surfaceCenter[2] - imageCenter[2]};
  TranslateImage(direction, imageToCrop);

  mitk::Image::Pointer convertedImage = mitk::Image::New();
  // stencil
  mitk::SurfaceToImageFilter::Pointer surfaceToImageFilter = mitk::SurfaceToImageFilter::New();
  surfaceToImageFilter->SetImage(imageToCrop);
  surfaceToImageFilter->SetInput(objectSurface);
  surfaceToImageFilter->SetReverseStencil(true);
  surfaceToImageFilter->SetBackgroundValue(1000.0);
  // surfaceToImageFilter->SetMakeOutputBinary(true);
  surfaceToImageFilter->Update();
  // boundingBox
  auto boundingBox = mitk::GeometryData::New();
  // InitializeWithSurfaceGeometry
  auto boundingGeometry = mitk::Geometry3D::New();
  auto geometry = objectSurface->GetGeometry();
  boundingGeometry->SetBounds(geometry->GetBounds());
  boundingGeometry->SetOrigin(geometry->GetOrigin());
  boundingGeometry->SetSpacing(geometry->GetSpacing());
  boundingGeometry->SetIndexToWorldTransform(geometry->GetIndexToWorldTransform()->Clone());
  boundingGeometry->Modified();
  boundingBox->SetGeometry(boundingGeometry);

  auto cutter = mitk::BoundingShapeCropper::New();
  cutter->SetGeometry(boundingBox);
  cutter->SetInput(surfaceToImageFilter->GetOutput());
  cutter->Update();
  convertedImage = cutter->GetOutput()->Clone();

  QString renameSuffix = "_converted";
  QString outputFilename = m_Controls.drrOutputFilenameLineEdit->text();
  auto existingNode = GetDataStorage()->GetNamedNode((outputFilename).toLocal8Bit().data());
  auto newNode = mitk::DataNode::New();
  // in case the output name already exists
  if (existingNode == nullptr)
  {
    newNode->SetName(outputFilename.toLocal8Bit().data());
  }
  else
  {
    newNode->SetName(outputFilename.append(renameSuffix).toLocal8Bit().data());
    m_Controls.drrOutputFilenameLineEdit->setText(outputFilename);
  }
  // add new node
  newNode->SetData(convertedImage);
  GetDataStorage()->Add(newNode);
}
void NodeEditor::PolyDataToImageWithWhiteBackGround()
{
  // auto imageToCrop = dynamic_cast<mitk::Image *>(m_InputImageToCropDataNode->GetData());
  auto objectSurface = dynamic_cast<mitk::Surface *>(m_InputSurfaceDataNode->GetData());

  vtkNew<vtkImageData> whiteImage;
  double imageBounds[6]{0};
  double imageSpacing[3]{1, 1, 1};
  whiteImage->SetSpacing(imageSpacing);

  auto geometry = objectSurface->GetGeometry();
  auto surfaceBounds = geometry->GetBounds();
  for (int n = 0; n < 6; n++)
  {
    imageBounds[n] = surfaceBounds[n];
  }

  int dim[3];
  for (int i = 0; i < 3; i++)
  {
    dim[i] = static_cast<int>(ceil((imageBounds[i * 2 + 1] - imageBounds[i * 2]) / imageSpacing[i]));
  }
  whiteImage->SetDimensions(dim);
  whiteImage->SetExtent(0, dim[0] - 1, 0, dim[1] - 1, 0, dim[2] - 1);

  double origin[3];
  origin[0] = imageBounds[0] + imageSpacing[0] / 2;
  origin[1] = imageBounds[2] + imageSpacing[1] / 2;
  origin[2] = imageBounds[4] + imageSpacing[2] / 2;
  whiteImage->SetOrigin(origin);
  whiteImage->AllocateScalars(VTK_SHORT, 1);

  // fill the image with foreground voxels:
  short inval = 1024;
  short outval = 0;
  vtkIdType count = whiteImage->GetNumberOfPoints();
  for (vtkIdType i = 0; i < count; ++i)
  {
    whiteImage->GetPointData()->GetScalars()->SetTuple1(i, inval);
  }

  auto imageToCrop = mitk::Image::New();
  imageToCrop->Initialize(whiteImage);
  imageToCrop->SetVolume(whiteImage->GetScalarPointer());

  // imageToCrop->GetPixelType()
  // mitk::Point3D imageCenter = imageToCrop->GetGeometry()->GetCenter();
  // mitk::Point3D surfaceCenter = objectSurface->GetGeometry()->GetOrigin();
  // double direction[3]{
  //   surfaceCenter[0] - imageCenter[0], surfaceCenter[1] - imageCenter[1], surfaceCenter[2] - imageCenter[2]};
  // TranslateImage(direction, imageToCrop);

  // mitk::Image::Pointer convertedImage = mitk::Image::New();
  // stencil
  mitk::SurfaceToImageFilter::Pointer surfaceToImageFilter = mitk::SurfaceToImageFilter::New();
  surfaceToImageFilter->SetImage(imageToCrop);
  surfaceToImageFilter->SetInput(objectSurface);
  surfaceToImageFilter->SetReverseStencil(false);
  // surfaceToImageFilter->SetMakeOutputBinary(true);
  surfaceToImageFilter->Update();

  // // boundingBox
  // auto boundingBox = mitk::GeometryData::New();
  // // InitializeWithSurfaceGeometry
  // auto boundingGeometry = mitk::Geometry3D::New();
  // auto geometry = objectSurface->GetGeometry();
  // boundingGeometry->SetBounds(geometry->GetBounds());
  // boundingGeometry->SetOrigin(geometry->GetOrigin());
  // boundingGeometry->SetSpacing(geometry->GetSpacing());
  // boundingGeometry->SetIndexToWorldTransform(geometry->GetIndexToWorldTransform()->Clone());
  // boundingGeometry->Modified();
  // boundingBox->SetGeometry(boundingGeometry);
  //
  // auto cutter = mitk::BoundingShapeCropper::New();
  // cutter->SetGeometry(boundingBox);
  // cutter->SetInput(surfaceToImageFilter->GetOutput());
  // cutter->Update();
  // convertedImage = cutter->GetOutput()->Clone();

  QString renameSuffix = "_converted";
  QString outputFilename = m_Controls.drrOutputFilenameLineEdit->text();
  auto existingNode = GetDataStorage()->GetNamedNode((outputFilename).toLocal8Bit().data());
  auto newNode = mitk::DataNode::New();
  // in case the output name already exists
  if (existingNode == nullptr)
  {
    newNode->SetName(outputFilename.toLocal8Bit().data());
  }
  else
  {
    newNode->SetName(outputFilename.append(renameSuffix).toLocal8Bit().data());
    m_Controls.drrOutputFilenameLineEdit->setText(outputFilename);
  }
  // add new node
  newNode->SetData(surfaceToImageFilter->GetOutput());
  GetDataStorage()->Add(newNode);
};

void NodeEditor::GenerateWhiteImage()
{
  auto objectSurface = dynamic_cast<mitk::Surface *>(m_InputSurfaceDataNode->GetData());

  vtkNew<vtkImageData> whiteImage;
  double imageBounds[6]{0};
  double imageSpacing[3]{1, 1, 1};
  whiteImage->SetSpacing(imageSpacing);

  auto geometry = objectSurface->GetGeometry();
  auto surfaceBounds = geometry->GetBounds();
  for (int n = 0; n < 6; n++)
  {
    imageBounds[n] = surfaceBounds[n];
  }

  int dim[3];
  for (int i = 0; i < 3; i++)
  {
    dim[i] = static_cast<int>(ceil((imageBounds[i * 2 + 1] - imageBounds[i * 2]) / imageSpacing[i]));
  }
  whiteImage->SetDimensions(dim);
  whiteImage->SetExtent(0, dim[0] - 1, 0, dim[1] - 1, 0, dim[2] - 1);

  double origin[3];
  origin[0] = imageBounds[0] + imageSpacing[0] / 2;
  origin[1] = imageBounds[2] + imageSpacing[1] / 2;
  origin[2] = imageBounds[4] + imageSpacing[2] / 2;
  whiteImage->SetOrigin(origin);
  whiteImage->AllocateScalars(VTK_SHORT, 1);

  // fill the image with foreground voxels:
  short insideValue = 1024;
  short outsideValue = 0;
  vtkIdType count = whiteImage->GetNumberOfPoints();
  for (vtkIdType i = 0; i < count; ++i)
  {
    whiteImage->GetPointData()->GetScalars()->SetTuple1(i, insideValue);
  }

  auto imageToCrop = mitk::Image::New();
  imageToCrop->Initialize(whiteImage);
  imageToCrop->SetVolume(whiteImage->GetScalarPointer());

  // add a new node for the white image
  auto newNode = mitk::DataNode::New();
  newNode->SetName("White Image");
  newNode->SetData(imageToCrop);
  GetDataStorage()->Add(newNode);
}
void NodeEditor::PolyDataToImageData()
{
  auto imageToCrop = dynamic_cast<mitk::Image *>(m_InputImageToCropDataNode->GetData());
  auto objectSurface = dynamic_cast<mitk::Surface *>(m_InputSurfaceDataNode->GetData());

  mitk::Image::Pointer convertedImage = mitk::Image::New();
  // stencil
  mitk::SurfaceToImageFilter::Pointer surfaceToImageFilter = mitk::SurfaceToImageFilter::New();
  surfaceToImageFilter->SetImage(imageToCrop);
  surfaceToImageFilter->SetInput(objectSurface);
  surfaceToImageFilter->SetReverseStencil(false);

  surfaceToImageFilter->Update();
  convertedImage = surfaceToImageFilter->GetOutput();

  auto newNode = mitk::DataNode::New();

  newNode->SetName("Generated CT image");

  // add new node
  newNode->SetData(convertedImage);
  GetDataStorage()->Add(newNode);
}

void NodeEditor::SetUiDefault()
{
  m_Controls.gantryRotationLineEdit->setText("0.0");
  m_Controls.sampleTranslationXLineEdit->setText("0.0");
  m_Controls.sampleTranslationYLineEdit->setText("0.0");
  m_Controls.sampleTranslationZLineEdit->setText("0.0");
  m_Controls.sampleRotationXLineEdit->setText("0");
  m_Controls.sampleRotationYLineEdit->setText("0.0");
  m_Controls.sampleRotationZLineEdit->setText("0.0");
  m_Controls.isocenterOffsetXLineEdit->setText("0.0");
  m_Controls.isocenterOffsetYLineEdit->setText("0.0");
  m_Controls.isocenterOffsetZLineEdit->setText("0.0");
  m_Controls.drrThresholdLineEdit->setText("0.0");
  m_Controls.drrSourceToIsoDistanceLineEdit->setText("607");
  m_Controls.drrOutputResolutionXLineEdit->setText("1.0");
  m_Controls.drrOutputResolutionYLineEdit->setText("1.0");
  m_Controls.centralAxisOffsetXLineEdit->setText("0.0");
  m_Controls.centralAxisOffsetYLineEdit->setText("0.0");
  m_Controls.drrOutputSizeXLineEdit->setText("512");
  m_Controls.drrOutputSizeYLineEdit->setText("512");
}

void NodeEditor::Drr()
{
  DrrVisualization();
  DrrGenerateData();
}

void NodeEditor::DrrGenerateData()
{
  if (m_DrrCtImageDataNode == nullptr)
  {
    MITK_ERROR << "m_DrrCtImageDataNode null";
    return;
  }
  // the original input image node will be named "unnamed", and you have to rename it because it really does not have a
  // name!!
  // auto image =
  // GetDataStorage()->GetNamedObject<mitk::Image>((m_Controls.inputFilename->text()).toLocal8Bit().data());
  auto image = dynamic_cast<mitk::Image *>(m_DrrCtImageDataNode->GetData());
  // auto sliced = dynamic_cast<mitk::SlicedData *>(m_DrrCtImageDataNode->GetData());

  // auto image = dynamic_cast<mitk::Image *>(sliced);
  // the original input image node will be named "unnamed", and you have to rename it because it really does not have a
  // name!!
  if (image == nullptr)
  {
    MITK_ERROR << "Can't Run DRR: Image null";
    m_Controls.drrTextBrowser->append("Error: Input image node is empty");
    return;
  }
  itk::SmartPointer<DRRSidonJacobsRayTracingFilter> drrFilter = DRRSidonJacobsRayTracingFilter::New();

  drrFilter->SetInput(image);

  double rprojection = (m_Controls.gantryRotationLineEdit->text()).toDouble();
  double tx = (m_Controls.sampleTranslationXLineEdit->text()).toDouble();
  double ty = (m_Controls.sampleTranslationYLineEdit->text()).toDouble();
  double tz = (m_Controls.sampleTranslationZLineEdit->text()).toDouble();
  double rx = (m_Controls.sampleRotationXLineEdit->text()).toDouble();
  double ry = (m_Controls.sampleRotationYLineEdit->text()).toDouble();
  double rz = (m_Controls.sampleRotationZLineEdit->text()).toDouble();
  double cx = (m_Controls.isocenterOffsetXLineEdit->text()).toDouble();
  double cy = (m_Controls.isocenterOffsetYLineEdit->text()).toDouble();
  double cz = (m_Controls.isocenterOffsetZLineEdit->text()).toDouble();
  double threshold = (m_Controls.drrThresholdLineEdit->text()).toDouble();
  double scd = (m_Controls.drrSourceToIsoDistanceLineEdit->text()).toDouble();
  double sx = (m_Controls.drrOutputResolutionXLineEdit->text()).toDouble();
  double sy = (m_Controls.drrOutputResolutionYLineEdit->text()).toDouble();
  double o2Dx = (m_Controls.centralAxisOffsetXLineEdit->text()).toDouble();
  double o2Dy = (m_Controls.centralAxisOffsetYLineEdit->text()).toDouble();
  int dx = (m_Controls.drrOutputSizeXLineEdit->text()).toInt();
  int dy = (m_Controls.drrOutputSizeYLineEdit->text()).toInt();

  drrFilter->Setrprojection(rprojection);
  drrFilter->SetObjTranslate(tx, ty, tz);
  drrFilter->SetObjRotate(rx, ry, rz);
  drrFilter->Setcx(cx);
  drrFilter->Setcy(cy);
  drrFilter->Setcz(cz);
  drrFilter->Setthreshold(threshold);
  drrFilter->Setscd(scd);
  drrFilter->Setim_sx(sx);
  drrFilter->Setim_sy(sy);
  drrFilter->Setdx(dx);
  drrFilter->Setdy(dy);
  drrFilter->Seto2Dx(o2Dx);
  drrFilter->Seto2Dy(o2Dy);
  drrFilter->Update();

  QString renameSuffix = "_new";
  QString outputFilename = m_Controls.drrOutputFilenameLineEdit->text();
  auto node = GetDataStorage()->GetNamedNode(outputFilename.toLocal8Bit().data());
  auto newnode = mitk::DataNode::New();
  // in case the output name already exists
  if (node == nullptr)
  {
    newnode->SetName(outputFilename.toLocal8Bit().data());
  }
  else
  {
    newnode->SetName(outputFilename.append(renameSuffix).toLocal8Bit().data());
    m_Controls.drrOutputFilenameLineEdit->setText(outputFilename);
  }
  // add new node
  newnode->SetData(drrFilter->GetOutput());
  GetDataStorage()->Add(newnode);
}
void NodeEditor::DrrVisualization()
{
  if (m_DrrCtImageDataNode == nullptr)
  {
    MITK_ERROR << "m_DrrCtImageDataNode null";
    return;
  }
  // the original input image node will be named "unnamed", and you have to rename it because it really does not have a
  // name!!
  // auto image =
  // GetDataStorage()->GetNamedObject<mitk::Image>((m_Controls.inputFilename->text()).toLocal8Bit().data());
  auto image = dynamic_cast<mitk::Image *>(m_DrrCtImageDataNode->GetData());
  // auto sliced = dynamic_cast<mitk::SlicedData *>(m_DrrCtImageDataNode->GetData());

  // auto image = dynamic_cast<mitk::Image *>(sliced);
  // the original input image node will be named "unnamed", and you have to rename it because it really does not have a
  // name!!
  if (image == nullptr)
  {
    MITK_ERROR << "Can't Run DRR: Image null";
    m_Controls.drrTextBrowser->append("Error: Input image node is empty");
    return;
  }
  itk::SmartPointer<DRRSidonJacobsRayTracingFilter> drrFilter = DRRSidonJacobsRayTracingFilter::New();

  drrFilter->SetInput(image);

  double rprojection = (m_Controls.gantryRotationLineEdit->text()).toDouble();
  double tx = (m_Controls.sampleTranslationXLineEdit->text()).toDouble();
  double ty = (m_Controls.sampleTranslationYLineEdit->text()).toDouble();
  double tz = (m_Controls.sampleTranslationZLineEdit->text()).toDouble();
  double rx = (m_Controls.sampleRotationXLineEdit->text()).toDouble();
  double ry = (m_Controls.sampleRotationYLineEdit->text()).toDouble();
  double rz = (m_Controls.sampleRotationZLineEdit->text()).toDouble();
  double cx = (m_Controls.isocenterOffsetXLineEdit->text()).toDouble();
  double cy = (m_Controls.isocenterOffsetYLineEdit->text()).toDouble();
  double cz = (m_Controls.isocenterOffsetZLineEdit->text()).toDouble();
  double threshold = (m_Controls.drrThresholdLineEdit->text()).toDouble();
  double scd = (m_Controls.drrSourceToIsoDistanceLineEdit->text()).toDouble();
  double sx = (m_Controls.drrOutputResolutionXLineEdit->text()).toDouble();
  double sy = (m_Controls.drrOutputResolutionYLineEdit->text()).toDouble();
  double o2Dx = (m_Controls.centralAxisOffsetXLineEdit->text()).toDouble();
  double o2Dy = (m_Controls.centralAxisOffsetYLineEdit->text()).toDouble();
  int dx = (m_Controls.drrOutputSizeXLineEdit->text()).toInt();
  int dy = (m_Controls.drrOutputSizeYLineEdit->text()).toInt();

  drrFilter->Setrprojection(rprojection);
  drrFilter->SetObjTranslate(tx, ty, tz);
  drrFilter->SetObjRotate(rx, ry, rz);
  drrFilter->Setcx(cx);
  drrFilter->Setcy(cy);
  drrFilter->Setcz(cz);
  drrFilter->Setthreshold(threshold);
  drrFilter->Setscd(scd);
  drrFilter->Setim_sx(sx);
  drrFilter->Setim_sy(sy);
  drrFilter->Setdx(dx);
  drrFilter->Setdy(dy);
  drrFilter->Seto2Dx(o2Dx);
  drrFilter->Seto2Dy(o2Dy);
  drrFilter->Update();

  QString renameSuffix = "_new";
  QString outputFilename = m_Controls.drrOutputFilenameLineEdit->text();
  auto node = GetDataStorage()->GetNamedNode(outputFilename.toLocal8Bit().data());
  auto newnode = mitk::DataNode::New();
  // in case the output name already exists
  if (node == nullptr)
  {
    newnode->SetName(outputFilename.toLocal8Bit().data());
  }
  else
  {
    newnode->SetName(outputFilename.append(renameSuffix).toLocal8Bit().data());
    m_Controls.drrOutputFilenameLineEdit->setText(outputFilename);
  }

  // add new node for DRR geometry visualization

  mitk::Image::Pointer image_trans = drrFilter->GetOutput();
  mitk::Point3D c_k = m_DrrCtImageDataNode->GetData()->GetGeometry()->GetCenter();

  double c_v[3]{c_k[0], c_k[1], c_k[2]};

  // rotate 90 degrees to fit the DRR geometry
  double x_axis[3]{1, 0, 0};
  double isoc[3]{0, 0, -scd};
  RotateImage(isoc, x_axis, -90, image_trans);

  // move the center of the image to the isocenter in the sample coordinates
  double p[3]{c_v[0] + cx, c_v[1] + cy, c_v[2] + cy + scd}; // translation vector
  // mitk::Point3D direciton{p};
  TranslateImage(p, image_trans);

  double isocw[3]{c_v[0] + cx, c_v[1] + cy, c_v[2] + cz};

  m_Controls.isocenterXLineEdit->setText(QString::number(isocw[0]));
  m_Controls.isocenterYLineEdit->setText(QString::number(isocw[1]));
  m_Controls.isocenterZLineEdit->setText(QString::number(isocw[2]));

  // move the image by some -y for better visualization
  double p_1[3]{0, scd, 0};
  TranslateImage(p_1, image_trans);

  // gantry rotation offset
  double z_axis[3]{0, 0, 1};
  RotateImage(isocw, z_axis, rprojection, image_trans);

  // mitk::RenderingManager::GetInstance()->RequestUpdateAll();

  auto geo_node = mitk::DataNode::New();
  QString geo_Suffix = "_visual";
  geo_node->SetName(outputFilename.append(geo_Suffix).toLocal8Bit().data());
  geo_node->SetData(image_trans);
  GetDataStorage()->Add(geo_node);

  if (m_Controls.generateMovedCtCheckBox->isChecked())
  {
    // add a node that contains the moved CT image
    itk::Image<short, 3>::Pointer m_movedCTimage;
    mitk::Image::Pointer image_tmp;
    mitk::CastToItkImage(image, m_movedCTimage);
    mitk::CastToMitkImage(m_movedCTimage, image_tmp);
    double Z_axis[3]{0, 0, 1};
    RotateImage(isocw, Z_axis, rz, image_tmp);
    double Y_axis[3]{0, 1, 0};
    RotateImage(isocw, Y_axis, ry, image_tmp);
    double X_axis[3]{1, 0, 0};
    RotateImage(isocw, X_axis, rz, image_tmp);
    double p_tmp[3]{tx, ty, tz};
    TranslateImage(p_tmp, image_tmp);

    auto movedCT_node = mitk::DataNode::New();
    QString movedCT_Suffix = "_sample";
    movedCT_node->SetName(outputFilename.append(movedCT_Suffix).toLocal8Bit().data());
    movedCT_node->SetData(image_tmp);
    GetDataStorage()->Add(movedCT_node);
  }

  c_v[0] = (c_v[0] + cx) + scd * sin(rprojection * 3.1415926 / 180);
  c_v[1] = (c_v[1] + cy) - scd * cos(rprojection * 3.1415926 / 180);
  c_v[2] = c_v[2] + cz;
  // double xsourcew[3]{c_v[0] + cx, c_v[1] + cy - scd, c_v[2] + cz};

  m_Controls.raySourceXLineEdit->setText(QString::number(c_v[0]));
  m_Controls.raySourceYLineEdit->setText(QString::number(c_v[1]));
  m_Controls.raySourceZLineEdit->setText(QString::number(c_v[2]));

  if (m_Controls.generateRaySourceCheckBox->isChecked())
  {
    mitk::Point3D point3D_raySource;
    point3D_raySource[0] = c_v[0];
    point3D_raySource[1] = c_v[1];
    point3D_raySource[2] = c_v[2];

    mitk::Point3D point3D_isocenter;
    point3D_isocenter[0] = isocw[0];
    point3D_isocenter[1] = isocw[1];
    point3D_isocenter[2] = isocw[2];

    auto pointSet_raySource = mitk::PointSet::New();
    pointSet_raySource->InsertPoint(point3D_raySource);
    pointSet_raySource->InsertPoint(point3D_isocenter);

    auto raySourceDataNode = mitk::DataNode::New();
    QString movedCT_Suffix = "_raySource";
    raySourceDataNode->SetName((outputFilename + movedCT_Suffix).toLocal8Bit().data());
    raySourceDataNode->SetData(pointSet_raySource);
    GetDataStorage()->Add(raySourceDataNode);
  }
}

void NodeEditor::V1DrrGenerateData() // this method incorporates the MITK coordinate system which can be regarded as the
                                     // NDI coordinate system later
{
  if (m_NewDrrCtImageDataNode == nullptr)
  {
    MITK_ERROR << "m_NewDrrCtImageDataNode null";
    return;
  }
  //-------------Below: Get the size and spacing of the input image-----------
  auto image = dynamic_cast<mitk::Image *>(m_NewDrrCtImageDataNode->GetData());
  typedef itk::Image<float, 3> TempImageType;
  TempImageType::Pointer image_tmp;
  mitk::CastToItkImage(image, image_tmp);
  const typename TempImageType::SpacingType spacing_temp = image_tmp->GetSpacing();
  typedef typename TempImageType::RegionType TempRegionType;
  typename TempRegionType region = image_tmp->GetBufferedRegion();
  typename TempRegionType::SizeType size_temp = region.GetSize();
  int dx = (m_Controls.imagerPixelNumXLineEdit->text()).toInt();
  int dy = (m_Controls.imagerPixelNumYLineEdit->text()).toInt();
  double sx = (m_Controls.imagerPixelSizeXLineEdit->text()).toDouble();
  double sy = (m_Controls.imagerPixelSizeYLineEdit->text()).toDouble();
  //-------------Above: Get the size and spacing of the input image-----------

  //-------------Below: Construct the Affine transform between coordinate systems: MITK scene, CT volume, c-arm imager,
  //c-arm internal CT volume-------------------------
  // Axes transform from "MITK frame" to "imager frame"
  auto transMitk2Imager = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixMitk2Imager = vtkSmartPointer<vtkMatrix4x4>::New();
  transMitk2Imager->Identity();
  transMitk2Imager->PostMultiply();
  transMitk2Imager->RotateZ(m_Controls.imagerRzLineEdit->text().toDouble());
  transMitk2Imager->RotateY(m_Controls.imagerRyLineEdit->text().toDouble());
  transMitk2Imager->RotateX(m_Controls.imagerRxLineEdit->text().toDouble());
  double translationMitk2Imager[3] = {m_Controls.imagerTxLineEdit->text().toDouble(),
                                      m_Controls.imagerTyLineEdit->text().toDouble(),
                                      m_Controls.imagerTzLineEdit->text().toDouble()};
  transMitk2Imager->Translate(translationMitk2Imager);
  transMitk2Imager->Update();
  transMitk2Imager->GetMatrix(matrixMitk2Imager);

  // Axes transform from "MITK frame" to "CT image frame"
  auto transMitk2Ct = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixMitk2Ct = vtkSmartPointer<vtkMatrix4x4>::New();
  // auto transCt2Mitk = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixCt2Mitk = vtkSmartPointer<vtkMatrix4x4>::New();
  transMitk2Ct->Identity();
  transMitk2Ct->PostMultiply();
  transMitk2Ct->RotateZ(m_Controls.ctRzLineEdit->text().toDouble());
  transMitk2Ct->RotateY(m_Controls.ctRyLineEdit->text().toDouble());
  transMitk2Ct->RotateX(m_Controls.ctRxLineEdit->text().toDouble());
  double translationMitk2Ct[3] = {m_Controls.ctTxLineEdit->text().toDouble(),
                                  m_Controls.ctTyLineEdit->text().toDouble(),
                                  m_Controls.ctTzLineEdit->text().toDouble()};
  transMitk2Ct->Translate(translationMitk2Ct);
  transMitk2Ct->Update();
  transMitk2Ct->GetMatrix(matrixMitk2Ct);
  transMitk2Ct->GetInverse(matrixCt2Mitk);

  // Axes transform from "imager frame" to "original C-arm internal CT image frame"
  auto transImager2InternalCt = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixImager2InternalCt = vtkSmartPointer<vtkMatrix4x4>::New();
  transImager2InternalCt->Identity();
  transImager2InternalCt->PostMultiply();
  transImager2InternalCt->RotateX(-90);
  double translationImager2InternalCt[3] = {
    m_Controls.sourceXLineEdit->text().toDouble() - spacing_temp[0] * double(size_temp[0]) / 2.0,
    m_Controls.sourceYLineEdit->text().toDouble() - spacing_temp[2] * double(size_temp[2]) / 2.0,
    spacing_temp[1] * double(size_temp[1]) / 2.0};

  transImager2InternalCt->Translate(translationImager2InternalCt);
  transImager2InternalCt->Update();
  transImager2InternalCt->GetMatrix(matrixImager2InternalCt);

  // Axes transform from "original C-arm internal CT image frame" to "CT image frame"
  auto transCt2InternalCt = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixInternalCt2Ct = vtkSmartPointer<vtkMatrix4x4>::New();
  transCt2InternalCt->Identity();
  transCt2InternalCt->PostMultiply();
  transCt2InternalCt->Concatenate(matrixImager2InternalCt);
  transCt2InternalCt->Concatenate(matrixMitk2Imager);
  transCt2InternalCt->Concatenate(matrixCt2Mitk);
  transCt2InternalCt->Update();
  transCt2InternalCt->GetInverse(matrixInternalCt2Ct);
  //-------------Above: Construct the Affine transform between coordinate systems-------------------------

  //----------Below: Extract ZYX angles from the affine transform matrix----------
  double rx, ry, rz;
  double piParameter = 180 / 3.1415926;
  if (matrixInternalCt2Ct->GetElement(0, 2) < 1)
  {
    if (matrixInternalCt2Ct->GetElement(0, 2) > -1)
    {
      ry = asin(matrixInternalCt2Ct->GetElement(0, 2));
      rx = atan2(-matrixInternalCt2Ct->GetElement(1, 2), matrixInternalCt2Ct->GetElement(2, 2));
      rz = atan2(-matrixInternalCt2Ct->GetElement(0, 1), matrixInternalCt2Ct->GetElement(0, 0));
    }
    else
    {
      ry = -3.1415926 / 2;
      rx = -atan2(matrixInternalCt2Ct->GetElement(1, 0), matrixInternalCt2Ct->GetElement(1, 1));
      rz = 0;
    }
  }
  else
  {
    ry = 3.1415926 / 2;
    rx = atan2(matrixInternalCt2Ct->GetElement(1, 0), matrixInternalCt2Ct->GetElement(1, 1));
    rz = 0;
  }
  rx = rx * piParameter;
  ry = ry * piParameter;
  rz = rz * piParameter;
  //----------Above: Extract ZYX angles from the affine transform matrix----------

  //----------Below: Construct a filter and feed in the image and the parameters generated above--------------
  itk::SmartPointer<DRRSidonJacobsRayTracingFilter> drrFilter = DRRSidonJacobsRayTracingFilter::New();

  drrFilter->SetInput(image);

  // The volume center of the internal Ct volume under the internal Ct coordinate system
  Eigen::Vector4d internalCtCenter{spacing_temp[0] * double(size_temp[0]) / 2.0,
                                   spacing_temp[1] * double(size_temp[1]) / 2.0,
                                   spacing_temp[2] * double(size_temp[2]) / 2.0,
                                   1};
  // The center of the real Ct volume under the real Ct coordinate system
  Eigen::Vector4d ctCenter{spacing_temp[0] * double(size_temp[0]) / 2.0,
                           spacing_temp[1] * double(size_temp[1]) / 2.0,
                           spacing_temp[2] * double(size_temp[2]) / 2.0,
                           1};
  Eigen::Matrix4d eigenMatrixInternalCt2Ct{matrixInternalCt2Ct->GetData()};
  eigenMatrixInternalCt2Ct.transposeInPlace();
  // The volume center of the real Ct volume under the internal Ct coordinate system
  Eigen::Vector4d targetCenterPoint = eigenMatrixInternalCt2Ct * ctCenter;

  double tx = targetCenterPoint[0] - internalCtCenter[0];
  double ty = targetCenterPoint[1] - internalCtCenter[1];
  double tz = targetCenterPoint[2] - internalCtCenter[2];

  double cx = 0;
  double cy = 0;
  double cz = 0;
  double threshold = (m_Controls.newDrrthresLineEdit->text()).toDouble();
  double scd = (m_Controls.sourceZLineEdit->text()).toDouble();

  double o2Dx = -((m_Controls.sourceXLineEdit->text()).toDouble() - sx * (dx - 1) / 2);
  double o2Dy = -((m_Controls.sourceYLineEdit->text()).toDouble() - sy * (dy - 1) / 2);

  drrFilter->Setrprojection(0);
  drrFilter->SetObjTranslate(tx, ty, tz);
  drrFilter->SetObjRotate(rx, ry, rz);
  drrFilter->Setcx(cx);
  drrFilter->Setcy(cy);
  drrFilter->Setcz(cz);
  drrFilter->Setthreshold(threshold);
  drrFilter->Setscd(scd);
  drrFilter->Setim_sx(sx);
  drrFilter->Setim_sy(sy);
  drrFilter->Setdx(dx);
  drrFilter->Setdy(dy);
  drrFilter->Seto2Dx(o2Dx);
  drrFilter->Seto2Dy(o2Dy);
  drrFilter->Update();

  //-----------Below: add the datanode containing the DRR--------------
  QString renameSuffix = "_new";
  QString outputFilename = m_Controls.drrNameLineEdit->text();
  auto node = GetDataStorage()->GetNamedNode(outputFilename.toLocal8Bit().data());
  auto newnode = mitk::DataNode::New();
  // in case the output name already exists
  if (node == nullptr)
  {
    newnode->SetName(outputFilename.toLocal8Bit().data());
  }
  else
  {
    newnode->SetName(outputFilename.append(renameSuffix).toLocal8Bit().data());
    m_Controls.drrNameLineEdit->setText(outputFilename);
  }
  // add new node
  newnode->SetData(drrFilter->GetOutput());
  GetDataStorage()->Add(newnode);
  //-----------Above: add the datanode containing the DRR--------------

  //-----------------Below: generate a image on the virtual monitor screen------------------
  QString pseudoImageSuffix = "_visual";
  outputFilename = m_Controls.drrNameLineEdit->text();
  auto visualnewnode = mitk::DataNode::New();
  // in case the output name already exists
  visualnewnode->SetName(outputFilename.append(pseudoImageSuffix).toLocal8Bit().data());
  // add new node
  auto pseudoImage = drrFilter->GetOutput()->Clone();
  mitk::Point3D pseudoImageOrigin;
  pseudoImageOrigin[0] = 300 - sx * (dx - 1) / 2;
  pseudoImageOrigin[1] = 300 - sy * (dy - 1) / 2;
  pseudoImageOrigin[2] = 1;
  pseudoImage->GetGeometry()->SetOrigin(pseudoImageOrigin);
  visualnewnode->SetData(pseudoImage);
  GetDataStorage()->Add(visualnewnode);

  //-----------------Above: generate a image on the virtual monitor screen------------------

  //------------Below: Print out the real parameters used for DRR generation ----------------
  m_Controls.newDrrTextBrowser->append("rprojection: " + QString::number(0));
  m_Controls.newDrrTextBrowser->append("tx: " + QString::number(tx));
  m_Controls.newDrrTextBrowser->append("ty: " + QString::number(ty));
  m_Controls.newDrrTextBrowser->append("tz: " + QString::number(tz));
  m_Controls.newDrrTextBrowser->append("rx: " + QString::number(rx));
  m_Controls.newDrrTextBrowser->append("ry: " + QString::number(ry));
  m_Controls.newDrrTextBrowser->append("rz: " + QString::number(rz));
  m_Controls.newDrrTextBrowser->append("scd: " + QString::number(scd));
  m_Controls.newDrrTextBrowser->append("sx: " + QString::number(sx));
  m_Controls.newDrrTextBrowser->append("sy: " + QString::number(sy));
  m_Controls.newDrrTextBrowser->append("dx: " + QString::number(dx));
  m_Controls.newDrrTextBrowser->append("dy: " + QString::number(dy));
  m_Controls.newDrrTextBrowser->append("o2Dx: " + QString::number(o2Dx));
  m_Controls.newDrrTextBrowser->append("o2Dy: " + QString::number(o2Dy));
  //------------Above: Print out the real parameters used for DRR generation ----------------
  //  double m_ArrayMatrixWorldToImager[16]
  // {
  // 	  1,0,0,2,
  // 	  0,1,0,3,
  // 	  0,0,1,4,
  // 	  0,0,0,5
  // };
  //  Eigen::Matrix4d matrixTest{m_ArrayMatrixWorldToImager};
  //  m_Controls.newDrrTextBrowser->append("Element 8" + QString::number(matrixTest(7)));

  // itk::SmartPointer<SetUpTcpCalibrator> tcpCorrector = SetUpTcpCalibrator::New();
  // double pointD[3]{8, 9, 10};
  // double pointP[3]{1, 4, 4};
  // double pointQ[3]{0.292993,4.707106,4};
  // double pointS[3]{1, 4, 3};
  // double arrayMatrix[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1};
  //
  // tcpCorrector->calibrateGooseSaw(arrayMatrix,pointD,pointP,pointQ,pointS);
  // m_Controls.newDrrTextBrowser->append("Rx: " + QString::number(tcpCorrector->GetRx()));
  // m_Controls.newDrrTextBrowser->append("Ry: " + QString::number(tcpCorrector->GetRy()));
  // m_Controls.newDrrTextBrowser->append("Rz: " + QString::number(tcpCorrector->GetRz()));
  // m_Controls.newDrrTextBrowser->append("Tx: " + QString::number(tcpCorrector->GetTx()));
  // m_Controls.newDrrTextBrowser->append("Ty: " + QString::number(tcpCorrector->GetTy()));
  // m_Controls.newDrrTextBrowser->append("Tz: " + QString::number(tcpCorrector->GetTz()));
  //
  // Eigen::Vector3d X(0.204007,-0.177015,0.494579);
  // Eigen::Vector3d x(0.356809,0.0814958,0.930616);
  //
  // m_Controls.newDrrTextBrowser->append("The initial result is " + QString::number(X.dot(x)));
  // m_Controls.newDrrTextBrowser->append("The fixed result is " + QString::number(X(0)*x(0)+X(1)*x(1)+X(2)*x(2)));
}

void NodeEditor::V2DrrGenerateData()
{
  if (m_NewDrrCtImageDataNode == nullptr)
  {
    MITK_ERROR << "m_NewDrrCtImageDataNode null";
    return;
  }
  //-------------Below: Get the size and spacing of the input image-----------
  auto image = dynamic_cast<mitk::Image *>(m_NewDrrCtImageDataNode->GetData());
  typedef itk::Image<float, 3> TempImageType;
  TempImageType::Pointer image_tmp;
  mitk::CastToItkImage(image, image_tmp);
  const typename TempImageType::SpacingType spacing_temp = image_tmp->GetSpacing();
  typedef typename TempImageType::RegionType TempRegionType;
  typename TempRegionType region = image_tmp->GetBufferedRegion();
  typename TempRegionType::SizeType size_temp = region.GetSize();
  int dx = (m_Controls.imagerPixelNumXLineEdit->text()).toInt();
  int dy = (m_Controls.imagerPixelNumYLineEdit->text()).toInt();
  double sx = (m_Controls.imagerPixelSizeXLineEdit->text()).toDouble();
  double sy = (m_Controls.imagerPixelSizeYLineEdit->text()).toDouble();
  //-------------Above: Get the size and spacing of the input image-----------

  //-------------Below: Construct the Affine transform between coordinate systems: MITK scene, CT volume, c-arm imager,
  // and c-arm internal CT volume-------------------------
  // Axes transform from "MITK frame" to "imager frame"
  auto transMitk2Imager = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixMitk2Imager = vtkSmartPointer<vtkMatrix4x4>::New();
  transMitk2Imager->Identity();
  transMitk2Imager->PostMultiply();
  transMitk2Imager->RotateZ(m_Controls.imagerRzLineEdit->text().toDouble());
  transMitk2Imager->RotateY(m_Controls.imagerRyLineEdit->text().toDouble());
  transMitk2Imager->RotateX(m_Controls.imagerRxLineEdit->text().toDouble());
  double translationMitk2Imager[3] = {m_Controls.imagerTxLineEdit->text().toDouble(),
                                      m_Controls.imagerTyLineEdit->text().toDouble(),
                                      m_Controls.imagerTzLineEdit->text().toDouble()};
  transMitk2Imager->Translate(translationMitk2Imager);
  transMitk2Imager->Update();
  transMitk2Imager->GetMatrix(matrixMitk2Imager);

  // Axes transform from "MITK frame" to "CT image frame"
  auto transMitk2Ct = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixMitk2Ct = vtkSmartPointer<vtkMatrix4x4>::New();
  // auto transCt2Mitk = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixCt2Mitk = vtkSmartPointer<vtkMatrix4x4>::New();
  transMitk2Ct->Identity();
  transMitk2Ct->PostMultiply();
  transMitk2Ct->RotateZ(m_Controls.ctRzLineEdit->text().toDouble());
  transMitk2Ct->RotateY(m_Controls.ctRyLineEdit->text().toDouble());
  transMitk2Ct->RotateX(m_Controls.ctRxLineEdit->text().toDouble());
  double translationMitk2Ct[3] = {m_Controls.ctTxLineEdit->text().toDouble(),
                                  m_Controls.ctTyLineEdit->text().toDouble(),
                                  m_Controls.ctTzLineEdit->text().toDouble()};
  transMitk2Ct->Translate(translationMitk2Ct);
  transMitk2Ct->Update();
  transMitk2Ct->GetMatrix(matrixMitk2Ct);

  //----------Below: Construct a filter and feed in the image and the parameters generated above--------------
  itk::SmartPointer<DrrGenerator> drrFilter = DrrGenerator::New();

  // The volume center of the internal Ct volume under the internal Ct coordinate system
  // Eigen::Vector4d internalCtCenter{spacing_temp[0] * double(size_temp[0]) / 2.0,
  //                                  spacing_temp[1] * double(size_temp[1]) / 2.0,
  //                                  spacing_temp[2] * double(size_temp[2]) / 2.0,
  //                                  1};
  // The center of the real Ct volume under the real Ct coordinate system
  // Eigen::Vector4d ctCenter{spacing_temp[0] * double(size_temp[0]) / 2.0,
  //                          spacing_temp[1] * double(size_temp[1]) / 2.0,
  //                          spacing_temp[2] * double(size_temp[2]) / 2.0,
  //                          1};
  // Eigen::Matrix4d eigenMatrixInternalCt2Ct{matrixInternalCt2Ct->GetData()};
  // eigenMatrixInternalCt2Ct.transposeInPlace();
  // // The volume center of the real Ct volume under the internal Ct coordinate system
  // Eigen::Vector4d targetCenterPoint = eigenMatrixInternalCt2Ct * ctCenter;
  //
  // double tx = targetCenterPoint[0] - internalCtCenter[0];
  // double ty = targetCenterPoint[1] - internalCtCenter[1];
  // double tz = targetCenterPoint[2] - internalCtCenter[2];

  double threshold = (m_Controls.newDrrthresLineEdit->text()).toDouble();
  // double scd = (m_Controls.sourceZLineEdit->text()).toDouble();

  // double o2Dx = -((m_Controls.sourceXLineEdit->text()).toDouble() - sx * (dx - 1) / 2);
  // double o2Dy = -((m_Controls.sourceYLineEdit->text()).toDouble() - sy * (dy - 1) / 2);

  // drrFilter->Setrprojection(0);
  // drrFilter->SetObjTranslate(tx, ty, tz);
  // drrFilter->SetObjRotate(rx, ry, rz);
  // drrFilter->Setcx(0);
  // drrFilter->Setcy(0);
  // drrFilter->Setcz(0);
  drrFilter->SetInput(image);
  drrFilter->SetArrayMatrixWorldToCt(matrixMitk2Ct->GetData());
  drrFilter->SetArrayMatrixWorldToImager(matrixMitk2Imager->GetData());
  drrFilter->Setthreshold(threshold);
  double arraySource[3]{m_Controls.sourceXLineEdit->text().toDouble(),
                        m_Controls.sourceYLineEdit->text().toDouble(),
                        m_Controls.sourceZLineEdit->text().toDouble()};
  drrFilter->SetRaySource(arraySource); // source xyz
  drrFilter->Setim_sx(sx);
  drrFilter->Setim_sy(sy);
  drrFilter->Setdx(dx);
  drrFilter->Setdy(dy);
  // drrFilter->Seto2Dx(o2Dx);
  // drrFilter->Seto2Dy(o2Dy);
  drrFilter->Update();

  //-----------Below: add the datanode containing the DRR--------------
  QString renameSuffix = "_new";
  QString outputFilename = m_Controls.drrNameLineEdit->text();
  auto node = GetDataStorage()->GetNamedNode(outputFilename.toLocal8Bit().data());
  auto newnode = mitk::DataNode::New();
  // in case the output name already exists
  if (node == nullptr)
  {
    newnode->SetName(outputFilename.toLocal8Bit().data());
  }
  else
  {
    newnode->SetName(outputFilename.append(renameSuffix).toLocal8Bit().data());
    m_Controls.drrNameLineEdit->setText(outputFilename);
  }
  // add new node
  newnode->SetData(drrFilter->GetOutput());
  GetDataStorage()->Add(newnode);
  //-----------Above: add the datanode containing the DRR--------------

  //-----------------Below: generate a image on the virtual monitor screen------------------
  QString pseudoImageSuffix = "_visual";
  outputFilename = m_Controls.drrNameLineEdit->text();
  auto visualnewnode = mitk::DataNode::New();
  // in case the output name already exists
  visualnewnode->SetName(outputFilename.append(pseudoImageSuffix).toLocal8Bit().data());
  // add new node
  auto pseudoImage = drrFilter->GetOutput()->Clone();
  mitk::Point3D pseudoImageOrigin;
  pseudoImageOrigin[0] = 300 - sx * (dx - 1) / 2;
  pseudoImageOrigin[1] = 300 - sy * (dy - 1) / 2;
  pseudoImageOrigin[2] = 1;
  pseudoImage->GetGeometry()->SetOrigin(pseudoImageOrigin);
  visualnewnode->SetData(pseudoImage);
  GetDataStorage()->Add(visualnewnode);
}

void NodeEditor::VisualizeDrrProjectionModel()
{
  //-------------Below: Visualize the real CT volume in MITK scene coordinate system -----------
  auto image = dynamic_cast<mitk::Image *>(m_NewDrrCtImageDataNode->GetData())->Clone();
  auto origin = image->GetGeometry()->GetOrigin();

  double rz = m_Controls.ctRzLineEdit->text().toDouble();
  double ry = m_Controls.ctRyLineEdit->text().toDouble();
  double rx = m_Controls.ctRxLineEdit->text().toDouble();
  double tz = m_Controls.ctTzLineEdit->text().toDouble();
  double ty = m_Controls.ctTyLineEdit->text().toDouble();
  double tx = m_Controls.ctTxLineEdit->text().toDouble();

  double translate2MITKorigin[3] = {-origin[0], -origin[1], -origin[2]};
  TranslateImage(translate2MITKorigin, image);
  double center[3] = {0, 0, 0};
  double z_axis[3]{0, 0, 1};
  RotateImage(center, z_axis, rz, image);
  double y_axis[3]{0, 1, 0};
  RotateImage(center, y_axis, ry, image);
  double x_axis[3]{1, 0, 0};
  RotateImage(center, x_axis, rx, image);
  double MITKorigin2CT[3] = {tx, ty, tz};
  TranslateImage(MITKorigin2CT, image);

  QString renameSuffix = "_CT_visual";
  QString outputFilename = m_Controls.drrNameLineEdit->text();
  auto newnode = mitk::DataNode::New();
  newnode->SetName(outputFilename.append(renameSuffix).toLocal8Bit().data());
  // add new node
  newnode->SetData(image);
  GetDataStorage()->Add(newnode);
  //-------------Above: Visualize the real CT volume in MITK scene coordinate system -----------

  //------------Below: Visualize the ray source---------------
  double source_x = m_Controls.sourceXLineEdit->text().toDouble();
  double source_y = m_Controls.sourceYLineEdit->text().toDouble();
  double source_z = m_Controls.sourceZLineEdit->text().toDouble();
  auto transMitk2Imager = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixMitk2Imager = vtkSmartPointer<vtkMatrix4x4>::New();
  transMitk2Imager->Identity();
  transMitk2Imager->PostMultiply();
  transMitk2Imager->RotateZ(m_Controls.imagerRzLineEdit->text().toDouble());
  transMitk2Imager->RotateY(m_Controls.imagerRyLineEdit->text().toDouble());
  transMitk2Imager->RotateX(m_Controls.imagerRxLineEdit->text().toDouble());
  double translationMitk2Imager[3] = {m_Controls.imagerTxLineEdit->text().toDouble(),
                                      m_Controls.imagerTyLineEdit->text().toDouble(),
                                      m_Controls.imagerTzLineEdit->text().toDouble()};
  transMitk2Imager->Translate(translationMitk2Imager);
  transMitk2Imager->GetMatrix(matrixMitk2Imager);
  Eigen::Matrix4d eigenMatrixMitk2Imager{matrixMitk2Imager->GetData()};
  eigenMatrixMitk2Imager.transposeInPlace();
  Eigen::Vector4d sourcePointUnderImager{source_x, source_y, source_z, 1};
  Eigen::Vector4d sourcePointUnderMitk = eigenMatrixMitk2Imager * sourcePointUnderImager;
  auto raySource = vtkSmartPointer<vtkSphereSource>::New();
  raySource->SetCenter(sourcePointUnderMitk[0], sourcePointUnderMitk[1], sourcePointUnderMitk[2]);
  raySource->SetRadius(17);
  raySource->Update();

  auto raySourceNode = mitk::DataNode::New();
  auto raySourceSurface = mitk::Surface::New();
  raySourceSurface->SetVtkPolyData(raySource->GetOutput());
  raySourceNode->SetData(raySourceSurface);
  outputFilename = m_Controls.drrNameLineEdit->text();
  raySourceNode->SetName(outputFilename.append("_raySource_visual").toLocal8Bit().data());
  raySourceNode->SetColor(1.0, 0.0, 0.0);
  raySourceNode->SetVisibility(true);
  raySourceNode->SetOpacity(0.7);
  GetDataStorage()->Add(raySourceNode);
  //------------Above: Visualize the ray source---------------

  //-----------Below: Visualize the imager plane----------
  int dx = (m_Controls.imagerPixelNumXLineEdit->text()).toInt();
  int dy = (m_Controls.imagerPixelNumYLineEdit->text()).toInt();
  double sx = (m_Controls.imagerPixelSizeXLineEdit->text()).toDouble();
  double sy = (m_Controls.imagerPixelSizeYLineEdit->text()).toDouble();
  Eigen::Vector4d imagerOriginUnderImager{0, 0, 0, 1};
  Eigen::Vector4d imagerP1UnderImager{sx * double(dx - 1), 0, 0, 1};
  Eigen::Vector4d imagerP2UnderImager{0, sy * double(dy - 1), 0, 1};

  Eigen::Vector4d imagerOriginUnderMitk = eigenMatrixMitk2Imager * imagerOriginUnderImager;
  Eigen::Vector4d imagerP1UnderMitk = eigenMatrixMitk2Imager * imagerP1UnderImager;
  Eigen::Vector4d imagerP2UnderMitk = eigenMatrixMitk2Imager * imagerP2UnderImager;

  auto imagerPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
  imagerPlaneSource->SetOrigin(imagerOriginUnderMitk[0], imagerOriginUnderMitk[1], imagerOriginUnderMitk[2]);
  imagerPlaneSource->SetPoint1(imagerP1UnderMitk[0], imagerP1UnderMitk[1], imagerP1UnderMitk[2]);
  imagerPlaneSource->SetPoint2(imagerP2UnderMitk[0], imagerP2UnderMitk[1], imagerP2UnderMitk[2]);
  imagerPlaneSource->Update();

  auto imagerPlaneNode = mitk::DataNode::New();
  auto imagerPlaneSurface = mitk::Surface::New();
  imagerPlaneSurface->SetVtkPolyData(imagerPlaneSource->GetOutput());
  imagerPlaneNode->SetData(imagerPlaneSurface);
  outputFilename = m_Controls.drrNameLineEdit->text();
  imagerPlaneNode->SetName(outputFilename.append("_imagerPlane_visual").toLocal8Bit().data());
  imagerPlaneNode->SetColor(0.0, 0.0, 1);
  imagerPlaneNode->SetVisibility(true);
  imagerPlaneNode->SetOpacity(0.5);
  GetDataStorage()->Add(imagerPlaneNode);
  //-----------Above: Visualize the imager plane----------
}

void NodeEditor::TranslateImage(double direction[3], mitk::Image *mitkImage)
{
  if (mitkImage != nullptr)
  {
    mitk::Point3D translationDir{direction};
    auto *pointOperation = new mitk::PointOperation(mitk::OpMOVE, 0, translationDir, 0);
    // execute the Operation
    // here no undo is stored, because the movement-steps aren't interesting.
    // only the start and the end is of interest to be stored for undo.
    mitkImage->GetGeometry()->ExecuteOperation(pointOperation);

    delete pointOperation;
    // updateStemCenter();

    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
  }
}

void NodeEditor::RotateImage(double center[3], double axis[3], double degree, mitk::Image *mitkImage)
{
  if (mitkImage != nullptr)
  {
    mitk::Point3D rotateCenter{center};
    mitk::Vector3D rotateAxis{axis};
    auto *rotateOperation = new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, rotateAxis, degree);
    // execute the Operation
    // here no undo is stored, because the movement-steps aren't interesting.
    // only the start and the end is of interest to be stored for undo.
    mitkImage->GetGeometry()->ExecuteOperation(rotateOperation);

    delete rotateOperation;
    // updateStemCenter();
    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
  }
}

/*=============================================================
Above is the Code for DRR
==============================================================*/

//-------------------------------- ↓  registration part  ↓---------------------------------------
void NodeEditor::Register()
{
  if (m_RegistrationCtImageDataNode == nullptr || m_InputDrrImageDataNode_1 == nullptr ||
      m_InputDrrImageDataNode_2 == nullptr)
  {
    MITK_ERROR << "Input nodes are not ready";
    return;
  }

  auto ctimage = dynamic_cast<mitk::Image *>(m_RegistrationCtImageDataNode->GetData());
  auto DRR1 = dynamic_cast<mitk::Image *>(m_InputDrrImageDataNode_1->GetData());
  auto DRR2 = dynamic_cast<mitk::Image *>(m_InputDrrImageDataNode_2->GetData());

  if (ctimage == nullptr || DRR1 == nullptr || DRR2 == nullptr)
  {
    MITK_ERROR << "Can't Run twoProjectionRegistration: Input images are empty";
    m_Controls.registerTextBrowser->append("Error: Input image node is empty");
    return;
  }
  itk::SmartPointer<TwoProjectionRegistration> registrator = TwoProjectionRegistration::New();
  registrator->SetswitchOffOptimizer(false);
  registrator->link_drr1_cast(DRR1);
  registrator->link_drr2_cast(DRR2);
  registrator->link_3d_cast(ctimage);

  double angleDRR1 = (m_Controls.angleDrr1LineEdit->text()).toDouble();
  double angleDRR2 = (m_Controls.angleDrr2LineEdit->text()).toDouble();
  double tx = (m_Controls.initialTranslationXLineEdit->text()).toDouble();
  double ty = (m_Controls.initialTranslationYLineEdit->text()).toDouble();
  double tz = (m_Controls.initialTranslationZLineEdit->text()).toDouble();
  double cx = (m_Controls.registrationIsoOffsetXLineEdit->text()).toDouble();
  double cy = (m_Controls.registrationIsoOffsetYLineEdit->text()).toDouble();
  double cz = (m_Controls.registrationIsoOffsetZLineEdit->text()).toDouble();
  double rx = (m_Controls.initialRotationXLineEdit->text()).toDouble();
  double ry = (m_Controls.initialRotationYLineEdit->text()).toDouble();
  double rz = (m_Controls.initialRotationZLineEdit->text()).toDouble();
  double threshold = (m_Controls.registrationThresholdLineEdit->text()).toDouble();
  double scd = (m_Controls.registrationSourceToIsoDistanceLineEdit->text()).toDouble();
  double sx_1 = (m_Controls.drr1ResolutionXLineEdit->text()).toDouble();
  double sy_1 = (m_Controls.drr1ResolutionYLineEdit->text()).toDouble();
  double sx_2 = (m_Controls.drr2ResolutionXLineEdit->text()).toDouble();
  double sy_2 = (m_Controls.drr2ResolutionYLineEdit->text()).toDouble();
  double o2Dx_1 = (m_Controls.drr1CentralAxisOffsetXLineEdit->text()).toDouble();
  double o2Dy_1 = (m_Controls.drr1CentralAxisOffsetYLineEdit->text()).toDouble();
  double o2Dx_2 = (m_Controls.drr2CentralAxisOffsetXLineEdit->text()).toDouble();
  double o2Dy_2 = (m_Controls.drr2CentralAxisOffsetYLineEdit->text()).toDouble();

  if (sx_1 == 0 || sy_1 || sx_2 == 0 || sy_2 == 0)
  {
    std::cout << "FLAG!" << std::endl;
  }

  registrator->SetangleDRR1(angleDRR1);
  registrator->SetangleDRR2(angleDRR2);
  registrator->Settx(tx);
  registrator->Setty(ty);
  registrator->Settz(tz);
  registrator->Setcx(cx);
  registrator->Setcy(cy);
  registrator->Setcz(cz);
  registrator->Setrx(rx);
  registrator->Setry(ry);
  registrator->Setrz(rz);
  registrator->Setthreshold(threshold);
  registrator->Setscd(scd);
  registrator->Setsx_1(sx_1);
  registrator->Setsy_1(sy_1);
  registrator->Setsx_2(sx_2);
  registrator->Setsy_2(sy_2);
  registrator->Seto2Dx_1(o2Dx_1);
  registrator->Seto2Dy_1(o2Dy_1);
  registrator->Seto2Dx_2(o2Dx_2);
  registrator->Seto2Dy_2(o2Dy_2);

  registrator->twoprojection_registration();

  m_Controls.registrationRotationXLineEdit->setText(QString::number(registrator->GetRX()));
  m_Controls.registrationRotationYLineEdit->setText(QString::number(registrator->GetRY()));
  m_Controls.registrationRotationZLineEdit->setText(QString::number(registrator->GetRZ()));

  m_Controls.registrationTranslationXLineEdit->setText(QString::number(registrator->GetTX()));
  m_Controls.registrationTranslationYLineEdit->setText(QString::number(registrator->GetTY()));
  m_Controls.registrationTranslationZLineEdit->setText(QString::number(registrator->GetTZ()));
  m_Controls.registerTextBrowser->append("The metric is:");
  m_Controls.registerTextBrowser->append(QString::number(registrator->Getmetric()));
  m_Controls.registerTextBrowser->append("(The closer to -1 the better)");

  // add a node containing the registration result
  mitk::Point3D c_v = m_RegistrationCtImageDataNode->GetData()->GetGeometry()->GetCenter();
  double isocw[3]{c_v[0] + cx, c_v[1] + cy, c_v[2] + cz};

  itk::Image<short, 3>::Pointer m_movedCTimage;
  mitk::Image::Pointer image_tmp;
  mitk::CastToItkImage(ctimage, m_movedCTimage);
  mitk::CastToMitkImage(m_movedCTimage, image_tmp);
  double Z_axis[3]{0, 0, 1};
  RotateImage(isocw, Z_axis, registrator->GetRZ(), image_tmp);
  double Y_axis[3]{0, 1, 0};
  RotateImage(isocw, Y_axis, registrator->GetRY(), image_tmp);
  double X_axis[3]{1, 0, 0};
  RotateImage(isocw, X_axis, registrator->GetRX(), image_tmp);
  double p_tmp[3]{registrator->GetTX(), registrator->GetTY(), registrator->GetTZ()};
  TranslateImage(p_tmp, image_tmp);

  // QString outputFilename = m_Controls.drrOutputFilenameLineEdit->text();
  auto movedCT_node = mitk::DataNode::New();
  // QString movedCT_Suffix = "_register";
  // movedCT_node->SetName(outputFilename.append(movedCT_Suffix).toLocal8Bit().data());
  movedCT_node->SetName("Registered image");
  movedCT_node->SetData(image_tmp);
  GetDataStorage()->Add(movedCT_node);
}

void NodeEditor::NewRegister()
{
  if (m_NewRegistrationCtDataNode == nullptr || m_RegDrr1DataNode == nullptr || m_RegDrr2DataNode == nullptr)
  {
    MITK_ERROR << "Input nodes are not ready";
    return;
  }

  auto ctimage = dynamic_cast<mitk::Image *>(m_NewRegistrationCtDataNode->GetData());
  auto DRR1 = dynamic_cast<mitk::Image *>(m_RegDrr1DataNode->GetData());
  auto DRR2 = dynamic_cast<mitk::Image *>(m_RegDrr2DataNode->GetData());

  if (ctimage == nullptr || DRR1 == nullptr || DRR2 == nullptr)
  {
    MITK_ERROR << "Can't Run twoProjectionRegistration: Input images are empty";
    m_Controls.newRegTextBrowser->append("Error: Input image node is empty");
    return;
  }
  itk::SmartPointer<VolumeRegistrator> registrator = VolumeRegistrator::New();

  registrator->link_drr1_cast(DRR1);
  registrator->link_drr2_cast(DRR2);
  registrator->link_3d_cast(ctimage);

  double threshold = (m_Controls.regThresholdLineEdit->text()).toDouble();
  double sx_1 = (m_Controls.dr1SpacingXLineEdit->text()).toDouble();
  double sy_1 = (m_Controls.dr1SpacingYLineEdit->text()).toDouble();
  double sx_2 = (m_Controls.dr2SpacingXLineEdit->text()).toDouble();
  double sy_2 = (m_Controls.dr2SpacingYLineEdit->text()).toDouble();

  // Axes transform from "MITK frame" to "CT image frame" (world to Ct)
  auto transMitk2Ct = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixMitk2Ct = vtkSmartPointer<vtkMatrix4x4>::New();
  // auto transCt2Mitk = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixCt2Mitk = vtkSmartPointer<vtkMatrix4x4>::New();
  transMitk2Ct->Identity();
  transMitk2Ct->PostMultiply();
  transMitk2Ct->RotateZ(m_Controls.regCtRzLineEdit->text().toDouble());
  transMitk2Ct->RotateY(m_Controls.regCtRyLineEdit->text().toDouble());
  transMitk2Ct->RotateX(m_Controls.regCtRxLineEdit->text().toDouble());
  double translationMitk2Ct[3] = {m_Controls.regCtTxLineEdit->text().toDouble(),
                                  m_Controls.regCtTyLineEdit->text().toDouble(),
                                  m_Controls.regCtTzLineEdit->text().toDouble()};
  transMitk2Ct->Translate(translationMitk2Ct);
  transMitk2Ct->Update();
  transMitk2Ct->GetMatrix(matrixMitk2Ct);

  // Axes transform from "MITK frame" to "imager1 frame" (world to imager1)
  auto transMitk2Imager1 = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixMitk2Imager1 = vtkSmartPointer<vtkMatrix4x4>::New();
  transMitk2Imager1->Identity();
  transMitk2Imager1->PostMultiply();
  transMitk2Imager1->RotateZ(m_Controls.imager1RzLineEdit->text().toDouble());
  transMitk2Imager1->RotateY(m_Controls.imager1RyLineEdit->text().toDouble());
  transMitk2Imager1->RotateX(m_Controls.imager1RxLineEdit->text().toDouble());
  double translationMitk2Imager1[3] = {m_Controls.imager1TxLineEdit->text().toDouble(),
                                       m_Controls.imager1TyLineEdit->text().toDouble(),
                                       m_Controls.imager1TzLineEdit->text().toDouble()};
  transMitk2Imager1->Translate(translationMitk2Imager1);
  transMitk2Imager1->Update();
  transMitk2Imager1->GetMatrix(matrixMitk2Imager1);

  // Axes transform from "MITK frame" to "imager1 frame" (world to imager2)
  auto transMitk2Imager2 = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixMitk2Imager2 = vtkSmartPointer<vtkMatrix4x4>::New();
  transMitk2Imager2->Identity();
  transMitk2Imager2->PostMultiply();
  transMitk2Imager2->RotateZ(m_Controls.imager2RzLineEdit->text().toDouble());
  transMitk2Imager2->RotateY(m_Controls.imager2RyLineEdit->text().toDouble());
  transMitk2Imager2->RotateX(m_Controls.imager2RxLineEdit->text().toDouble());
  double translationMitk2Imager2[3] = {m_Controls.imager2TxLineEdit->text().toDouble(),
                                       m_Controls.imager2TyLineEdit->text().toDouble(),
                                       m_Controls.imager2TzLineEdit->text().toDouble()};
  transMitk2Imager2->Translate(translationMitk2Imager2);
  transMitk2Imager2->Update();
  transMitk2Imager2->GetMatrix(matrixMitk2Imager2);

  // obtain the 2 raySources
  double arraySource1[3]{m_Controls.source1XLineEdit->text().toDouble(),
                         m_Controls.source1YLineEdit->text().toDouble(),
                         m_Controls.source1ZLineEdit->text().toDouble()};
  double arraySource2[3]{m_Controls.source2XLineEdit->text().toDouble(),
                         m_Controls.source2YLineEdit->text().toDouble(),
                         m_Controls.source2ZLineEdit->text().toDouble()};

  registrator->SetArrayMatrixWorldToCt(matrixMitk2Ct->GetData());
  registrator->SetArrayMatrixWorldToImager1(matrixMitk2Imager1->GetData());
  registrator->SetArrayMatrixWorldToImager2(matrixMitk2Imager2->GetData());
  registrator->Setthreshold(threshold);
  registrator->SetRaySource1(arraySource1);
  registrator->SetRaySource2(arraySource2);
  registrator->Setsx_1(sx_1);
  registrator->Setsy_1(sy_1);
  registrator->Setsx_2(sx_2);
  registrator->Setsy_2(sy_2);

  registrator->SetswitchOffOptimizer(false);

  registrator->registration();

  m_Controls.regResultCtTxLineEdit->setText(QString::number(registrator->GetTX()));
  m_Controls.regResultCtTyLineEdit->setText(QString::number(registrator->GetTY()));
  m_Controls.regResultCtTzLineEdit->setText(QString::number(registrator->GetTZ()));

  m_Controls.regResultCtRxLineEdit->setText(QString::number(registrator->GetRX()));
  m_Controls.regResultCtRyLineEdit->setText(QString::number(registrator->GetRY()));
  m_Controls.regResultCtRzLineEdit->setText(QString::number(registrator->GetRZ()));
  m_Controls.newRegTextBrowser->append("The metric is:");
  m_Controls.newRegTextBrowser->append(QString::number(registrator->Getmetric()));
  m_Controls.newRegTextBrowser->append("(The closer to -1 the better)");
}

void NodeEditor::Visualize2ProjectionModel()
{
  //-------------Below: Visualize the real CT volume in MITK scene coordinate system -----------
  auto image = dynamic_cast<mitk::Image *>(m_NewDrrCtImageDataNode->GetData())->Clone();
  auto origin = image->GetGeometry()->GetOrigin();

  double rz = m_Controls.regResultCtRzLineEdit->text().toDouble();
  double ry = m_Controls.regResultCtRyLineEdit->text().toDouble();
  double rx = m_Controls.regResultCtRxLineEdit->text().toDouble();
  double tz = m_Controls.regResultCtTzLineEdit->text().toDouble();
  double ty = m_Controls.regResultCtTyLineEdit->text().toDouble();
  double tx = m_Controls.regResultCtTxLineEdit->text().toDouble();

  double translate2MITKorigin[3] = {-origin[0], -origin[1], -origin[2]};
  TranslateImage(translate2MITKorigin, image);
  double center[3] = {0, 0, 0};
  double z_axis[3]{0, 0, 1};
  RotateImage(center, z_axis, rz, image);
  double y_axis[3]{0, 1, 0};
  RotateImage(center, y_axis, ry, image);
  double x_axis[3]{1, 0, 0};
  RotateImage(center, x_axis, rx, image);
  double MITKorigin2CT[3] = {tx, ty, tz};
  TranslateImage(MITKorigin2CT, image);

  QString renameSuffix = "_CT_visual";
  QString outputFilename = "TwoProjection";
  auto newnode = mitk::DataNode::New();
  newnode->SetName(outputFilename.append(renameSuffix).toLocal8Bit().data());
  // add new node
  newnode->SetData(image);
  GetDataStorage()->Add(newnode);
  //-------------Above: Visualize the real CT volume in MITK scene coordinate system -----------

  //------------Below: Visualize the ray source1---------------
  double source_x1 = m_Controls.source1XLineEdit->text().toDouble();
  double source_y1 = m_Controls.source1YLineEdit->text().toDouble();
  double source_z1 = m_Controls.source1ZLineEdit->text().toDouble();
  auto transMitk2Imager1 = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixMitk2Imager1 = vtkSmartPointer<vtkMatrix4x4>::New();
  transMitk2Imager1->Identity();
  transMitk2Imager1->PostMultiply();
  transMitk2Imager1->RotateZ(m_Controls.imager1RzLineEdit->text().toDouble());
  transMitk2Imager1->RotateY(m_Controls.imager1RyLineEdit->text().toDouble());
  transMitk2Imager1->RotateX(m_Controls.imager1RxLineEdit->text().toDouble());
  double translationMitk2Imager1[3] = {m_Controls.imager1TxLineEdit->text().toDouble(),
                                       m_Controls.imager1TyLineEdit->text().toDouble(),
                                       m_Controls.imager1TzLineEdit->text().toDouble()};
  transMitk2Imager1->Translate(translationMitk2Imager1);
  transMitk2Imager1->GetMatrix(matrixMitk2Imager1);
  Eigen::Matrix4d eigenMatrixMitk2Imager1{matrixMitk2Imager1->GetData()};
  eigenMatrixMitk2Imager1.transposeInPlace();
  Eigen::Vector4d sourcePointUnderImager1{source_x1, source_y1, source_z1, 1};
  Eigen::Vector4d sourcePointUnderMitk1 = eigenMatrixMitk2Imager1 * sourcePointUnderImager1;
  auto raySource1 = vtkSmartPointer<vtkSphereSource>::New();
  raySource1->SetCenter(sourcePointUnderMitk1[0], sourcePointUnderMitk1[1], sourcePointUnderMitk1[2]);
  raySource1->SetRadius(17);
  raySource1->Update();

  auto raySourceNode1 = mitk::DataNode::New();
  auto raySourceSurface1 = mitk::Surface::New();
  raySourceSurface1->SetVtkPolyData(raySource1->GetOutput());
  raySourceNode1->SetData(raySourceSurface1);

  outputFilename = "TwoProjection";
  raySourceNode1->SetName(outputFilename.append("_raySource1_visual").toLocal8Bit().data());
  raySourceNode1->SetColor(1.0, 0.0, 0.0);
  raySourceNode1->SetVisibility(true);
  raySourceNode1->SetOpacity(0.7);
  GetDataStorage()->Add(raySourceNode1);
  //------------Above: Visualize the ray source1---------------

  //------------Below: Visualize the ray source2---------------
  double source_x2 = m_Controls.source2XLineEdit->text().toDouble();
  double source_y2 = m_Controls.source2YLineEdit->text().toDouble();
  double source_z2 = m_Controls.source2ZLineEdit->text().toDouble();
  auto transMitk2Imager2 = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> matrixMitk2Imager2 = vtkSmartPointer<vtkMatrix4x4>::New();
  transMitk2Imager2->Identity();
  transMitk2Imager2->PostMultiply();
  transMitk2Imager2->RotateZ(m_Controls.imager2RzLineEdit->text().toDouble());
  transMitk2Imager2->RotateY(m_Controls.imager2RyLineEdit->text().toDouble());
  transMitk2Imager2->RotateX(m_Controls.imager2RxLineEdit->text().toDouble());
  double translationMitk2Imager2[3] = {m_Controls.imager2TxLineEdit->text().toDouble(),
                                       m_Controls.imager2TyLineEdit->text().toDouble(),
                                       m_Controls.imager2TzLineEdit->text().toDouble()};
  transMitk2Imager2->Translate(translationMitk2Imager2);
  transMitk2Imager2->GetMatrix(matrixMitk2Imager2);
  Eigen::Matrix4d eigenMatrixMitk2Imager2{matrixMitk2Imager2->GetData()};
  eigenMatrixMitk2Imager2.transposeInPlace();
  Eigen::Vector4d sourcePointUnderImager2{source_x2, source_y2, source_z2, 1};
  Eigen::Vector4d sourcePointUnderMitk2 = eigenMatrixMitk2Imager2 * sourcePointUnderImager2;
  auto raySource2 = vtkSmartPointer<vtkSphereSource>::New();
  raySource2->SetCenter(sourcePointUnderMitk2[0], sourcePointUnderMitk2[1], sourcePointUnderMitk2[2]);
  raySource2->SetRadius(17);
  raySource2->Update();

  auto raySourceNode2 = mitk::DataNode::New();
  auto raySourceSurface2 = mitk::Surface::New();
  raySourceSurface2->SetVtkPolyData(raySource2->GetOutput());
  raySourceNode2->SetData(raySourceSurface2);

  outputFilename = "TwoProjection";
  raySourceNode2->SetName(outputFilename.append("_raySource2_visual").toLocal8Bit().data());
  raySourceNode2->SetColor(0.0, 0.0, 1);
  raySourceNode2->SetVisibility(true);
  raySourceNode2->SetOpacity(0.7);
  GetDataStorage()->Add(raySourceNode2);
  //------------Above: Visualize the ray source2---------------

  //-----------Below: Visualize the imager planes----------
  int dx = 512;
  int dy = 512;
  double sx1 = (m_Controls.dr2SpacingXLineEdit->text()).toDouble();
  double sy1 = (m_Controls.dr2SpacingYLineEdit->text()).toDouble();
  Eigen::Vector4d imagerOriginUnderImager{0, 0, 0, 1};
  Eigen::Vector4d imagerP1UnderImager{sx1 * double(dx - 1), 0, 0, 1};
  Eigen::Vector4d imagerP2UnderImager{0, sy1 * double(dy - 1), 0, 1};

  Eigen::Vector4d imagerOriginUnderMitk1 = eigenMatrixMitk2Imager1 * imagerOriginUnderImager;
  Eigen::Vector4d imagerP1UnderMitk1 = eigenMatrixMitk2Imager1 * imagerP1UnderImager;
  Eigen::Vector4d imagerP2UnderMitk1 = eigenMatrixMitk2Imager1 * imagerP2UnderImager;

  auto imagerPlaneSource1 = vtkSmartPointer<vtkPlaneSource>::New();
  imagerPlaneSource1->SetOrigin(imagerOriginUnderMitk1[0], imagerOriginUnderMitk1[1], imagerOriginUnderMitk1[2]);
  imagerPlaneSource1->SetPoint1(imagerP1UnderMitk1[0], imagerP1UnderMitk1[1], imagerP1UnderMitk1[2]);
  imagerPlaneSource1->SetPoint2(imagerP2UnderMitk1[0], imagerP2UnderMitk1[1], imagerP2UnderMitk1[2]);
  imagerPlaneSource1->Update();

  auto imagerPlaneNode1 = mitk::DataNode::New();
  auto imagerPlaneSurface1 = mitk::Surface::New();
  imagerPlaneSurface1->SetVtkPolyData(imagerPlaneSource1->GetOutput());
  imagerPlaneNode1->SetData(imagerPlaneSurface1);
  outputFilename = "TwoProjection";
  imagerPlaneNode1->SetName(outputFilename.append("_imagerPlane1_visual").toLocal8Bit().data());
  imagerPlaneNode1->SetColor(1.0, 0.0, 0.0);
  imagerPlaneNode1->SetVisibility(true);
  imagerPlaneNode1->SetOpacity(0.5);
  GetDataStorage()->Add(imagerPlaneNode1);

  Eigen::Vector4d imagerOriginUnderMitk2 = eigenMatrixMitk2Imager2 * imagerOriginUnderImager;
  Eigen::Vector4d imagerP1UnderMitk2 = eigenMatrixMitk2Imager2 * imagerP1UnderImager;
  Eigen::Vector4d imagerP2UnderMitk2 = eigenMatrixMitk2Imager2 * imagerP2UnderImager;

  auto imagerPlaneSource2 = vtkSmartPointer<vtkPlaneSource>::New();
  imagerPlaneSource2->SetOrigin(imagerOriginUnderMitk2[0], imagerOriginUnderMitk2[1], imagerOriginUnderMitk2[2]);
  imagerPlaneSource2->SetPoint1(imagerP1UnderMitk2[0], imagerP1UnderMitk2[1], imagerP1UnderMitk2[2]);
  imagerPlaneSource2->SetPoint2(imagerP2UnderMitk2[0], imagerP2UnderMitk2[1], imagerP2UnderMitk2[2]);
  imagerPlaneSource2->Update();

  auto imagerPlaneNode2 = mitk::DataNode::New();
  auto imagerPlaneSurface2 = mitk::Surface::New();
  imagerPlaneSurface2->SetVtkPolyData(imagerPlaneSource2->GetOutput());
  imagerPlaneNode2->SetData(imagerPlaneSurface2);
  outputFilename = "TwoProjection";
  imagerPlaneNode2->SetName(outputFilename.append("_imagerPlane2_visual").toLocal8Bit().data());
  imagerPlaneNode2->SetColor(0.0, 0.0, 1.0);
  imagerPlaneNode2->SetVisibility(true);
  imagerPlaneNode2->SetOpacity(0.5);
  GetDataStorage()->Add(imagerPlaneNode2);
  //-----------Above: Visualize the imager planes----------
}

// --------------------Below: test the line intersection function -------------------------
void NodeEditor::GetIntersection()
{
  unsigned int lineNumber = 3;
  double line0StartX = m_Controls.line0StartXLineEdit->text().toDouble();
  double line0StartY = m_Controls.line0StartYLineEdit->text().toDouble();
  double line0StartZ = m_Controls.line0StartZLineEdit->text().toDouble();

  double line0EndX = m_Controls.line0EndXLineEdit->text().toDouble();
  double line0EndY = m_Controls.line0EndYLineEdit->text().toDouble();
  double line0EndZ = m_Controls.line0EndZLineEdit->text().toDouble();

  double line1StartX = m_Controls.line1StartXLineEdit->text().toDouble();
  double line1StartY = m_Controls.line1StartYLineEdit->text().toDouble();
  double line1StartZ = m_Controls.line1StartZLineEdit->text().toDouble();

  double line1EndX = m_Controls.line1EndXLineEdit->text().toDouble();
  double line1EndY = m_Controls.line1EndYLineEdit->text().toDouble();
  double line1EndZ = m_Controls.line1EndZLineEdit->text().toDouble();

  double line2StartX = m_Controls.line2StartXLineEdit->text().toDouble();
  double line2StartY = m_Controls.line2StartYLineEdit->text().toDouble();
  double line2StartZ = m_Controls.line2StartZLineEdit->text().toDouble();

  double line2EndX = m_Controls.line2EndXLineEdit->text().toDouble();
  double line2EndY = m_Controls.line2EndYLineEdit->text().toDouble();
  double line2EndZ = m_Controls.line2EndZLineEdit->text().toDouble();

  Eigen::VectorXd d(9);
  d << line0StartX, line0StartY, line0StartZ, line1StartX, line1StartY, line1StartZ, line2StartX, line2StartY,
    line2StartZ;
  Eigen::VectorXd d_End(9);
  d_End << line0EndX, line0EndY, line0EndZ, line1EndX, line1EndY, line1EndZ, line2EndX, line2EndY, line2EndZ;
  Eigen::VectorXd d_Substraction(9);
  d_Substraction = d_End - d;

  Eigen::MatrixXd G(3 * lineNumber, lineNumber + 3);
  // G.Zero();

  for (int i = 0; i < 3 * lineNumber; i = i + 1)
  {
    for (int j = 0; j < 3 + lineNumber; j = j + 1)
    {
      G(i, j) = 0;

      if (i % 3 == 0 && j == 0)
      {
        G(i, j) = 1;
      }
      if (i % 3 == 1 && j == 1)
      {
        G(i, j) = 1;
      }
      if (i % 3 == 2 && j == 2)
      {
        G(i, j) = 1;
      }

      if ( j - 2 > 0 )
      {
        for (int q = 0; q < 3; q = q + 1)
        {
          G(q + 3 * (j - 3), j) = - d_Substraction[q + 3 * (j - 3)];
        }
      }

    }
  }

  Eigen::VectorXd m(3 + lineNumber);

  Eigen::MatrixXd G_Transpose(3 * lineNumber, lineNumber + 3);
  G_Transpose = G.transpose();

  m = (G_Transpose * G).inverse() * G_Transpose * d;

  m_Controls.lineIntersectionTextBrowser->append(QString::number(m[0]));
  m_Controls.lineIntersectionTextBrowser->append(QString::number(m[1]));
  m_Controls.lineIntersectionTextBrowser->append(QString::number(m[2]));


  // visualize the scene
  // Line 0
  double startLine0[3]{line0StartX, line0StartY, line0StartZ};
  double endLine0[3]{line0EndX, line0EndY, line0EndZ};
  vtkNew<vtkPoints> points_Line0;
  points_Line0->InsertNextPoint(startLine0);
  points_Line0->InsertNextPoint(endLine0);
  vtkNew<vtkPolyLine> polyLine0;
  polyLine0->GetPointIds()->SetNumberOfIds(2);
  for (unsigned int i = 0; i < 2; i++)
  {
    polyLine0->GetPointIds()->SetId(i, i);
  }

  vtkNew<vtkCellArray> cells0;
  cells0->InsertNextCell(polyLine0);


  vtkNew<vtkPolyData> polyData0;
  polyData0->SetPoints(points_Line0);
  polyData0->SetLines(cells0);

  auto linesNode0 = mitk::DataNode::New();
  auto linesSurface0 = mitk::Surface::New();
  linesSurface0->SetVtkPolyData(polyData0);
  linesNode0->SetData(linesSurface0);

  
  linesNode0->SetName("Lines");
  linesNode0->SetColor(0.7, 0, 0.0);
  linesNode0->SetVisibility(true);
  linesNode0->SetOpacity(0.7);
  GetDataStorage()->Add(linesNode0);


  double startLine1[3]{line1StartX, line1StartY, line1StartZ};
  double endLine1[3]{line1EndX, line1EndY, line1EndZ};
  vtkNew<vtkPoints> points_Line1;
  points_Line1->InsertNextPoint(startLine1);
  points_Line1->InsertNextPoint(endLine1);
  vtkNew<vtkPolyLine> polyLine1;
  polyLine1->GetPointIds()->SetNumberOfIds(2);
  for (unsigned int i = 0; i < 2; i++)
  {
    polyLine1->GetPointIds()->SetId(i, i);
  }

  vtkNew<vtkCellArray> cells1;
  cells1->InsertNextCell(polyLine1);

  vtkNew<vtkPolyData> polyData1;
  polyData1->SetPoints(points_Line1);
  polyData1->SetLines(cells1);

  auto linesNode1 = mitk::DataNode::New();
  auto linesSurface1 = mitk::Surface::New();
  linesSurface1->SetVtkPolyData(polyData1);
  linesNode1->SetData(linesSurface1);

  linesNode1->SetName("Lines");
  linesNode1->SetColor(0.0, 0.7, 0.0);
  linesNode1->SetVisibility(true);
  linesNode1->SetOpacity(0.7);
  GetDataStorage()->Add(linesNode1);


  double startLine2[3]{line2StartX, line2StartY, line2StartZ};
  double endLine2[3]{line2EndX, line2EndY, line2EndZ};
  vtkNew<vtkPoints> points_Line2;
  points_Line2->InsertNextPoint(startLine2);
  points_Line2->InsertNextPoint(endLine2);
  vtkNew<vtkPolyLine> polyLine2;
  polyLine2->GetPointIds()->SetNumberOfIds(2);
  for (unsigned int i = 0; i < 2; i++)
  {
    polyLine2->GetPointIds()->SetId(i, i);
  }

  vtkNew<vtkCellArray> cells2;
  cells2->InsertNextCell(polyLine2);

  vtkNew<vtkPolyData> polyData2;
  polyData2->SetPoints(points_Line2);
  polyData2->SetLines(cells2);

  auto linesNode2 = mitk::DataNode::New();
  auto linesSurface2 = mitk::Surface::New();
  linesSurface2->SetVtkPolyData(polyData2);
  linesNode2->SetData(linesSurface2);

  linesNode2->SetName("Lines");
  linesNode2->SetColor(0.0, 0, 0.7);
  linesNode2->SetVisibility(true);
  linesNode2->SetOpacity(0.7);
  GetDataStorage()->Add(linesNode2);

  auto raySource2 = vtkSmartPointer<vtkSphereSource>::New();
  raySource2->SetCenter(m[0], m[1], m[2]);
  raySource2->SetRadius(7);
  raySource2->Update();

  auto raySourceNode2 = mitk::DataNode::New();
  auto raySourceSurface2 = mitk::Surface::New();
  raySourceSurface2->SetVtkPolyData(raySource2->GetOutput());
  raySourceNode2->SetData(raySourceSurface2);

  
  raySourceNode2->SetName("Intersection");
  raySourceNode2->SetColor(0.5, 0.0, 0.5);
  raySourceNode2->SetVisibility(true);
  raySourceNode2->SetOpacity(0.7);
  GetDataStorage()->Add(raySourceNode2);
}
// --------------------Above: test the line intersection function -------------------------

void NodeEditor::InitialMetric()
{
  if (m_RegistrationCtImageDataNode == nullptr || m_InputDrrImageDataNode_1 == nullptr ||
      m_InputDrrImageDataNode_2 == nullptr)
  {
    MITK_ERROR << "Input nodes are not ready";
    return;
  }

  auto ctimage = dynamic_cast<mitk::Image *>(m_RegistrationCtImageDataNode->GetData());
  auto DRR1 = dynamic_cast<mitk::Image *>(m_InputDrrImageDataNode_1->GetData());
  auto DRR2 = dynamic_cast<mitk::Image *>(m_InputDrrImageDataNode_2->GetData());

  if (ctimage == nullptr || DRR1 == nullptr || DRR2 == nullptr)
  {
    MITK_ERROR << "Can't Run twoProjectionRegistration: Input images are empty";
    m_Controls.registerTextBrowser->append("Error: Input image node is empty");
    return;
  }
  itk::SmartPointer<TwoProjectionRegistration> metricCalculator = TwoProjectionRegistration::New();
  metricCalculator->SetswitchOffOptimizer(true);
  metricCalculator->link_drr1_cast(DRR1);
  metricCalculator->link_drr2_cast(DRR2);
  metricCalculator->link_3d_cast(ctimage);

  double angleDRR1 = (m_Controls.angleDrr1LineEdit->text()).toDouble();
  double angleDRR2 = (m_Controls.angleDrr2LineEdit->text()).toDouble();
  double tx = (m_Controls.initialTranslationXLineEdit->text()).toDouble();
  double ty = (m_Controls.initialTranslationYLineEdit->text()).toDouble();
  double tz = (m_Controls.initialTranslationZLineEdit->text()).toDouble();
  double cx = (m_Controls.registrationIsoOffsetXLineEdit->text()).toDouble();
  double cy = (m_Controls.registrationIsoOffsetYLineEdit->text()).toDouble();
  double cz = (m_Controls.registrationIsoOffsetZLineEdit->text()).toDouble();
  double rx = (m_Controls.initialRotationXLineEdit->text()).toDouble();
  double ry = (m_Controls.initialRotationYLineEdit->text()).toDouble();
  double rz = (m_Controls.initialRotationZLineEdit->text()).toDouble();
  double threshold = (m_Controls.registrationThresholdLineEdit->text()).toDouble();
  double scd = (m_Controls.registrationSourceToIsoDistanceLineEdit->text()).toDouble();
  double sx_1 = (m_Controls.drr1ResolutionXLineEdit->text()).toDouble();
  double sy_1 = (m_Controls.drr1ResolutionYLineEdit->text()).toDouble();
  double sx_2 = (m_Controls.drr2ResolutionXLineEdit->text()).toDouble();
  double sy_2 = (m_Controls.drr2ResolutionYLineEdit->text()).toDouble();
  double o2Dx_1 = (m_Controls.drr1CentralAxisOffsetXLineEdit->text()).toDouble();
  double o2Dy_1 = (m_Controls.drr1CentralAxisOffsetYLineEdit->text()).toDouble();
  double o2Dx_2 = (m_Controls.drr2CentralAxisOffsetXLineEdit->text()).toDouble();
  double o2Dy_2 = (m_Controls.drr2CentralAxisOffsetYLineEdit->text()).toDouble();

  if (sx_1 == 0 || sy_1 || sx_2 == 0 || sy_2 == 0)
  {
    std::cout << "FLAG!" << std::endl;
  }

  metricCalculator->SetangleDRR1(angleDRR1);
  metricCalculator->SetangleDRR2(angleDRR2);
  metricCalculator->Settx(tx);
  metricCalculator->Setty(ty);
  metricCalculator->Settz(tz);
  metricCalculator->Setcx(cx);
  metricCalculator->Setcy(cy);
  metricCalculator->Setcz(cz);
  metricCalculator->Setrx(rx);
  metricCalculator->Setry(ry);
  metricCalculator->Setrz(rz);
  metricCalculator->Setthreshold(threshold);
  metricCalculator->Setscd(scd);
  metricCalculator->Setsx_1(sx_1);
  metricCalculator->Setsy_1(sy_1);
  metricCalculator->Setsx_2(sx_2);
  metricCalculator->Setsy_2(sy_2);
  metricCalculator->Seto2Dx_1(o2Dx_1);
  metricCalculator->Seto2Dy_1(o2Dy_1);
  metricCalculator->Seto2Dx_2(o2Dx_2);
  metricCalculator->Seto2Dy_2(o2Dy_2);

  metricCalculator->twoprojection_registration();

  m_Controls.registerTextBrowser->append("The metric is:");
  m_Controls.registerTextBrowser->append(QString::number(metricCalculator->Getmetric()));
  m_Controls.registerTextBrowser->append("(The closer to -1 the better)");

  // add a node containing the registration result
  mitk::Point3D c_v = m_RegistrationCtImageDataNode->GetData()->GetGeometry()->GetCenter();
  double isocw[3]{c_v[0] + cx, c_v[1] + cy, c_v[2] + cz};

  itk::Image<short, 3>::Pointer m_movedCTimage;
  mitk::Image::Pointer image_tmp;
  mitk::CastToItkImage(ctimage, m_movedCTimage);
  mitk::CastToMitkImage(m_movedCTimage, image_tmp);
  double Z_axis[3]{0, 0, 1};
  RotateImage(isocw, Z_axis, (m_Controls.initialRotationZLineEdit->text()).toDouble(), image_tmp);
  double Y_axis[3]{0, 1, 0};
  RotateImage(isocw, Y_axis, (m_Controls.initialRotationYLineEdit->text()).toDouble(), image_tmp);
  double X_axis[3]{1, 0, 0};
  RotateImage(isocw, X_axis, (m_Controls.initialRotationXLineEdit->text()).toDouble(), image_tmp);
  double p_tmp[3]{(m_Controls.initialTranslationXLineEdit->text()).toDouble(),
                  (m_Controls.initialTranslationYLineEdit->text()).toDouble(),
                  (m_Controls.initialTranslationZLineEdit->text()).toDouble()};
  TranslateImage(p_tmp, image_tmp);

  // QString outputFilename = m_Controls.drrOutputFilenameLineEdit->text();
  auto movedCT_node = mitk::DataNode::New();
  // QString movedCT_Suffix = "_register";
  // movedCT_node->SetName(outputFilename.append(movedCT_Suffix).toLocal8Bit().data());
  movedCT_node->SetName("Initial image");
  movedCT_node->SetData(image_tmp);
  GetDataStorage()->Add(movedCT_node);
}

//-------------------------------- ↑  registration part  ↑---------------------------------------

// void NodeEditor::SetUpTcpCalibrator(double toolPointA[3],
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
//   if (X.dot(x)<0)
//   {
//     X = - X;
//   }
//  
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



//-------------------------------- ↓  QT part  ↓---------------------------------------

#define PI acos(-1)
const std::string NodeEditor::VIEW_ID = "org.mitk.views.nodeeditor";

void NodeEditor::SetFocus()
{
  // m_Controls.pushButton_applyLandMark->setFocus();
}

void NodeEditor::InitPointSetSelector(QmitkSingleNodeSelectionWidget *widget)
{
  widget->SetDataStorage(GetDataStorage());
  widget->SetNodePredicate(mitk::NodePredicateAnd::New(
    mitk::TNodePredicateDataType<mitk::PointSet>::New(),
    mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
                                                           mitk::NodePredicateProperty::New("hidden object")))));

  widget->SetSelectionIsOptional(true);
  widget->SetAutoSelectNewNodes(true);
  widget->SetEmptyInfo(QString("Please select a point set"));
  widget->SetPopUpTitel(QString("Select point set"));
}

void NodeEditor::InitNodeSelector(QmitkSingleNodeSelectionWidget *widget)
{
  widget->SetDataStorage(GetDataStorage());
  widget->SetNodePredicate(mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(
    mitk::NodePredicateProperty::New("helper object"), mitk::NodePredicateProperty::New("hidden object"))));
  widget->SetSelectionIsOptional(true);
  widget->SetAutoSelectNewNodes(true);
  widget->SetEmptyInfo(QString("Please select a node"));
  widget->SetPopUpTitel(QString("Select node"));
}

void NodeEditor::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);
  // connect(m_Controls.buttonPerformImageProcessing, &QPushButton::clicked, this, &NodeEditor::DoImageProcessing);

  // Set Node Selection Widget

  InitNodeSelector(m_Controls.drrCtImageSingleNodeSelectionWidget);
  InitNodeSelector(m_Controls.newDrrCtImageSingleNodeSelectionWidget);
  InitNodeSelector(m_Controls.widget_Poly);
  InitNodeSelector(m_Controls.widget_CropImage);
  InitNodeSelector(m_Controls.registrationCtSingleNodeSelectionWidget);
  InitNodeSelector(m_Controls.registrationDrr1SingleNodeSelectionWidget);
  InitNodeSelector(m_Controls.registrationDrr2SingleNodeSelectionWidget);
  InitNodeSelector(m_Controls.rawCtImageSingleNodeSelectionWidget);
  InitNodeSelector(m_Controls.evaluationPointsSingleNodeSelectionWidget);
  InitNodeSelector(m_Controls.newRegistrationCtSingleNodeSelectionWidget);
  InitNodeSelector(m_Controls.regDrr1SingleNodeSelectionWidget);
  InitNodeSelector(m_Controls.regDrr2SingleNodeSelectionWidget);

  connect(m_Controls.newRegistrationCtSingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::NewRegistrationCtChanged);

  connect(m_Controls.regDrr1SingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::RegDrr1Changed);

  connect(m_Controls.regDrr2SingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::RegDrr2Changed);

  connect(m_Controls.getIntersectionPushButton, &QPushButton::clicked, this, &NodeEditor::GetIntersection);

  connect(m_Controls.newRegPushButton, &QPushButton::clicked, this, &NodeEditor::NewRegister);

  connect(m_Controls.newRegVisualPushButton, &QPushButton::clicked, this, &NodeEditor::Visualize2ProjectionModel);

  connect(m_Controls.rawCtImageSingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::RawCtImageChanged);
  connect(m_Controls.evaluationPointsSingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::EvaluationPointsChanged);
  connect(m_Controls.evaluateRegisterPushButton, &QPushButton::clicked, this, &NodeEditor::EvaluateRegistration);

  connect(m_Controls.v1GenerateDrrPushButton, &QPushButton::clicked, this, &NodeEditor::V1DrrGenerateData);
  connect(m_Controls.v2GenerateDrrPushButton, &QPushButton::clicked, this, &NodeEditor::V2DrrGenerateData);
  connect(m_Controls.drrModelVisualPushButton, &QPushButton::clicked, this, &NodeEditor::VisualizeDrrProjectionModel);

  // drr
  connect(m_Controls.recoverDefaultValuesPushButton, &QPushButton::clicked, this, &NodeEditor::SetUiDefault);
  connect(m_Controls.generateDrrPushButton, &QPushButton::clicked, this, &NodeEditor::Drr);
  connect(m_Controls.drrCtImageSingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::DrrCtImageChanged);
  connect(m_Controls.newDrrCtImageSingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::NewDrrCtImageChanged);

  connect(m_Controls.widget_CropImage,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::InputImageToCropChanged);

  connect(m_Controls.widget_Poly,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::InputSurfaceChanged);

  // twoProjectionRegistration
  connect(m_Controls.registerPushButton, &QPushButton::clicked, this, &NodeEditor::Register);
  connect(m_Controls.initialMetricPushButton, &QPushButton::clicked, this, &NodeEditor::InitialMetric);
  connect(m_Controls.registrationCtSingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::RegistrationCtImageChanged);
  connect(m_Controls.registrationDrr1SingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::InputDrrImageChanged_1);
  connect(m_Controls.registrationDrr2SingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::InputDrrImageChanged_2);
  // stl polydata to imagedata
  connect(m_Controls.surfaceToImagePushButton, &QPushButton::clicked, this, &NodeEditor::PolyDataToImageData);
  connect(m_Controls.generateWhiteImagePushButton, &QPushButton::clicked, this, &NodeEditor::GenerateWhiteImage);
}
//-------------------------------- ↑  QT part  ↑---------------------------------------
