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
#include "vtkPlaneSource.h"
#include "vtkSphereSource.h"
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

#include <eigen3/Eigen/Eigen>
#include <mitkPadImageFilter.h>
#include <drr.h>
#include <nodebinder.h>
#include <surfaceregistraion.h>
#include <drrsidonjacobsraytracing.h>

// registration header
#include <twoprojectionregistration.h>

/*=============================================================
Above are Headers for the Node Editor plugin
==============================================================*/

/*=============================================================
Below are Headers for DRR testing
==============================================================*/
#include "itkImage.h"
// #include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCenteredEuler3DTransform.h"
#include "itkNearestNeighborInterpolateImageFunction.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"
//---

#include "mitkImageCast.h"
#include "itkRayCastInterpolateImageFunction.h"
#include <boost/numeric/conversion/bounds.hpp>


#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>




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
  double direction[3]{surfaceCenter[0] - imageCenter[0], surfaceCenter[1] - imageCenter[1], surfaceCenter[2] - imageCenter[2]};
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
  for(int n =0; n<6;n++)
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

  //add a new node for the white image
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
  //auto image = GetDataStorage()->GetNamedObject<mitk::Image>((m_Controls.inputFilename->text()).toLocal8Bit().data());
  auto image = dynamic_cast<mitk::Image *>(m_DrrCtImageDataNode->GetData());
  //auto sliced = dynamic_cast<mitk::SlicedData *>(m_DrrCtImageDataNode->GetData());
  
 // auto image = dynamic_cast<mitk::Image *>(sliced);
  //the original input image node will be named "unnamed", and you have to rename it because it really does not have a name!!
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


  //add new node for DRR geometry visualization


  mitk::Image::Pointer image_trans = drrFilter->GetOutput();
  mitk::Point3D c_k = m_DrrCtImageDataNode->GetData()->GetGeometry()->GetCenter();

  double c_v[3]{c_k[0], c_k[1], c_k[2]};

  // rotate 90 degrees to fit the DRR geometry
  double x_axis[3]{1, 0, 0};
  double isoc[3]{0, 0, -scd};
  RotateImage(isoc, x_axis, -90, image_trans);

  // move the center of the image to the isocenter in the sample coordinates
  double p[3]{c_v[0]+cx, c_v[1]+cy  ,c_v[2]+cy + scd }; // translation vector
  // mitk::Point3D direciton{p};
  TranslateImage(p, image_trans);

  
  double isocw[3]{c_v[0] + cx, c_v[1] + cy, c_v[2] + cz };

  

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

  c_v[0] =(c_v[0] + cx)+ scd * sin(rprojection*3.1415926/180);
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
    raySourceDataNode->SetName((outputFilename+movedCT_Suffix).toLocal8Bit().data());
    raySourceDataNode->SetData(pointSet_raySource);
    GetDataStorage()->Add(raySourceDataNode);
  }
}

void NodeEditor::NewDrrGenerateData() // this method incorporates the MITK coordinate system which can be regarded as the NDI coordinate system later 
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
  
  //-------------Below: Construct the Affine transform between coordinate systems: MITK scene, CT volume, c-arm imager, c-arm internal CT volume-------------------------
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
  // transCt2InternalCt->Concatenate(matrixCt2Mitk);
  // transCt2InternalCt->Concatenate(matrixMitk2Imager);
  // transCt2InternalCt->Concatenate(matrixImager2InternalCt);
  transCt2InternalCt->Concatenate(matrixImager2InternalCt);
  transCt2InternalCt->Concatenate(matrixMitk2Imager);
  transCt2InternalCt->Concatenate(matrixCt2Mitk);
  transCt2InternalCt->Update();
  transCt2InternalCt->GetInverse(matrixInternalCt2Ct);
  //-------------Above: Construct the Affine transform between coordinate systems-------------------------

  //----------Below: Extract ZYX angles from the affine transform matrix----------
  double rx, ry, rz;
  double piParameter = 180 / 3.1415926;
  if (matrixInternalCt2Ct->GetElement(0,2) < 1)
  {
    if (matrixInternalCt2Ct->GetElement(0, 2) > -1)
    {
      ry = asin(matrixInternalCt2Ct->GetElement(0, 2));
      rx = atan2(-matrixInternalCt2Ct->GetElement(1, 2), matrixInternalCt2Ct->GetElement(2, 2));
      rz = atan2(-matrixInternalCt2Ct->GetElement(0, 1), matrixInternalCt2Ct->GetElement(0, 0));
    }else
    {
      ry = -3.1415926 / 2;
      rx = -atan2(matrixInternalCt2Ct->GetElement(1, 0), matrixInternalCt2Ct->GetElement(1, 1));
      rz = 0;
    }
  }else
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
  
  double rprojection = 0;
  double tx = matrixInternalCt2Ct->GetElement(0,3);
  double ty = matrixInternalCt2Ct->GetElement(1, 3);
  double tz = matrixInternalCt2Ct->GetElement(2, 3);

  double cx = 0;
  double cy = 0;
  double cz = 0;
  double threshold = (m_Controls.newDrrthresLineEdit->text()).toDouble();
  double scd = (m_Controls.sourceZLineEdit->text()).toDouble();


  double o2Dx = -((m_Controls.sourceXLineEdit->text()).toDouble() - sx * (dx - 1) / 2);
  double o2Dy = -((m_Controls.sourceYLineEdit->text()).toDouble() - sy * (dy - 1) / 2);
  
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
  m_Controls.newDrrTextBrowser->append("rprojection: " + QString::number(rprojection));
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
  m_Controls.newDrrTextBrowser->append("o2Dx: " + QString::number(o2Dx));
  m_Controls.newDrrTextBrowser->append("o2Dy: " + QString::number(o2Dy));
  //------------Above: Print out the real parameters used for DRR generation ----------------
  
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

  // auto tempCalibrePlaneNode = GetDataStorage()->GetNamedNode("Calibrator Plane");
  // if (tempCalibrePlaneNode != nullptr)
  // {
  //   GetDataStorage()->Remove(tempCalibrePlaneNode);
  // }

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

  if (sx_1 == 0 || sy_1 || sx_2 == 0 || sy_2==0)
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
  //connect(m_Controls.buttonPerformImageProcessing, &QPushButton::clicked, this, &NodeEditor::DoImageProcessing);

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
   
  connect(m_Controls.rawCtImageSingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::RawCtImageChanged);
  connect(m_Controls.evaluationPointsSingleNodeSelectionWidget,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &NodeEditor::EvaluationPointsChanged);
  connect(m_Controls.evaluateRegisterPushButton, &QPushButton::clicked, this, &NodeEditor::EvaluateRegistration);

  connect(m_Controls.newGenerateDrrPushButton, &QPushButton::clicked, this, &NodeEditor::NewDrrGenerateData);
  connect(m_Controls.drrModelVisualPushButton, &QPushButton::clicked, this, &NodeEditor::VisualizeDrrProjectionModel);


  //drr
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


  //twoProjectionRegistration
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
  //stl polydata to imagedata
  connect(m_Controls.surfaceToImagePushButton, &QPushButton::clicked, this, &NodeEditor::PolyDataToImageData);
  connect(m_Controls.generateWhiteImagePushButton, &QPushButton::clicked, this, &NodeEditor::GenerateWhiteImage);
  // connect(m_Controls.widget_stl,
  //         &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
  //         this,
  //         &NodeEditor::inputstldataChanged);
  

}



//-------------------------------- ↑  QT part  ↑---------------------------------------


