/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include <sstream>
// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "VCView.h"

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

#include <polish.h>
#include <vtkLinearSubdivisionFilter.h>
#define PI acos(-1)
const std::string VCView::VIEW_ID = "org.mitk.views.vcview";

void VCView::SetFocus()
{
  m_Controls.pushButton_applyPelvicVC->setFocus();
}

void VCView::InitPointSetSelector(QmitkSingleNodeSelectionWidget *widget)
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

void VCView::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);
  m_Controls.pushButton_applyPelvicVC->setEnabled(false);
  m_Controls.pushButton_applyFemurVC->setEnabled(false);
  m_Controls.pushButton_buildFemurL->setEnabled(false);
  m_Controls.pushButton_buildFemurR->setEnabled(false);
  m_Controls.pushButton_buildPelvis->setEnabled(false);

  connect(m_Controls.pushButton_CutImage, &QPushButton::clicked, this, &VCView::CutImage);
  connect(m_Controls.pushButton_AddPoint, &QPushButton::clicked, this, &VCView::AddPoint);
  connect(m_Controls.pushButton_Init, &QPushButton::clicked, this, &VCView::Initialize);
  connect(m_Controls.pushButton_autoDetect, &QPushButton::clicked, this, &VCView::AutoDetect);

  connect(m_Controls.comboBox_OpSide, &QComboBox::currentTextChanged, this, &VCView::SetOpSide);
  connect(m_Controls.pushButton_buildPelvis, &QPushButton::clicked, this, &VCView::BuildPelvis);
  connect(m_Controls.pushButton_buildFemurL, &QPushButton::clicked, this, &VCView::BuildFemurL);
  connect(m_Controls.pushButton_buildFemurR, &QPushButton::clicked, this, &VCView::BuildFemurR);
  connect(m_Controls.pushButton_applyPelvicVC, &QPushButton::clicked, this, &VCView::ApplyPelvicVC);
  connect(m_Controls.pushButton_applyFemurVC, &QPushButton::clicked, this, &VCView::ApplyFemurVC);

  connect(m_Controls.pushButton_RotateXAdd, &QPushButton::clicked, this, &VCView::RoTationPelvisXAdd);
  connect(m_Controls.pushButton_RotateXSub, &QPushButton::clicked, this, &VCView::RoTationPelvisXSub);

  void (QDoubleSpinBox::*value)(double) = &QDoubleSpinBox::valueChanged;
  connect(m_Controls.doubleSpinBox_RotateXAdd, value, this, &VCView::SetRoTationPelvisXAddValue);
  connect(m_Controls.doubleSpinBox_RotateXSub, value, this, &VCView::SetRoTationPelvisXSubValue);
  // connect(m_Controls.pushButton_AddLImage, &QPushButton::clicked, this, &VCView::cutLFemur);
  // connect(m_Controls.pushButton_AddRImage, &QPushButton::clicked, this, &VCView::cutRFemur);
  // Build COR
  connect(m_Controls.pushButton_PLC, &QPushButton::clicked, this, &VCView::OnpushButton_PLC);
  connect(m_Controls.pushButton_PRC, &QPushButton::clicked, this, &VCView::OnpushButton_PRC);
  connect(m_Controls.pushButton_FLC, &QPushButton::clicked, this, &VCView::OnpushButton_FLC);
  connect(m_Controls.pushButton_FRC, &QPushButton::clicked, this, &VCView::OnpushButton_FRC);
  connect(m_Controls.pushButton_Updata, &QPushButton::clicked, this, &VCView::OnpushButton_Updata);
  connect(
    m_Controls.pushButton_FemoralVersionAngle, &QPushButton::clicked, this, &VCView::OnpushButton_FemoralVersionAngle);
  connect(m_Controls.pushButton_pelvisAngle, &QPushButton::clicked, this, &VCView::OnpushButton_PelvisVersionAngle);

  connect(m_Controls.pushButton_enabelT, &QPushButton::clicked, this, &VCView::EnableTransform);
  connect(m_Controls.pushButton_xp, &QPushButton::clicked, this, &VCView::moveXp);
  connect(m_Controls.pushButton_xm, &QPushButton::clicked, this, &VCView::moveXm);
  connect(m_Controls.pushButton_yp, &QPushButton::clicked, this, &VCView::moveYp);
  connect(m_Controls.pushButton_ym, &QPushButton::clicked, this, &VCView::moveYm);
  connect(m_Controls.pushButton_zp, &QPushButton::clicked, this, &VCView::moveZp);
  connect(m_Controls.pushButton_zm, &QPushButton::clicked, this, &VCView::moveZm);

  connect(m_Controls.pushButton_rxp, &QPushButton::clicked, this, &VCView::rotateXp);
  connect(m_Controls.pushButton_rxm, &QPushButton::clicked, this, &VCView::rotateXm);
  connect(m_Controls.pushButton_ryp, &QPushButton::clicked, this, &VCView::rotateYp);
  connect(m_Controls.pushButton_rym, &QPushButton::clicked, this, &VCView::rotateYm);
  connect(m_Controls.pushButton_rzp, &QPushButton::clicked, this, &VCView::rotateZp);
  connect(m_Controls.pushButton_rzm, &QPushButton::clicked, this, &VCView::rotateZm);
  connect(m_Controls.comboBox_stemList,
          static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          this,
          &VCView::SelectStem);
  connect(m_Controls.pushButton_initPos, &QPushButton::clicked, this, &VCView::InitStemPos);
  connect(m_Controls.pushButton_stemCenter, &QPushButton::clicked, this, &VCView::updateStemCenter);

  connect(m_Controls.pushButton_clip, &QPushButton::clicked, this, &VCView::ClipFemur);
  connect(m_Controls.pushButton_cutLeftFemur, &QPushButton::clicked, this, &VCView::StlCutLeftImage);
  connect(m_Controls.pushButton_cutRightFemur, &QPushButton::clicked, this, &VCView::StlCutRightImage);
  connect(m_Controls.pushButton_cutImage, &QPushButton::clicked, this, &VCView::StlCutImage);
  connect(m_Controls.pushButton_overlay, &QPushButton::clicked, this, &VCView::StlCutOverlayImage);

  SetOpSide(m_Controls.comboBox_OpSide->currentText());

  m_stempara = StemParameter{1, 135, 31.8};

  // Set Node Selection Widget
  // source
  InitPointSetSelector(m_Controls.widget_SourcePset);
  // target
  InitPointSetSelector(m_Controls.widget_TargetPset);

  // moving node
  m_Controls.widget_MovingNode->SetDataStorage(GetDataStorage());
  m_Controls.widget_MovingNode->SetNodePredicate(mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(
    mitk::NodePredicateProperty::New("helper object"), mitk::NodePredicateProperty::New("hidden object"))));

  m_Controls.widget_MovingNode->SetSelectionIsOptional(true);
  m_Controls.widget_MovingNode->SetAutoSelectNewNodes(true);
  m_Controls.widget_MovingNode->SetEmptyInfo(QString("Please select a node"));
  m_Controls.widget_MovingNode->SetPopUpTitel(QString("Select node"));

  connect(m_Controls.widget_SourcePset,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &VCView::onSourcePsetChanged);
  connect(m_Controls.widget_TargetPset,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &VCView::onTargetPsetChanged);
  connect(m_Controls.widget_MovingNode,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &VCView::onMovingNodeChanged);
  connect(m_Controls.pushButton_applyTrans, &QPushButton::clicked, this, &VCView::OnPushButtonApplyTrans);

  // 5Cut
  // 1
  InitPointSetSelector(m_Controls.widget_plane_distal);
  InitPointSetSelector(m_Controls.widget_plane_backOblique);
  InitPointSetSelector(m_Controls.widget_plane_back);
  InitPointSetSelector(m_Controls.widget_plane_frontOblique);
  InitPointSetSelector(m_Controls.widget_plane_front);

  connect(m_Controls.widget_plane_distal,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &VCView::onSourcePsetChanged);
  connect(m_Controls.widget_plane_backOblique,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &VCView::onTargetPsetChanged);
  connect(m_Controls.widget_plane_back,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &VCView::onSourcePsetChanged);
  connect(m_Controls.widget_plane_frontOblique,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &VCView::onTargetPsetChanged);
  connect(m_Controls.widget_plane_front,
          &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
          this,
          &VCView::onTargetPsetChanged);
  connect(m_Controls.pushButton_padImage, &QPushButton::clicked, this, &VCView::padImage);

  //polish
  connect(m_Controls.pushButton_setupPolish, &QPushButton::clicked, this, &VCView::SetupPolish);
  connect(m_Controls.pushButton_startPolishing, &QPushButton::clicked, this, &VCView::StartPolish);
  connect(m_Controls.pushButton_stopPolishing, &QPushButton::clicked, this, &VCView::StopPolishing);
  connect(m_Controls.pushButton_polishonce, &QPushButton::clicked, this, &VCView::PolishOnce);
  connect(m_Controls.pushButton_restore, &QPushButton::clicked, this, &VCView::RestorePolish);
}

void VCView::CutImage()
{
  auto image = GetDataStorage()->GetNamedObject<mitk::Image>("srcimage");

  if (m_femurL == nullptr || image == nullptr)
  {
    MITK_INFO << "no femur L";
    return;
  }

  // left leg
  auto leftLegImage = SurfaceCutImage(m_femurL, image, false, true);
  if (leftLegImage != nullptr)
  {
    mitk::DataNode::Pointer croppedImageNodeL = mitk::DataNode::New();
    croppedImageNodeL->SetName("femurimage_left");
    croppedImageNodeL->SetData(leftLegImage);
    GetDataStorage()->Add(croppedImageNodeL);
  }
  else
    MITK_ERROR << "leftLegImage null";

  if (m_femurR == nullptr || image == nullptr)
  {
    MITK_INFO << "no femur R";
    return;
  }
  // right leg
  auto rightLegImage = SurfaceCutImage(m_femurR, image, false, true);
  if (rightLegImage != nullptr)
  {
    mitk::DataNode::Pointer croppedImageNodeR = mitk::DataNode::New();
    croppedImageNodeR->SetName("femurimage_right");
    croppedImageNodeR->SetData(rightLegImage);
    GetDataStorage()->Add(croppedImageNodeR);
  }
  else
    MITK_ERROR << "rightLegImage null";

  // image without legs
  auto imageWithoutLegL = SurfaceCutImage(m_femurL, image, true, false);
  auto imageWithoutLegs = SurfaceCutImage(m_femurR, imageWithoutLegL, true, false);
  mitk::DataNode::Pointer imageNodeWithoutLegs = mitk::DataNode::New();
  imageNodeWithoutLegs->SetName("image");
  imageNodeWithoutLegs->SetData(imageWithoutLegs);

  GetDataStorage()->Add(imageNodeWithoutLegs);

  this->StlCutOverlayImage();

}
void VCView::AddPoint()
{
  //double m_temp[3]{0};
  mitk::PointSet::Pointer _asis = GetDataStorage()->GetNamedObject<mitk::PointSet>("ASIS");
  if (_asis == nullptr)
  {
    _asis = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("ASIS");
    pNode->SetData(_asis);
    pNode->SetFloatProperty("pointsize", 8);
    pNode->SetBoolProperty("show contour", true);
    pNode->SetProperty("contourcolor", mitk::ColorProperty::New(1, 1, 0));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  mitk::PointSet::Pointer _pt = GetDataStorage()->GetNamedObject<mitk::PointSet>("PT");
  if (_pt == nullptr)
  {
    _pt = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("PT");
    pNode->SetData(_pt);
    pNode->SetFloatProperty("pointsize", 8);
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  mitk::PointSet::Pointer _trochanterL = GetDataStorage()->GetNamedObject<mitk::PointSet>("trochanter_left");
  if (_trochanterL == nullptr)
  {
    _trochanterL = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("trochanter_left");
    pNode->SetData(_trochanterL);
    pNode->SetFloatProperty("pointsize", 8);
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  mitk::PointSet::Pointer _trochanterR = GetDataStorage()->GetNamedObject<mitk::PointSet>("trochanter_right");
  if (_trochanterR == nullptr)
  {
    _trochanterR = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("trochanter_right");
    pNode->SetData(_trochanterR);
    pNode->SetFloatProperty("pointsize", 8);
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  mitk::PointSet::Pointer _femurCORL = GetDataStorage()->GetNamedObject<mitk::PointSet>("CalFemurCOR_left");
  if (_femurCORL == nullptr)
  {
    _femurCORL = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("CalFemurCOR_left");
    pNode->SetData(_femurCORL);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  mitk::PointSet::Pointer _femurCORR = GetDataStorage()->GetNamedObject<mitk::PointSet>("CalFemurCOR_right");
  if (_femurCORR == nullptr)
  {
    _femurCORR = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("CalFemurCOR_right");
    pNode->SetData(_femurCORR);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  mitk::PointSet::Pointer _pelvisCORL = GetDataStorage()->GetNamedObject<mitk::PointSet>("VerifyCOR_left");
  if (_pelvisCORL == nullptr)
  {
    _pelvisCORL = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("VerifyCOR_left");
    pNode->SetData(_pelvisCORL);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetColor(0, 0, 1);
    pNode->SetProperty("selectedcolor", mitk::ColorProperty::New(1, 1, 1));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  mitk::PointSet::Pointer _pelvisCORR = GetDataStorage()->GetNamedObject<mitk::PointSet>("VerifyCOR_right");
  if (_pelvisCORR == nullptr)
  {
    _pelvisCORR = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("VerifyCOR_right");
    pNode->SetData(_pelvisCORR);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetColor(0, 0, 1);
    pNode->SetProperty("selectedcolor", mitk::ColorProperty::New(1, 1, 1));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  mitk::PointSet::Pointer _axisL = GetDataStorage()->GetNamedObject<mitk::PointSet>("femurAxis_left");
  if (_axisL == nullptr)
  {
    _axisL = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("femurAxis_left");
    pNode->SetData(_axisL);
    pNode->SetFloatProperty("pointsize", 8);
    pNode->SetBoolProperty("show contour", true);
    pNode->SetProperty("contourcolor", mitk::ColorProperty::New(1, 1, 0));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  mitk::PointSet::Pointer _axisR = GetDataStorage()->GetNamedObject<mitk::PointSet>("femurAxis_right");
  if (_axisR == nullptr)
  {
    _axisR = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("femurAxis_right");
    pNode->SetData(_axisR);
    pNode->SetFloatProperty("pointsize", 8);
    pNode->SetBoolProperty("show contour", true);
    pNode->SetProperty("contourcolor", mitk::ColorProperty::New(1, 1, 0));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }

  mitk::PointSet::Pointer pCOR = GetDataStorage()->GetNamedObject<mitk::PointSet>("pelvisCOR");
  if (pCOR == nullptr)
  {
    pCOR = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("pelvisCOR");
    pNode->SetData(pCOR);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetColor(1, 0, 1);
    pNode->SetFloatProperty("point 2D size", 8);
    pNode->SetProperty("Pointset.2D.shape", mitk::PointSetShapeProperty::New(7));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }

  mitk::PointSet::Pointer fCORL = GetDataStorage()->GetNamedObject<mitk::PointSet>("femurCOR_left");
  if (fCORL == nullptr)
  {
    fCORL = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("femurCOR_left");
    pNode->SetData(fCORL);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetColor(1, 0, 1);
    pNode->SetFloatProperty("point 2D size", 8);
    pNode->SetProperty("Pointset.2D.shape", mitk::PointSetShapeProperty::New(7));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }

  mitk::PointSet::Pointer fCORR = GetDataStorage()->GetNamedObject<mitk::PointSet>("femurCOR_right");
  if (fCORR == nullptr)
  {
    fCORR = mitk::PointSet::New();
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("femurCOR_right");
    pNode->SetData(fCORR);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetColor(1, 0, 1);
    pNode->SetFloatProperty("point 2D size", 8);
    pNode->SetProperty("Pointset.2D.shape", mitk::PointSetShapeProperty::New(7));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
}

void VCView::Initialize()
{
  m_rotateFemurLeftAngle = 0;
  m_rotateFemurRightAngle = 0;
  m_builtFemurR = false;
  m_builtFemurL = false;
  m_builtPelvis = false;
  m_Controls.lineEdit_objname_pelvis->setText("");
  m_Controls.lineEdit_objname_asis->setText("");
  m_rotate_X = 0;
  m_rotate_Y = 0;
  m_rotate_Z = 0;
  m_rotate_XAdd = 1;
  m_rotate_XSub = 1;
  m_Controls.pushButton_applyPelvicVC->setEnabled(false);
  m_Controls.pushButton_applyFemurVC->setEnabled(false);
}

void VCView::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/, const QList<mitk::DataNode::Pointer> &nodes)
{
  std::string nodeName;
  // iterate all selected objects, adjust warning visibility
  foreach (mitk::DataNode::Pointer node, nodes)
  {
    if (node.IsNull())
    {
      return;
    }

    node->GetName(nodeName);
    mitk::Surface *p_surface = dynamic_cast<mitk::Surface *>(node->GetData());
    if (p_surface != nullptr)
    {
      m_Controls.lineEdit_surfaceName->setText(QString::fromStdString(nodeName));
      m_tmpSurface = p_surface;
    }
    // pelvis
    if (nodeName == "pelvis")
    {
      m_pelvis = dynamic_cast<mitk::Surface *>(node->GetData());
      m_Controls.lineEdit_objname_pelvis->setText(QString::fromStdString(nodeName));
    }
    if (nodeName == "ASIS")
    {
      m_asis = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_asis->setText(QString::fromStdString(nodeName));
    }
    if (nodeName == "PT")
    {
      m_pt = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_PT->setText(QString::fromStdString(nodeName));
    }
    // femur
    if (nodeName == "femur_left")
    {
      m_femurL = dynamic_cast<mitk::Surface *>(node->GetData());
      m_Controls.lineEdit_objname_femurL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[0] = m_femurL;
    }
    if (nodeName == "femurimage_left")
    {
      m_femurImageL = dynamic_cast<mitk::Image *>(node->GetData());
      m_Controls.lineEdit_objname_femurImageL->setText(QString::fromStdString(nodeName));
    }
    if (nodeName == "knee_left")
    {
      m_kneeL = dynamic_cast<mitk::Surface *>(node->GetData());
      m_Controls.lineEdit_objname_kneeL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[1] = m_kneeL;
    }
    if (nodeName == "femurAxis_left")
    {
      m_canalAxisL = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_femurAxisL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[2] = m_canalAxisL;
    }
    if (nodeName == "trochanter_left")
    {
      m_trochanterL = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_trochanterL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[3] = m_trochanterL;
    }

    if (nodeName == "femurCOR_left")
    {
      m_fhcL = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_femurCorL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[4] = m_fhcL;
    }
    if (nodeName == "Btrochanter_left")
    {
      m_BtrochanterL = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_BTrochanterL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[5] = m_BtrochanterL;
    }
    if (nodeName == "condyles_left")
    {
      m_condylesL = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_condylesL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[6] = m_condylesL;
    }
    if (nodeName == "femur_right")
    {
      m_femurR = dynamic_cast<mitk::Surface *>(node->GetData());
      m_Controls.lineEdit_objname_femurR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[0] = m_femurR;
    }
    if (nodeName == "femurimage_right")
    {
      m_femurImageR = dynamic_cast<mitk::Image *>(node->GetData());
      m_Controls.lineEdit_objname_femurImageR->setText(QString::fromStdString(nodeName));
    }
    if (nodeName == "knee_right")
    {
      m_kneeR = dynamic_cast<mitk::Surface *>(node->GetData());
      m_Controls.lineEdit_objname_kneeR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[1] = m_kneeR;
    }
    if (nodeName == "femurAxis_right")
    {
      m_canalAxisR = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_femurAxisR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[2] = m_canalAxisR;
    }
    if (nodeName == "trochanter_right")
    {
      m_trochanterR = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_trochanterR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[3] = m_trochanterR;
    }
    if (nodeName == "femurCOR_right")
    {
      m_fhcR = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_femurCorR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[4] = m_fhcR;
    }
    if (nodeName == "Btrochanter_right")
    {
      m_BtrochanterR = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_BTrochanterR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[5] = m_BtrochanterR;
    }
    if (nodeName == "condyles_right")
    {
      m_condylesR = dynamic_cast<mitk::PointSet *>(node->GetData());
      m_Controls.lineEdit_objname_condylesR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[6] = m_condylesR;
    }
  }

  if (m_Controls.lineEdit_objname_pelvis->text() != "" && m_Controls.lineEdit_objname_asis->text() != "" &&
      m_Controls.lineEdit_objname_PT->text() != "")
  {
    m_Controls.pushButton_buildPelvis->setEnabled(true);
    m_Controls.pushButton_applyPelvicVC->setEnabled(true);
  }

  if (m_Controls.lineEdit_objname_femurL->text() != "" && m_Controls.lineEdit_objname_femurAxisL->text() != "" &&
      m_Controls.lineEdit_objname_femurCorL->text() != "" && m_Controls.lineEdit_objname_trochanterL->text() != "")
  {
    m_Controls.pushButton_buildFemurL->setEnabled(true);
  }

  if (m_Controls.lineEdit_objname_femurR->text() != "" && m_Controls.lineEdit_objname_femurAxisR->text() != "" &&
      m_Controls.lineEdit_objname_femurCorR->text() != "" && m_Controls.lineEdit_objname_trochanterR->text() != "")
  {
    m_Controls.pushButton_buildFemurR->setEnabled(true);
  }
}

void VCView::AutoDetect()
{
  using VectorContainerType = itk::VectorContainer<unsigned, mitk::DataNode::Pointer>;

  std::string nodeName;
  mitk::DataStorage::SetOfObjects::ConstPointer SetofObj = GetDataStorage()->GetAll();
  for (auto dataNode : *SetofObj)
  {
    if (dataNode.IsNull())
    {
      return;
    }

    dataNode->GetName(nodeName);
    // mitk::Surface* p_surface = dynamic_cast<mitk::Surface*>(node->GetData());

    // pelvis
    if (nodeName == "pelvis")
    {
      m_pelvis = dynamic_cast<mitk::Surface *>(dataNode->GetData());
      m_Controls.lineEdit_objname_pelvis->setText(QString::fromStdString(nodeName));
    }
    if (nodeName == "ASIS")
    {
      m_asis = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_asis->setText(QString::fromStdString(nodeName));
    }
    if (nodeName == "PT")
    {
      m_pt = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_PT->setText(QString::fromStdString(nodeName));
    }
    // femur
    if (nodeName == "femur_left")
    {
      m_femurL = dynamic_cast<mitk::Surface *>(dataNode->GetData());
      m_Controls.lineEdit_objname_femurL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[0] = m_femurL;
    }
    if (nodeName == "femur_clipped_left")
    {
      m_femurClipL = dynamic_cast<mitk::Surface *>(dataNode->GetData());
      m_femurLeftGroup[7] = m_femurClipL;
    }
    if (nodeName == "femurimage_left")
    {
      m_femurImageL = dynamic_cast<mitk::Image *>(dataNode->GetData());
      m_Controls.lineEdit_objname_femurImageL->setText(QString::fromStdString(nodeName));
    }
    if (nodeName == "knee_left")
    {
      m_kneeL = dynamic_cast<mitk::Surface *>(dataNode->GetData());
      m_Controls.lineEdit_objname_kneeL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[1] = m_kneeL;
    }
    if (nodeName == "femurAxis_left")
    {
      m_canalAxisL = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_femurAxisL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[2] = m_canalAxisL;
    }
    if (nodeName == "trochanter_left")
    {
      m_trochanterL = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_trochanterL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[3] = m_trochanterL;
    }

    if (nodeName == "femurCOR_left")
    {
      m_fhcL = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_femurCorL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[4] = m_fhcL;
    }
    if (nodeName == "Btrochanter_left")
    {
      m_BtrochanterL = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_BTrochanterL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[5] = m_BtrochanterL;
    }
    if (nodeName == "condyles_left")
    {
      m_condylesL = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_condylesL->setText(QString::fromStdString(nodeName));
      m_femurLeftGroup[6] = m_condylesL;
    }
    if (nodeName == "femur_right")
    {
      m_femurR = dynamic_cast<mitk::Surface *>(dataNode->GetData());
      m_Controls.lineEdit_objname_femurR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[0] = m_femurR;
    }
    if (nodeName == "femur_clipped_right")
    {
      m_femurClipR = dynamic_cast<mitk::Surface *>(dataNode->GetData());
      m_femurRightGroup[7] = m_femurClipR;
    }
    if (nodeName == "femurimage_right")
    {
      m_femurImageR = dynamic_cast<mitk::Image *>(dataNode->GetData());
      m_Controls.lineEdit_objname_femurImageR->setText(QString::fromStdString(nodeName));
    }
    if (nodeName == "knee_right")
    {
      m_kneeR = dynamic_cast<mitk::Surface *>(dataNode->GetData());
      m_Controls.lineEdit_objname_kneeR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[1] = m_kneeR;
    }
    if (nodeName == "femurAxis_right")
    {
      m_canalAxisR = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_femurAxisR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[2] = m_canalAxisR;
    }
    if (nodeName == "trochanter_right")
    {
      m_trochanterR = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_trochanterR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[3] = m_trochanterR;
    }
    if (nodeName == "femurCOR_right")
    {
      m_fhcR = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_femurCorR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[4] = m_fhcR;
    }
    if (nodeName == "Btrochanter_right")
    {
      m_BtrochanterR = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_BTrochanterR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[5] = m_BtrochanterR;
    }
    if (nodeName == "condyles_right")
    {
      m_condylesR = dynamic_cast<mitk::PointSet *>(dataNode->GetData());
      m_Controls.lineEdit_objname_condylesR->setText(QString::fromStdString(nodeName));
      m_femurRightGroup[6] = m_condylesR;
    }
  }
  if (m_Controls.lineEdit_objname_pelvis->text() != "" && m_Controls.lineEdit_objname_asis->text() != "" &&
      m_Controls.lineEdit_objname_PT->text() != "")
  {
    m_Controls.pushButton_buildPelvis->setEnabled(true);
  }

  if (m_Controls.lineEdit_objname_femurL->text() != "" && m_Controls.lineEdit_objname_femurAxisL->text() != "" &&
      m_Controls.lineEdit_objname_femurCorL->text() != "" && m_Controls.lineEdit_objname_trochanterL->text() != "")
  {
    m_Controls.pushButton_buildFemurL->setEnabled(true);
  }

  if (m_Controls.lineEdit_objname_femurR->text() != "" && m_Controls.lineEdit_objname_femurAxisR->text() != "" &&
      m_Controls.lineEdit_objname_femurCorR->text() != "" && m_Controls.lineEdit_objname_trochanterR->text() != "")
  {
    m_Controls.pushButton_buildFemurR->setEnabled(true);
  }
}

void VCView::ApplyPelvicVC()
{
  calRotation();
  double rotateVec_Z[3]{0, 0, 1};
  double rotateVec_X[3]{1, 0, 0};
  double rotateVec_Y[3]{0, 1, 0};
  MITK_INFO << "rotate Z: " << m_rotate_Z;
  MITK_INFO << "rotate Y: " << m_rotate_Y;
  mitk::RotationOperation *op_rotate_Y =
    new mitk::RotationOperation(mitk::OpROTATE, m_asis->GetPoint(0), mitk::Vector3D{rotateVec_Y}, m_rotate_Y);
  mitk::RotationOperation *op_rotate_Z =
    new mitk::RotationOperation(mitk::OpROTATE, m_asis->GetPoint(0), mitk::Vector3D{rotateVec_Z}, m_rotate_Z);

  mitk::DataStorage::SetOfObjects::ConstPointer SetofObj = GetDataStorage()->GetAll();
  for (auto dataNode : *SetofObj)
  {
    if (dataNode->GetData() != nullptr)
    {
      auto data = dataNode->GetData();
      if (data->GetGeometry() != nullptr)
      {
        data->GetGeometry()->ExecuteOperation(op_rotate_Y);
        data->GetGeometry()->ExecuteOperation(op_rotate_Z);
      }
      else
        MITK_INFO << "geo null";
    }
    else
      MITK_INFO << dataNode->GetName() << " "
                << "data null";
  }
  delete op_rotate_Y, op_rotate_Z;
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::ApplyFemurVC()
{
  if (m_fhcL == nullptr || m_fhcR == nullptr || m_femurImageL == nullptr || m_femurImageR == nullptr)
  {
    return;
  }

  calMechanCorrectionL();
  MITK_INFO << "m_rotateFemurLeftAxis: " << m_rotateFemurLeftAxis[0] << "," << m_rotateFemurLeftAxis[1] << ","
            << m_rotateFemurLeftAxis[2];
  mitk::Vector3D rotateAxisL(m_rotateFemurLeftAxis);
  MITK_INFO << "!L:" << rotateAxisL;
  mitk::RotationOperation *op_L =
    new mitk::RotationOperation(mitk::OpROTATE, m_fhcL->GetPoint(0), rotateAxisL, m_rotateFemurLeftAngle * 180 / PI);
  for (auto data : m_femurLeftGroup)
  {
    if (data != nullptr)
    {
      data->GetGeometry()->ExecuteOperation(op_L);
    }
    else
    {
      MITK_ERROR << "group data null";
    }
  }
  m_femurImageL->GetGeometry()->ExecuteOperation(op_L);
  delete op_L;
  //----------------------------------------------------------------------------------------------------------------
  MITK_INFO << "1: " << m_rotateFemurRightAxis[0] << "," << m_rotateFemurRightAxis[1] << ","
            << m_rotateFemurRightAxis[2];
  mitk::Vector3D rotateAxisR(m_rotateFemurRightAxis);
  MITK_INFO << "!R:" << rotateAxisR;
  // double axis[3]{ 0.348401, -0.709679, 0.612349 };
  // mitk::Vector3D rotateAxisR{ axis };
  mitk::RotationOperation *op_R =
    new mitk::RotationOperation(mitk::OpROTATE, m_fhcR->GetPoint(0), rotateAxisR, m_rotateFemurRightAngle * 180 / PI);
  for (auto data : m_femurRightGroup)
  {
    if (data != nullptr)
    {
      data->GetGeometry()->ExecuteOperation(op_R);
    }
    else
    {
      MITK_ERROR << "group data null";
    }
  }
  m_femurImageR->GetGeometry()->ExecuteOperation(op_R);
  delete op_R;

  /*vtkSmartPointer<vtkTransform> transform_opside = vtkTransform::New();
  transform_opside->SetMatrix(m_femurR->GetUpdatedGeometry()->GetVtkTransform()->GetMatrix());
  // transform_opside->Concatenate(THA_MODEL.Femur_opSide()->GetMatrix(lancetAlgorithm::CANAL).data());
  transform_opside->Concatenate(THA_MODEL.Femur_opSide()->GetMatrix(lancetAlgorithm::MECHANICS).data());

  mitk::ApplyTransformMatrixOperation *op_opside = new mitk::ApplyTransformMatrixOperation(
    mitk::OpAPPLYTRANSFORMMATRIX, transform_opside->GetMatrix(), mitk::Point3D{refPoint});


  // FemurImageR
  double rotateVec_X[3]{ 1, 0, 0 };
  double rotateVec_Y[3]{ 0, 1, 0 };
  double rotateVec_Z[3]{ 0, 0, 1 };
  std::vector<double> op_R = this->vtkmatrix2angle(transform_opside->GetMatrix());
  double m_beforeR = m_fhcR->GetPoint(0)[0];
  // mitk::RotationOperation *op_rotateR_X =
  //   new mitk::RotationOperation(mitk::OpROTATE, m_fhcR->GetPoint(0), mitk::Vector3D{rotateVec_X}, op_R[0] -
  m_rotate_X);
  // mitk::RotationOperation *op_rotateR_Y =
  //   new mitk::RotationOperation(mitk::OpROTATE, m_fhcR->GetPoint(0), mitk::Vector3D{rotateVec_Y}, op_R[1] -
  m_rotate_Y);
  // mitk::RotationOperation *op_rotateR_Z =
  //   new mitk::RotationOperation(mitk::OpROTATE, m_fhcR->GetPoint(0), mitk::Vector3D{rotateVec_Z}, op_R[2] -
  m_rotate_Z); mitk::RotationOperation* op_rotateR_X = new mitk::RotationOperation(mitk::OpROTATE, m_fhcR->GetPoint(0),
  mitk::Vector3D{ rotateVec_X }, op_R[0] ); mitk::RotationOperation* op_rotateR_Y = new
  mitk::RotationOperation(mitk::OpROTATE, m_fhcR->GetPoint(0), mitk::Vector3D{ rotateVec_Y }, op_R[1] );
  mitk::RotationOperation* op_rotateR_Z =
      new mitk::RotationOperation(mitk::OpROTATE, m_fhcR->GetPoint(0), mitk::Vector3D{ rotateVec_Z }, op_R[2] );
  m_femurImageR->GetGeometry()->ExecuteOperation(op_rotateR_Y);
  m_femurImageR->GetGeometry()->ExecuteOperation(op_rotateR_X);
  m_femurImageR->GetGeometry()->ExecuteOperation(op_rotateR_Z);
  delete op_rotateR_X, op_rotateR_Y, op_rotateR_Z;

  for (auto data : m_femurRightGroup)
  {
    if (data != nullptr)
    {
      data->GetGeometry()->ExecuteOperation(op_opside);
    }
    else
    {
      MITK_ERROR << "group data null";
    }
  }
  delete op_opside;*/

  BuildFemurL();
  BuildFemurR();
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::SetOpSide(QString side)
{
  if (side == "right")
  {
    THA_MODEL.SetOprationSide(ESide::right);
    MITK_INFO << "Set OpSide right";
  }

  if (side == "left")
  {
    THA_MODEL.SetOprationSide(ESide::left);
    MITK_INFO << "Set OpSide left";
  }
}

void VCView::BuildPelvis()
{
  if (m_asis == nullptr || m_pt == nullptr)
  {
    MITK_ERROR << "BuildPelvis == nullptr";
    return;
  }

  auto RASI = m_asis->GetPoint(0);
  auto LASI = m_asis->GetPoint(1);
  auto PT = m_pt->GetPoint(0);

  THA_MODEL.BuildPelvis(3, RASI.GetDataPointer(), LASI.GetDataPointer(), PT.GetDataPointer());

  MITK_INFO << "Pelvis model build";
  m_builtPelvis = true;
  m_Controls.pushButton_applyPelvicVC->setEnabled(true);

  double pt = 0;
  THA_MODEL.Pelvis()->GetResult(EResult::f_PT, pt);
  MITK_INFO << "pt:" << pt;
}

void VCView::BuildFemurL()
{
  //左腿
  if (m_canalAxisL == nullptr || m_fhcL == nullptr || m_trochanterL == nullptr)
  {
    MITK_ERROR << "BuildFemurL == nullptr";
    return;
  }
  mitk::Point3D PFCA = m_canalAxisL->GetPoint(0);
  mitk::Point3D DFCA = m_canalAxisL->GetPoint(1);
  mitk::Point3D FHC = m_fhcL->GetPoint(0);
  mitk::Point3D LT = m_trochanterL->GetPoint(0);
  // MITK_INFO << "PFCA: " << PFCA;
  // MITK_INFO << "DFCA: " << DFCA;
  // MITK_INFO << "FHC: " << FHC;
  // MITK_INFO << "LT: " << LT;

  THA_MODEL.BuildFemur(
    ESide::left, DFCA.GetDataPointer(), PFCA.GetDataPointer(), FHC.GetDataPointer(), LT.GetDataPointer());

  MITK_INFO << "FemurL model build";
  m_builtFemurL = true;
  if (m_builtFemurL && m_builtFemurR && m_builtPelvis)
  {
    m_Controls.pushButton_applyFemurVC->setEnabled(true);
  }
}

void VCView::BuildFemurR()
{
  //右腿
  if (m_canalAxisR == nullptr || m_fhcR == nullptr || m_trochanterR == nullptr)
  {
    MITK_ERROR << "BuildFemurR == nullptr";
    return;
  }

  mitk::Point3D PFCA_right = m_canalAxisR->GetPoint(0);
  mitk::Point3D DFCA_right = m_canalAxisR->GetPoint(1);
  mitk::Point3D FHC_right = m_fhcR->GetPoint(0);
  mitk::Point3D LT_right = m_trochanterR->GetPoint(0);
  // MITK_INFO << "PFCA_right: " << PFCA_right;
  // MITK_INFO << "DFCA_right: " << DFCA_right;
  // MITK_INFO << "FHC_right: " << FHC_right;
  // MITK_INFO << "LT_right: " << LT_right;

  THA_MODEL.BuildFemur(ESide::right,
                       DFCA_right.GetDataPointer(),
                       PFCA_right.GetDataPointer(),
                       FHC_right.GetDataPointer(),
                       LT_right.GetDataPointer());
  MITK_INFO << "FemurR model build";

  m_builtFemurR = true;
  if (m_builtFemurL && m_builtFemurR)
  {
    m_Controls.pushButton_applyFemurVC->setEnabled(true);
  }
}

void VCView::calRotation()
{
  mitk::Point3D p_left = m_asis->GetPoint(1);
  mitk::Point3D p_right = m_asis->GetPoint(0);

  MITK_INFO << p_left;
  MITK_INFO << p_right;

  // Eigen::Vector3d vec{p_right[0] - p_left[0], p_right[1] - p_left[1], p_right[2] - p_left[2]};
  Eigen::Vector3d vec{p_left[0] - p_right[0], p_left[1] - p_right[1], p_left[2] - p_right[2]};

  double Rotate_Y = fabs(atan2(vec[2], vec[0]) * (180.0f / M_PI));
  vec[2] > 0.0 ? m_rotate_Y = Rotate_Y : m_rotate_Y = -Rotate_Y;
  // vec[2] < 0.0 ? m_rotate_Y = Rotate_Y : m_rotate_Y = -Rotate_Y;

  double Rotate_Z = fabs(atan2(vec[1], vec[0]) * (180.0f / M_PI));
  // vec[1] > 0.0 ? m_rotate_Z = Rotate_Z : m_rotate_Z = -Rotate_Z;
  vec[1] < 0.0 ? m_rotate_Z = Rotate_Z : m_rotate_Z = -Rotate_Z;
}

void VCView::calMechanCorrectionL()
{
  // left
  Eigen::Vector3d axis_z_l, axis_x_l, axis_y_l, axis_z_r, axis_x_r, axis_y_r;
  Eigen::AngleAxisd angle_axisd_l, angle_axisd_r;
  auto FHC_L = m_fhcL->GetPoint(0).GetDataPointer();
  auto PFCA_L = m_canalAxisL->GetPoint(0).GetDataPointer();
  auto DFCA_L = m_canalAxisL->GetPoint(1).GetDataPointer();
  axis_z_l << FHC_L[0] - DFCA_L[0], FHC_L[1] - DFCA_L[1], FHC_L[2] - DFCA_L[2];
  axis_z_l.normalize();

  axis_y_l = axis_z_l.cross(Eigen::Vector3d{PFCA_L[0] - FHC_L[0], PFCA_L[1] - FHC_L[1], PFCA_L[2] - FHC_L[2]});
  axis_y_l.normalize();
  axis_x_l = axis_y_l.cross(axis_z_l);

  //===========cal orientation matrix============
  Eigen::Matrix3d rotation_l;
  rotation_l.setIdentity();
  for (unsigned int i = 0; i < 3; i++)
  {
    rotation_l(i, 0) = axis_x_l[i];
    rotation_l(i, 1) = axis_y_l[i];
    rotation_l(i, 2) = axis_z_l[i];
  }
  rotation_l = rotation_l.inverse().eval();

  angle_axisd_l.fromRotationMatrix(rotation_l);
  m_rotateFemurLeftAngle = angle_axisd_l.angle();
  MITK_INFO << "m_rotateFemurLeftAngle" << m_rotateFemurLeftAngle;
  memcpy(&m_rotateFemurLeftAxis, angle_axisd_r.axis().data(), 3 * sizeof(double));
  // m_rotateFemurLeftAxis = angle_axisd_l.axis().data();
  MITK_INFO << "m_rotateFemurLeftAxis: " << m_rotateFemurLeftAxis[0] << "," << m_rotateFemurLeftAxis[1] << ","
            << m_rotateFemurLeftAxis[2];

  // right
  auto FHC_R = m_fhcR->GetPoint(0).GetDataPointer();
  auto PFCA_R = m_canalAxisR->GetPoint(0).GetDataPointer();
  auto DFCA_R = m_canalAxisR->GetPoint(1).GetDataPointer();

  axis_z_r << FHC_R[0] - DFCA_R[0], FHC_R[1] - DFCA_R[1], FHC_R[2] - DFCA_R[2];
  axis_z_r.normalize();

  axis_y_r = Eigen::Vector3d{PFCA_R[0] - FHC_R[0], PFCA_R[1] - FHC_R[1], PFCA_R[2] - FHC_R[2]}.cross(axis_z_r);
  axis_y_r.normalize();
  axis_x_r = axis_y_r.cross(axis_z_r);

  //===========cal orientation matrix============
  Eigen::Matrix3d rotation_r;
  rotation_r.setIdentity();
  for (unsigned int i = 0; i < 3; i++)
  {
    rotation_r(i, 0) = axis_x_r[i];
    rotation_r(i, 1) = axis_y_r[i];
    rotation_r(i, 2) = axis_z_r[i];
  }
  rotation_r = rotation_r.inverse().eval();

  angle_axisd_r.fromRotationMatrix(rotation_r);
  m_rotateFemurRightAngle = angle_axisd_r.angle();
  MITK_INFO << "m_rotateFemurRightAngle" << m_rotateFemurRightAngle;
  memcpy(&m_rotateFemurRightAxis, angle_axisd_r.axis().data(), 3 * sizeof(double));
  // m_rotateFemurRightAxis = angle_axisd_r.axis().data();
  MITK_INFO << "m_rotateFemurRightAngle: " << m_rotateFemurRightAxis[0] << "," << m_rotateFemurRightAxis[1] << ","
            << m_rotateFemurRightAxis[2];
}

void VCView::AdjustPelvisTilt(bool positive)
{
  if (m_pelvis != nullptr || m_asis != nullptr)
  {
    MITK_INFO << "RoTationPelvisXAdd";
    double axis[3]{1, 0, 0};
    // mitk::Point3D rotateCenter{ m_pelvis->GetGeometry()->GetCenter() };
    mitk::Vector3D rotateAxis{axis};
    double rotateAngle = m_Controls.doubleSpinBox_RotateXAdd->value();

    if (!positive)
    {
      rotateAngle = -rotateAngle;
    }
    m_rotate_X += rotateAngle;
    MITK_INFO << "m_rotate_X " << m_rotate_X;

    auto *doOp = new mitk::RotationOperation(mitk::OpROTATE, m_asis->GetPoint(0), rotateAxis, rotateAngle);

    mitk::DataStorage::SetOfObjects::ConstPointer SetofObj = GetDataStorage()->GetAll();
    for (auto dataNode : *SetofObj)
    {
      if (dataNode->GetData() != nullptr)
      {
        auto data = dataNode->GetData();
        if (data->GetGeometry() != nullptr)
        {
          data->GetGeometry()->ExecuteOperation(doOp);
        }
        else
          MITK_INFO << "geo null";
      }
      else
        MITK_INFO << dataNode->GetName() << " "
                  << "data null";
    }
    delete doOp;
    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
  }
}

void VCView::RoTationPelvisXAdd()
{
  AdjustPelvisTilt(true);
}

void VCView::RoTationPelvisXSub()
{
  AdjustPelvisTilt(false);
}

void VCView::SetRoTationPelvisXAddValue()
{
  m_rotate_XAdd = m_Controls.doubleSpinBox_RotateXAdd->value();
  MITK_INFO << "m_rotate_XAdd" << m_rotate_XAdd;
}

void VCView::SetRoTationPelvisXSubValue()
{
  m_rotate_XSub = m_Controls.doubleSpinBox_RotateXSub->value();
  MITK_INFO << "m_rotate_XSub" << m_rotate_XSub;
}

mitk::Image::Pointer VCView::SurfaceCutImage(mitk::Surface *surface,
                                             mitk::Image *image,
                                             bool isReverseStencil,
                                             bool isSmallestCut)
{
  mitk::Image::Pointer res = mitk::Image::New();
  // stencil
  mitk::SurfaceToImageFilter::Pointer surface_to_image = mitk::SurfaceToImageFilter::New();
  surface_to_image->SetImage(image);
  surface_to_image->SetInput(surface);
  surface_to_image->SetReverseStencil(isReverseStencil);
  surface_to_image->Update();

  if (!isSmallestCut)
  {
    res = surface_to_image->GetOutput()->Clone();
    return res;
  }
  // boundingBox
  auto boundingBox = mitk::GeometryData::New();
  // InitializeWithSurfaceGeometry
  auto boundingGeometry = mitk::Geometry3D::New();
  auto geometry = surface->GetGeometry();
  boundingGeometry->SetBounds(geometry->GetBounds());
  boundingGeometry->SetOrigin(geometry->GetOrigin());
  boundingGeometry->SetSpacing(geometry->GetSpacing());
  boundingGeometry->SetIndexToWorldTransform(geometry->GetIndexToWorldTransform()->Clone());
  boundingGeometry->Modified();
  boundingBox->SetGeometry(boundingGeometry);

  auto cutter = mitk::BoundingShapeCropper::New();
  cutter->SetGeometry(boundingBox);
  cutter->SetInput(surface_to_image->GetOutput());
  cutter->Update();
  res = cutter->GetOutput()->Clone();
  return res;
}

void VCView::StlCutImage()
{
  // //cut two legs form whole image
  // //stencil
  // mitk::SurfaceToImageFilter::Pointer surface_to_image = mitk::SurfaceToImageFilter::New();
  // surface_to_image->SetImage(image);
  // surface_to_image->SetInput(m_femurL);
  // surface_to_image->SetReverseStencil(false);
  // //surface_to_image->GenerateData();
  // surface_to_image->Update();
  // //boundingBox
  //
  // auto cutter = mitk::BoundingShapeCropper::New();
  // auto boundingBox = mitk::GeometryData::New();
  // //InitializeWithSurfaceGeometry
  // auto boundingGeometry = mitk::Geometry3D::New();
  // boundingGeometry->SetBounds(m_femurL->GetGeometry()->GetBounds());
  // boundingGeometry->SetOrigin(m_femurL->GetGeometry()->GetOrigin());
  // boundingGeometry->SetSpacing(m_femurL->GetGeometry()->GetSpacing());
  // boundingGeometry->SetIndexToWorldTransform(m_femurL->GetGeometry()->GetIndexToWorldTransform()->Clone());
  // boundingGeometry->Modified();
  //
  // boundingBox->SetGeometry(boundingGeometry);
  //
  // cutter->SetGeometry(boundingBox);
  //
  // cutter->SetInput(surface_to_image->GetOutput());
  // cutter->Update();
  auto image = GetDataStorage()->GetNamedObject<mitk::Image>("srcimage");
  if (image == nullptr)
  {
    MITK_INFO << "no image";
    return;
  }
  // image without legs
  auto imageWithoutLegL = SurfaceCutImage(m_femurL, image, true, false);
  auto imageWithoutLegs = SurfaceCutImage(m_femurR, imageWithoutLegL, true, false);
  mitk::DataNode::Pointer imageNodeWithoutLegs = mitk::DataNode::New();
  imageNodeWithoutLegs->SetName("imageWithoutLegs");
  imageNodeWithoutLegs->SetData(imageWithoutLegs);

  GetDataStorage()->Add(imageNodeWithoutLegs);
}

void VCView::StlCutLeftImage()
{
  auto image = GetDataStorage()->GetNamedObject<mitk::Image>("srcimage");

  if (m_femurL == nullptr || image == nullptr)
  {
    MITK_INFO << "no femur L";
    return;
  }

  // left leg
  auto leftLegImage = SurfaceCutImage(m_femurL, image, false, true);
  if (leftLegImage != nullptr)
  {
    mitk::DataNode::Pointer croppedImageNodeL = mitk::DataNode::New();
    croppedImageNodeL->SetName("femurL_Image");
    croppedImageNodeL->SetData(leftLegImage);
    GetDataStorage()->Add(croppedImageNodeL);
  }
  else
    MITK_ERROR << "leftLegImage null";
}

void VCView::StlCutRightImage()
{
  auto image = GetDataStorage()->GetNamedObject<mitk::Image>("srcimage");

  if (m_femurR == nullptr || image == nullptr)
  {
    MITK_INFO << "no femur R";
    return;
  }
  // right leg
  auto rightLegImage = SurfaceCutImage(m_femurR, image, false, true);
  if (rightLegImage != nullptr)
  {
    mitk::DataNode::Pointer croppedImageNodeR = mitk::DataNode::New();
    croppedImageNodeR->SetName("femurR_Image");
    croppedImageNodeR->SetData(rightLegImage);
    GetDataStorage()->Add(croppedImageNodeR);
  }
  else
    MITK_ERROR << "rightLegImage null";
}

void VCView::StlCutOverlayImage() 
{
  auto image = GetDataStorage()->GetNamedObject<mitk::Image>("srcimage");
  auto overlay = GetDataStorage()->GetNamedObject<mitk::Surface>("pelvis_overlay");

  if (overlay == nullptr || image == nullptr)
  {
    MITK_INFO << "no pelvis_overlay";
    return;
  }

  auto overlayImage = SurfaceCutImage(overlay, image, false, true);
  if (overlayImage != nullptr)
  {
    mitk::DataNode::Pointer ImageNode = mitk::DataNode::New();
    ImageNode->SetName("pelvis_overlay_side");
    ImageNode->SetData(overlayImage);
    GetDataStorage()->Add(ImageNode);
  }
  else
    MITK_ERROR << "ImageNode null";

  mitk::DataNode::Pointer pNode11 = this->GetDataStorage()->GetNamedNode("srcimage");
  if (pNode11 != nullptr)
  {
    this->GetDataStorage()->Remove(pNode11);
  }
  mitk::DataNode::Pointer pNode12 = this->GetDataStorage()->GetNamedNode("pelvis_overlay");
  if (pNode12 != nullptr)
  {
    this->GetDataStorage()->Remove(pNode12);
  }
  mitk::DataNode::Pointer pNode13 = this->GetDataStorage()->GetNamedNode("pelvis_overlay_side");
  if (pNode13 != nullptr)
  {
    pNode13->SetName("pelvis_overlay_left");
  }

}

void VCView::CutLeftFemur()
{
  // Left
  short cutValue = -1023.0;
  auto femurImageL = GetDataStorage()->GetNamedObject<mitk::Image>("femurimage_left");
  auto bigImage = GetDataStorage()->GetNamedObject<mitk::Image>("image");

  if (femurImageL == nullptr || bigImage == nullptr)
  {
    return;
  }
  mitk::ImagePixelReadAccessor<short, 3> readL_accessor(femurImageL);
  mitk::ImagePixelWriteAccessor<short, 3> write_accessor(bigImage);

  auto sizeL = femurImageL->GetDimensions();
  auto arraysizeL = sizeL[0] * sizeL[1] * sizeL[2];

  for (unsigned n = 0; n < arraysizeL; n++)
  {
    auto *dataAdressBegin = readL_accessor.GetData();
    short valueInFemurImage = *(dataAdressBegin + n);

    if (valueInFemurImage > cutValue)
    {
      try
      {
        auto idx = readL_accessor.Get3DIndex(n);
        mitk::Point3D coord{};
        femurImageL->GetGeometry()->IndexToWorld(idx, coord);
        if (bigImage->GetGeometry()->IsInside(coord))
        {
          itk::Index<3> idx_inBigImage;
          bigImage->GetGeometry()->WorldToIndex(coord, idx_inBigImage);
          /*itk::Index<3> idx_1{ idx_inBigImage[0] + 1,idx_inBigImage[1],idx_inBigImage[2] };
          itk::Index<3> idx_2{ idx_inBigImage[0] - 1,idx_inBigImage[1],idx_inBigImage[2] };
          itk::Index<3> idx_3{ idx_inBigImage[0] ,idx_inBigImage[1] + 1,idx_inBigImage[2] };
          itk::Index<3> idx_4{ idx_inBigImage[0] ,idx_inBigImage[1] - 1,idx_inBigImage[2] };
          itk::Index<3> idx_5{ idx_inBigImage[0] ,idx_inBigImage[1],idx_inBigImage[2] + 1 };
          itk::Index<3> idx_6{ idx_inBigImage[0] ,idx_inBigImage[1],idx_inBigImage[2] - 1 };*/
          write_accessor.SetPixelByIndex(idx_inBigImage, valueInFemurImage);
          /*write_accessor.SetPixelByIndex(idx_1, valueInFemurImage);
          write_accessor.SetPixelByIndex(idx_2, valueInFemurImage);
          write_accessor.SetPixelByIndex(idx_3, valueInFemurImage);
          write_accessor.SetPixelByIndex(idx_4, valueInFemurImage);
          write_accessor.SetPixelByIndex(idx_5, valueInFemurImage);
          write_accessor.SetPixelByIndex(idx_6, valueInFemurImage);*/
          // MITK_INFO << coord;
          // MITK_INFO << idx_inBigImage;
        }
      }
      catch (mitk::Exception &e)
      {
        MITK_ERROR << e;
      }
    }
  }
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::CutRightFemur() {}

void VCView::EnableTransform()
{
  m_surface_transform = m_tmpSurface;
}

void VCView::move(double d[3], mitk::Surface *surface)
{
  if (surface != nullptr)
  {
    mitk::Point3D direciton{d};
    auto *doOp = new mitk::PointOperation(mitk::OpMOVE, 0, direciton, 0);
    // execute the Operation
    // here no undo is stored, because the movement-steps aren't interesting.
    // only the start and the end is interisting to store for undo.
    m_surface_transform->GetGeometry()->ExecuteOperation(doOp);
    delete doOp;
    updateStemCenter();

    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
  }
}

void VCView::rotate(double center[3], double axis[3], mitk::Surface *surface)
{
  if (surface != nullptr)
  {
    mitk::Point3D rotateCenter{center};
    mitk::Vector3D rotateAxis{axis};
    auto *doOp = new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, rotateAxis, 3);
    // execute the Operation
    // here no undo is stored, because the movement-steps aren't interesting.
    // only the start and the end is interisting to store for undo.
    m_surface_transform->GetGeometry()->ExecuteOperation(doOp);
    delete doOp;
    updateStemCenter();
    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
  }
}

void VCView::moveXp()
{
  double p[3]{3, 0, 0};
  move(p, m_surface_transform);
}

void VCView::moveXm()
{
  double p[3]{-3, 0, 0};
  move(p, m_surface_transform);
}

void VCView::moveYm()
{
  double p[3]{0, -3, 0};
  move(p, m_surface_transform);
}

void VCView::moveYp()
{
  double p[3]{0, 3, 0};
  move(p, m_surface_transform);
}

void VCView::moveZm()
{
  double p[3]{0, 0, -3};
  move(p, m_surface_transform);
}

void VCView::moveZp()
{
  double p[3]{0, 0, 3};
  move(p, m_surface_transform);
}

void VCView::rotateXp()
{
  if (m_surface_transform)
  {
    double axis[3]{1, 0, 0};
    rotate(m_surface_transform->GetGeometry()->GetCenter().GetDataPointer(), axis, m_surface_transform);
  }
}

void VCView::rotateXm()
{
  if (m_surface_transform)
  {
    double axis[3]{-1, 0, 0};
    rotate(m_surface_transform->GetGeometry()->GetCenter().GetDataPointer(), axis, m_surface_transform);
  }
}

void VCView::rotateYm()
{
  if (m_surface_transform)
  {
    double axis[3]{0, -1, 0};
    rotate(m_surface_transform->GetGeometry()->GetCenter().GetDataPointer(), axis, m_surface_transform);
  }
}

void VCView::rotateYp()
{
  if (m_surface_transform)
  {
    double axis[3]{0, 1, 0};
    rotate(m_surface_transform->GetGeometry()->GetCenter().GetDataPointer(), axis, m_surface_transform);
  }
}

void VCView::rotateZp()
{
  if (m_surface_transform)
  {
    double axis[3]{0, 0, 1};
    rotate(m_surface_transform->GetGeometry()->GetCenter().GetDataPointer(), axis, m_surface_transform);
  }
}

void VCView::InitStemPos()
{
  vtkSmartPointer<vtkMatrix4x4> matrix = vtkMatrix4x4::New();
  matrix->SetElement(0, 3, 117.0);
  matrix->SetElement(1, 3, -9.0);
  matrix->SetElement(2, 3, -141.0);

  // matrix->Print(std::cout);

  auto *transOp = new mitk::ApplyTransformMatrixOperation(mitk::OpAPPLYTRANSFORMMATRIX, matrix, mitk::Point3D{});
  GetDataStorage()->GetNamedNode("femoral_1")->SetVisibility(true);

  m_stem = GetDataStorage()->GetNamedObject<mitk::Surface>("femoral_1");

  mitk::PointSet::Pointer pSet_stemCenter = mitk::PointSet::New();
  auto dn = mitk::DataNode::New();
  dn->SetName("stem_center");
  dn->SetData(pSet_stemCenter);
  dn->SetFloatProperty("pointsize", 5);
  dn->SetColor(0, 1, 0);
  GetDataStorage()->Add(dn);
  pSet_stemCenter->SetPoint(0, CalStemAssemblyCenter(m_stempara, m_stem->GetGeometry()->GetVtkMatrix()));
  m_stemCenter = pSet_stemCenter;

  m_stemGroup[1] = m_stemCenter;
  m_stemGroup[0] = m_stem;

  for (auto base_data : m_stemGroup)
  {
    if (base_data != nullptr)
    {
      base_data->GetGeometry()->ExecuteOperation(transOp);
    }
    else
      MITK_INFO << "m_stemGroup data null";
  }
  delete transOp;
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::SelectStem(int index)
{
  vtkNew<vtkSTLReader> reader;

  switch (index)
  {
    case 0:
      reader->SetFileName("D:/ImageSource/stem_model/femoral_1.stl");
      reader->Update();
      m_stempara = StemParameter{1, 135, 31.8};
      break;
    case 1:
      reader->SetFileName("D:/ImageSource/stem_model/femoral_9.stl");
      reader->Update();
      m_stempara = StemParameter{1, 135, 42.1};
      break;
    default:
      reader->SetFileName("D:/ImageSource/stem_model/femoral_1.stl");
      reader->Update();
      m_stempara = StemParameter{1, 135, 31.8};
      break;
  }

  // replace stl
  auto *doOp = new mitk::SurfaceOperation(mitk::OpSURFACECHANGED, reader->GetOutput(), 0);
  auto surface = GetDataStorage()->GetNamedObject<mitk::Surface>("femoral_1");

  if (surface != nullptr)
  {
    vtkSmartPointer<vtkMatrix4x4> matrix = vtkMatrix4x4::New();
    surface->GetGeometry()->GetVtkTransform()->Print(std::cout);
    surface->GetGeometry()->GetVtkMatrix()->Print(std::cout);
    matrix->DeepCopy(surface->GetGeometry()->GetVtkTransform()->GetMatrix());

    matrix->Print(std::cout);
    surface->ExecuteOperation(doOp);
    double ref[3]{0, 0, 0};
    mitk::Point3D refp{ref};
    auto *transOp = new mitk::ApplyTransformMatrixOperation(mitk::OpAPPLYTRANSFORMMATRIX, matrix, refp);
    surface->GetGeometry()->ExecuteOperation(transOp);
  }
  updateStemCenter();
}

void VCView::updateStemCenter()
{
  auto pset = GetDataStorage()->GetNamedObject<mitk::PointSet>("stem_center");
  auto stem = GetDataStorage()->GetNamedObject<mitk::Surface>("femoral_1");
  if (stem != nullptr && pset != nullptr)
  {
    // stem->GetGeometry()->GetVtkMatrix()->Print(std::cout);
    pset->SetPoint(0, CalStemAssemblyCenter(m_stempara, stem->GetGeometry()->GetVtkMatrix()));
  }
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::cutLFemur()
{
  // Left
  short cutValue = -1023.0;
  auto femurImageL = GetDataStorage()->GetNamedObject<mitk::Image>("femurimage_left");
  auto bigImage = GetDataStorage()->GetNamedObject<mitk::Image>("image");

  if (femurImageL == nullptr || bigImage == nullptr)
  {
    return;
  }
  mitk::ImagePixelReadAccessor<short, 3> readL_accessor(femurImageL);
  mitk::ImagePixelWriteAccessor<short, 3> write_accessor(bigImage);

  auto sizeL = femurImageL->GetDimensions();
  auto arraysizeL = sizeL[0] * sizeL[1] * sizeL[2];

  for (unsigned n = 0; n < arraysizeL; n++)
  {
    auto *dataAdressBegin = readL_accessor.GetData();
    short valueInFemurImage = *(dataAdressBegin + n);

    if (valueInFemurImage > cutValue)
    {
      try
      {
        auto idx = readL_accessor.Get3DIndex(n);
        mitk::Point3D coord{};
        femurImageL->GetGeometry()->IndexToWorld(idx, coord);
        if (bigImage->GetGeometry()->IsInside(coord))
        {
          itk::Index<3> idx_inBigImage;
          bigImage->GetGeometry()->WorldToIndex(coord, idx_inBigImage);
          /*itk::Index<3> idx_1{ idx_inBigImage[0] + 1,idx_inBigImage[1],idx_inBigImage[2] };
          itk::Index<3> idx_2{ idx_inBigImage[0] - 1,idx_inBigImage[1],idx_inBigImage[2] };
          itk::Index<3> idx_3{ idx_inBigImage[0] ,idx_inBigImage[1] + 1,idx_inBigImage[2] };
          itk::Index<3> idx_4{ idx_inBigImage[0] ,idx_inBigImage[1] - 1,idx_inBigImage[2] };
          itk::Index<3> idx_5{ idx_inBigImage[0] ,idx_inBigImage[1],idx_inBigImage[2] + 1 };
          itk::Index<3> idx_6{ idx_inBigImage[0] ,idx_inBigImage[1],idx_inBigImage[2] - 1 };*/
          write_accessor.SetPixelByIndex(idx_inBigImage, valueInFemurImage);
          /*write_accessor.SetPixelByIndex(idx_1, valueInFemurImage);
          write_accessor.SetPixelByIndex(idx_2, valueInFemurImage);
          write_accessor.SetPixelByIndex(idx_3, valueInFemurImage);
          write_accessor.SetPixelByIndex(idx_4, valueInFemurImage);
          write_accessor.SetPixelByIndex(idx_5, valueInFemurImage);
          write_accessor.SetPixelByIndex(idx_6, valueInFemurImage);*/
          // MITK_INFO << coord;
          // MITK_INFO << idx_inBigImage;
        }
      }
      catch (mitk::Exception &e)
      {
        MITK_ERROR << e;
      }
    }
  }
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::cutRFemur()
{
  // Right
  short cutValue = -1023.0;
  auto bigImage = GetDataStorage()->GetNamedObject<mitk::Image>("image");
  auto femurImageR = GetDataStorage()->GetNamedObject<mitk::Image>("femurimage_right");
  if (femurImageR == nullptr || bigImage == nullptr)
  {
    return;
  }
  mitk::ImagePixelReadAccessor<short, 3> readR_accessor(femurImageR);
  mitk::ImagePixelWriteAccessor<short, 3> write_accessor(bigImage);
  auto sizeR = femurImageR->GetDimensions();
  auto arraysizeR = sizeR[0] * sizeR[1] * sizeR[2];

  for (unsigned n = 0; n < arraysizeR; n++)
  {
    auto *dataAdressBeginR = readR_accessor.GetData();
    short valueInFemurImageR = *(dataAdressBeginR + n);
    if (valueInFemurImageR > cutValue)
    {
      try
      {
        auto idx = readR_accessor.Get3DIndex(n);
        mitk::Point3D coord{};
        femurImageR->GetGeometry()->IndexToWorld(idx, coord);
        if (bigImage->GetGeometry()->IsInside(coord))
        {
          itk::Index<3> idx_inBigImageR;
          bigImage->GetGeometry()->WorldToIndex(coord, idx_inBigImageR);
          write_accessor.SetPixelByIndex(idx_inBigImageR, valueInFemurImageR);
        }
      }
      catch (mitk::Exception &e)
      {
        MITK_ERROR << e;
      }
    }
  }
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::upSampling()
{
  // auto femurImage = GetDataStorage()->GetNamedObject<mitk::Image>("femurimage_left");
  // femurImage->itk
  //// Typedef's for pixel, image, reader and writer types
  // using T_InputPixel = short;

  //// Doesn't work for RGB pixels
  //// using T_OutputPixel = unsigned char;
  //// using T_InputPixel = itk::CovariantVector<unsigned char, 3>;
  //// using T_OutputPixel = itk::CovariantVector<unsigned char, 3>;

  // using T_Image = itk::Image<T_InputPixel, 3>;

  // using T_WritePixel = short;
  // using T_WriteImage = itk::Image<T_WritePixel, 3>;

  //// Typedefs for the different (numerous!) elements of the "resampling"

  //// Identity transform.
  //// We don't want any transform on our image except rescaling which is not
  //// specified by a transform but by the input/output spacing as we will see
  //// later.
  //// So no transform will be specified.
  // using T_Transform = itk::IdentityTransform<double, 3>;

  //// If ITK resampler determines there is something to interpolate which is
  //// usually the case when upscaling (!) then we must specify the interpolation
  //// algorithm. In our case, we want bicubic interpolation. One way to implement
  //// it is with a third order b-spline. So the type is specified here and the
  //// order will be specified with a method call later on.
  // using T_Interpolator = itk::BSplineInterpolateImageFunction<T_Image, double, double>;

  //// The resampler type itself.
  // using T_ResampleFilter = itk::ResampleImageFilter<T_Image, T_Image>;

  //// Prepare the resampler.

  //// Instantiate the transform and specify it should be the id transform.
  // T_Transform::Pointer _pTransform = T_Transform::New();
  //_pTransform->SetIdentity();

  //// Instantiate the b-spline interpolator and set it as the third order
  //// for bicubic.
  // T_Interpolator::Pointer _pInterpolator = T_Interpolator::New();
  //_pInterpolator->SetSplineOrder(3);

  //// Instantiate the resampler. Wire in the transform and the interpolator.
  // T_ResampleFilter::Pointer _pResizeFilter = T_ResampleFilter::New();
  //_pResizeFilter->SetTransform(_pTransform);
  //_pResizeFilter->SetInterpolator(_pInterpolator);

  //// Set the output origin. You may shift the original image "inside" the
  //// new image size by specifying something else than 0.0, 0.0 here.

  // const double vfOutputOrigin[3] = { 0.0, 0.0,0.0 };
  //_pResizeFilter->SetOutputOrigin(vfOutputOrigin);

  ////     Compute and set the output spacing
  ////     Compute the output spacing from input spacing and old and new sizes.
  ////
  ////     The computation must be so that the following holds:
  ////
  ////     new width         old x spacing
  ////     ----------   =   ---------------
  ////     old width         new x spacing
  ////
  ////
  ////     new height         old y spacing
  ////    ------------  =   ---------------
  ////     old height         new y spacing
  ////
  ////     So either we specify new height and width and compute new spacings (as
  ////     we do here) or we specify new spacing and compute new height and width
  ////     and computations that follows need to be modified a little (as it is
  ////     done at step 2 there:
  ////       http://itk.org/Wiki/ITK/Examples/DICOM/ResampleDICOM)
  ////

  //// Fetch original image size.
  // const T_Image::RegionType& inputRegion = pReader->GetOutput()->GetLargestPossibleRegion();
  // const T_Image::SizeType& vnInputSize = inputRegion.GetSize();
  // unsigned int                nOldWidth = vnInputSize[0];
  // unsigned int                nOldHeight = vnInputSize[1];

  //// Fetch original image spacing.
  // const T_Image::SpacingType& vfInputSpacing = pReader->GetOutput()->GetSpacing();
  //// Will be {1.0, 1.0} in the usual
  //// case.

  // double vfOutputSpacing[2];
  // vfOutputSpacing[0] = vfInputSpacing[0] * (double)nOldWidth / (double)nNewWidth;
  // vfOutputSpacing[1] = vfInputSpacing[1] * (double)nOldHeight / (double)nNewHeight;

  //// Set the output spacing. If you comment out the following line, the original
  //// image will be simply put in the upper left corner of the new image without
  //// any scaling.
  //_pResizeFilter->SetOutputSpacing(vfOutputSpacing);

  //// Set the output size as specified on the command line.

  // itk::Size<2> vnOutputSize = { { nNewWidth, nNewHeight } };
  //_pResizeFilter->SetSize(vnOutputSize);

  //// Specify the input.

  //_pResizeFilter->SetInput(pReader->GetOutput());

  //// Write the result
  // T_Writer::Pointer pWriter = T_Writer::New();
  // pWriter->SetFileName(argv[2]);
  // pWriter->SetInput(_pResizeFilter->GetOutput());
  // pWriter->Update();
}

void VCView::sealImage()
{
  auto padImageFilter = mitk::PadImageFilter::New();
  // padImageFilter->SetInput();
}

mitk::Point3D VCView::CalStemAssemblyCenter(StemParameter para, vtkMatrix4x4 *pos)
{
  double p[3]{-para.Offset, 0, para.Offset};

  vtkSmartPointer<vtkTransform> transform = vtkTransform::New();
  transform->SetMatrix(pos);
  transform->Update();
  double *p_transed = transform->TransformDoublePoint(p);

  return mitk::Point3D{p_transed};
}

void VCView::TransferItkTransformToVtkMatrix(mitk::AffineTransform3D *itkTransform, vtkMatrix4x4 *vtkmatrix)
{
  int i, j;
  for (i = 0; i < 3; ++i)
    for (j = 0; j < 3; ++j)
      vtkmatrix->SetElement(i, j, itkTransform->GetMatrix().GetVnlMatrix().get(i, j));
  for (i = 0; i < 3; ++i)
    vtkmatrix->SetElement(i, 3, itkTransform->GetOffset()[i]);
  for (i = 0; i < 3; ++i)
    vtkmatrix->SetElement(3, i, 0.0);
  vtkmatrix->SetElement(3, 3, 1);
}

std::vector<double> VCView::vtkmatrix2angle(vtkMatrix4x4 *pMatrix)
{
  double sy = (double)sqrt(pMatrix->GetElement(0, 0) * pMatrix->GetElement(0, 0) +
                           pMatrix->GetElement(1, 0) * pMatrix->GetElement(1, 0));
  bool singular = sy < 1e-6; // If
  double x, y, z;
  if (!singular)
  {
    x = (float)atan2(pMatrix->GetElement(2, 1), pMatrix->GetElement(2, 2));
    y = (float)atan2(-pMatrix->GetElement(2, 0), sy);
    z = (float)atan2(pMatrix->GetElement(1, 0), pMatrix->GetElement(0, 0));
  }
  else
  {
    x = (float)atan2(-pMatrix->GetElement(1, 2), pMatrix->GetElement(1, 1));
    y = (float)atan2(-pMatrix->GetElement(2, 0), sy);
    z = 0;
  }
  std::vector<double> i;
  i.push_back((double)(x * (180.0f / M_PI)));
  i.push_back((double)(y * (180.0f / M_PI)));
  i.push_back((double)(z * (180.0f / M_PI)));
  return i;
}

void VCView::on_widget_plane_distal_Changed(QmitkSingleNodeSelectionWidget::NodeList)
{
  m_distalCut = m_Controls.widget_plane_distal->GetSelectedNode();
}

void VCView::on_widget_plane_backOblique_Changed(QmitkSingleNodeSelectionWidget::NodeList)
{
  m_backObliCut = m_Controls.widget_plane_backOblique->GetSelectedNode();
}

void VCView::on_widget_plane_back_Changed(QmitkSingleNodeSelectionWidget::NodeList)
{
  m_backCut = m_Controls.widget_plane_back->GetSelectedNode();
}

void VCView::on_widget_plane_frontOblique_Changed(QmitkSingleNodeSelectionWidget::NodeList)
{
  m_frontObliCut = m_Controls.widget_plane_frontOblique->GetSelectedNode();
}

void VCView::on_widget_plane_front_Changed(QmitkSingleNodeSelectionWidget::NodeList)
{
  m_frontCut = m_Controls.widget_plane_front->GetSelectedNode();
}

void VCView::OnpushButton_calNormalPlaneRMS() {}

void VCView::OnpushButtonCalAngle() {}

void VCView::padImage()
{
  auto padImgFilter = mitk::PadImageFilter::New();
  auto image1 = GetDataStorage()->GetNamedObject<mitk::Image>("src");
  auto image2 = GetDataStorage()->GetNamedObject<mitk::Image>("pad");

  if (image1 == nullptr || image2 == nullptr)
  {
    MITK_ERROR << "pad null";
    return;
  }
  padImgFilter->SetInput(0, image1);
  padImgFilter->SetInput(1, image2);
  // padImgFilter->SetBinaryFilter(true);
  padImgFilter->SetPadConstant(0);
  padImgFilter->Update();

  auto newNode = mitk::DataNode::New();
  newNode->SetName("pelvis_overlay_right");
  newNode->SetData(padImgFilter->GetOutput());

  GetDataStorage()->Add(newNode);
}

void VCView::SetupPolish()
{
  auto boneImage = GetDataStorage()->GetNamedObject<mitk::Image>("boneImage");
  if (boneImage == nullptr)
  {
    MITK_ERROR << "boneImage null";
    return;
  }
  auto toolSurface = GetDataStorage()->GetNamedObject<mitk::Surface>("tool");
  if (toolSurface == nullptr)
  {
    MITK_ERROR << "toolSurface null";
    return;
  }
  auto boneSurface = GetDataStorage()->GetNamedObject<mitk::Surface>("boneSurface");
  if (boneSurface == nullptr)
  {
    MITK_ERROR << "boneSurface null";
    return;
  }
  if (m_polish.IsNull())
  {
    m_polish = Polish::New();
  }

  m_polish->SetboneImage(boneImage);
  m_polish->SettoolSurface(toolSurface);
  m_polish->SetboneSurface(boneSurface);

  //for showing the polish result
  auto resnode = GetDataStorage()->GetNamedNode("BoneSurfacePolished");
  if (resnode == nullptr)
  {
    mitk::DataNode::Pointer resnode = mitk::DataNode::New();
    resnode->SetData(m_polish->GetboneSurface_polished());
    resnode->SetName("BoneSurfacePolished");

    GetDataStorage()->Add(resnode);
  }
  else
  {
    resnode->SetData(m_polish->GetboneImage_polished());
  }
}

void VCView::PolishOnce()
{
  m_polish->PolishWorkflow();

  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::StartPolish()
{

  
  // vtkNew<vtkLinearSubdivisionFilter> subdivision;
  // subdivision->SetInputData(toolSurface->GetVtkPolyData());
  // subdivision->SetNumberOfSubdivisions(1);
  // subdivision->Update();
  // toolSurface->SetVtkPolyData(subdivision->GetOutput());
  
  
  //m_polish->PolishWorkflow();
  m_polish->StartPolishing();

  
  //
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::StopPolishing()
{
  if (m_polish.IsNotNull())
  {
    m_polish ->StopPolishing();
  }
}

void VCView::RestorePolish()
{
  if (m_polish.IsNotNull())
  {
    m_polish->RestorePolish();
  }
}

void VCView::OnPushButtonApplyTrans()
{
  MITK_INFO << "OnPushButtonApplyTrans";
  if (m_targetPset == nullptr || m_sourcePset == nullptr)
  {
    m_Controls.textBrowser_trans_info->append("select source and target pointset first");
    return;
  }
  auto sourcePointset = dynamic_cast<mitk::PointSet *>(m_sourcePset->GetData());
  auto targetPointset = dynamic_cast<mitk::PointSet *>(m_targetPset->GetData());
  if (sourcePointset->GetSize() != targetPointset->GetSize())
  {
    m_Controls.textBrowser_trans_info->append("Failed: the Size of source and target pointset should be same");
    return;
  }

  vtkSmartPointer<vtkPoints> sourcePoints = vtkPoints::New();
  vtkSmartPointer<vtkPoints> targetPoints = vtkPoints::New();

  for (int i = 0; i < sourcePointset->GetSize(); i++)
  {
    targetPoints->InsertPoint(i, targetPointset->GetPoint(i).GetDataPointer());
    sourcePoints->InsertPoint(i, sourcePointset->GetPoint(i).GetDataPointer());
  }

  vtkSmartPointer<vtkLandmarkTransform> transform = vtkLandmarkTransform::New();

  transform->SetSourceLandmarks(sourcePoints);
  transform->SetTargetLandmarks(targetPoints);
  transform->SetMode(VTK_LANDMARK_RIGIDBODY);
  transform->Update();

  std::ostringstream os;
  transform->Print(os);
  m_Controls.textBrowser_trans_info->append(QString::fromStdString(os.str()));

  if (m_movingNode != nullptr)
  {
    vtkTransform *trans = vtkTransform::New();
    trans->SetMatrix(m_movingNode->GetData()->GetGeometry()->GetVtkMatrix());
    trans->Concatenate(transform->GetMatrix());
    trans->Update();
    mitk::Point3D ref;
    auto *doOp = new mitk::ApplyTransformMatrixOperation(mitk::OpAPPLYTRANSFORMMATRIX, trans->GetMatrix(), ref);
    // execute the Operation
    // here no undo is stored, because the movement-steps aren't interesting.
    // only the start and the end is interisting to store for undo.
    m_movingNode->GetData()->GetGeometry()->ExecuteOperation(doOp);
    delete doOp;

    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
  }
}

void VCView::OnpushButton_PLC()
{
  MITK_INFO << "OnpushButton_PLC";
  auto m_node = GetDataStorage()->GetNamedObject<mitk::PointSet>("VerifyCOR_left");
  double radius = 0;
  if (m_node != nullptr)
  {
    mitk::Vector3D temp{this->CalCOR(m_node, radius)};
    double position[3]{temp[0], temp[1], temp[2]};
    mitk::PointSet::Pointer point = GetDataStorage()->GetNamedObject<mitk::PointSet>("pelvisCOR");
    if (point == nullptr)
    {
      point = mitk::PointSet::New();
      point->InsertPoint(0, position);
      point->InsertPoint(1, position);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("pelvisCOR");
      pNode->SetData(point);
      pNode->SetFloatProperty("pointsize", 5);
      pNode->SetColor(1, 0, 1);
      pNode->SetFloatProperty("point 2D size", 8);
      pNode->SetProperty("Pointset.2D.shape", mitk::PointSetShapeProperty::New(7));
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }
    else
    {
      point->SetPoint(1, position);
    }
  }
  m_Controls.lineEdit_PLC->setText("Radius:" + QString::number(radius));
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::OnpushButton_PRC()
{
  MITK_INFO << "OnpushButton_PRC";
  auto m_node = GetDataStorage()->GetNamedObject<mitk::PointSet>("VerifyCOR_right");
  double radius = 0;
  if (m_node != nullptr)
  {
    mitk::Vector3D temp{this->CalCOR(m_node, radius)};
    double position[3]{temp[0], temp[1], temp[2]};
    mitk::PointSet::Pointer point = GetDataStorage()->GetNamedObject<mitk::PointSet>("pelvisCOR");
    if (point == nullptr)
    {
      point = mitk::PointSet::New();
      point->InsertPoint(0, position);
      point->InsertPoint(1, position);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("pelvisCOR");
      pNode->SetData(point);
      pNode->SetFloatProperty("pointsize", 5);
      pNode->SetColor(1, 0, 1);
      pNode->SetFloatProperty("point 2D size", 8);
      pNode->SetProperty("Pointset.2D.shape", mitk::PointSetShapeProperty::New(7));
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }
    else
    {
      point->SetPoint(0, position);
    }
  }
  m_Controls.lineEdit_PRC->setText("Radius:" + QString::number(radius));
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::OnpushButton_FLC()
{
  MITK_INFO << "OnpushButton_FLC";
  auto m_node = GetDataStorage()->GetNamedObject<mitk::PointSet>("CalFemurCOR_left");
  double radius = 0;
  if (m_node != nullptr)
  {
    mitk::Vector3D temp{this->CalCOR(m_node, radius)};
    double position[3]{temp[0], temp[1], temp[2]};
    mitk::PointSet::Pointer pointCor = GetDataStorage()->GetNamedObject<mitk::PointSet>("femurCOR_left");
    if (pointCor == nullptr)
    {
      pointCor = mitk::PointSet::New();
      pointCor->InsertPoint(0, position);
      mitk::DataNode::Pointer pNodeCor = mitk::DataNode::New();
      pNodeCor->SetName("femurCOR_left");
      pNodeCor->SetData(pointCor);
      pNodeCor->SetFloatProperty("pointsize", 5);
      pNodeCor->SetVisibility(true);
      GetDataStorage()->Add(pNodeCor);
    }
    else
      pointCor->SetPoint(0, position);
  }
  m_Controls.lineEdit_FLC->setText("Radius:" + QString::number(radius));
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::OnpushButton_FRC()
{
  MITK_INFO << "OnpushButton_FRC";
  auto m_node = GetDataStorage()->GetNamedObject<mitk::PointSet>("CalFemurCOR_right");
  double radius = 0;
  if (m_node != nullptr)
  {
    mitk::Vector3D temp{this->CalCOR(m_node, radius)};
    double position[3]{temp[0], temp[1], temp[2]};
    mitk::PointSet::Pointer pointCor = GetDataStorage()->GetNamedObject<mitk::PointSet>("femurCOR_right");
    if (pointCor == nullptr)
    {
      pointCor = mitk::PointSet::New();
      pointCor->InsertPoint(0, position);
      mitk::DataNode::Pointer pNodeCor = mitk::DataNode::New();
      pNodeCor->SetName("femurCOR_right");
      pNodeCor->SetData(pointCor);
      pNodeCor->SetFloatProperty("pointsize", 5);
      pNodeCor->SetVisibility(true);
      GetDataStorage()->Add(pNodeCor);
    }
    else
      pointCor->SetPoint(0, position);
  }
  m_Controls.lineEdit_FRC->setText("Radius:" + QString::number(radius));
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

mitk::Vector3D VCView::CalCOR(mitk::PointSet *inp_pointset, double &radius)
{
  Eigen::Vector4d Y;
  Eigen::Matrix4d X;
  Y.setZero();
  X.setZero();
  int nCount = inp_pointset->GetSize();
  MITK_INFO << "nCount" << nCount << endl;

  for (int i = 0; i < nCount; i++)
  {
    Y[0] += pow(inp_pointset->GetPoint(i)[0], 3) + pow(inp_pointset->GetPoint(i)[1], 2) * inp_pointset->GetPoint(i)[0] +
            pow(inp_pointset->GetPoint(i)[2], 2) * inp_pointset->GetPoint(i)[0];
    Y[1] += pow(inp_pointset->GetPoint(i)[0], 2) * inp_pointset->GetPoint(i)[1] + pow(inp_pointset->GetPoint(i)[1], 3) +
            pow(inp_pointset->GetPoint(i)[2], 2) * inp_pointset->GetPoint(i)[1];
    Y[2] += pow(inp_pointset->GetPoint(i)[0], 2) * inp_pointset->GetPoint(i)[2] +
            pow(inp_pointset->GetPoint(i)[1], 2) * inp_pointset->GetPoint(i)[2] + pow(inp_pointset->GetPoint(i)[2], 3);
    Y[3] -= pow(inp_pointset->GetPoint(i)[0], 2) + pow(inp_pointset->GetPoint(i)[1], 2) +
            pow(inp_pointset->GetPoint(i)[2], 2);

    X(0, 0) += pow(inp_pointset->GetPoint(i)[0], 2);
    X(0, 1) += inp_pointset->GetPoint(i)[0] * inp_pointset->GetPoint(i)[1];
    X(0, 2) += inp_pointset->GetPoint(i)[0] * inp_pointset->GetPoint(i)[2];
    X(0, 3) -= inp_pointset->GetPoint(i)[0];
    X(1, 1) += pow(inp_pointset->GetPoint(i)[1], 2);
    X(1, 2) += inp_pointset->GetPoint(i)[1] * inp_pointset->GetPoint(i)[2];
    X(1, 3) -= inp_pointset->GetPoint(i)[1];
    X(2, 2) += pow(inp_pointset->GetPoint(i)[2], 2);
    X(2, 3) -= inp_pointset->GetPoint(i)[2];
  }
  //对称矩阵
  X(1, 0) = X(0, 1);
  X(2, 0) = X(0, 2);
  X(2, 1) = X(1, 2);
  X(3, 0) = X(0, 3);
  X(3, 1) = X(1, 3);
  X(3, 2) = X(2, 3);
  X(3, 3) = nCount;
  //判断是否可逆
  double dX = 0.0, dY = 0.0, dZ = 0.0, dD = 0.0;
  double dRadius = 0.0;
  if (X.determinant() != 0)
  {
    Eigen::Matrix4d XI = X.inverse();
    for (int ni = 0; ni < XI.RowsAtCompileTime; ni++)
    {
      dX += XI(0, ni) * Y[ni];
      dY += XI(1, ni) * Y[ni];
      dZ += XI(2, ni) * Y[ni];
      dD += XI(3, ni) * Y[ni];
    }
    dX = dX / 2.0;
    dY = dY / 2.0;
    dZ = dZ / 2.0;
    dRadius = pow(dX, 2) + pow(dY, 2) + pow(dZ, 2) - dD;
    dRadius = pow(dRadius, 0.5);
  }
  MITK_INFO << "X :" << dX << endl
            << "Y :" << dY << endl
            << "Z :" << dZ << endl
            << "D:" << dD << endl
            << "R :" << dRadius << endl;
  radius = dRadius;
  double temp[3]{dX, dY, dZ};
  mitk::Vector3D cor{temp};
  return cor;
}

void VCView::OnpushButton_Updata()
{
  MITK_INFO << "OnpushButton_Updata";

  if (m_asis != nullptr)
  {
    auto asisNode = GetDataStorage()->GetNamedNode("ASIS");
    asisNode->SetFloatProperty("pointsize", 8);
    asisNode->SetBoolProperty("show contour", true);
    asisNode->SetProperty("contourcolor", mitk::ColorProperty::New(1, 1, 0));
  }

  double m_pmtemp0[3]{m_pt->GetPoint(0)[0], m_asis->GetPoint(0)[1], m_asis->GetPoint(0)[2]};
  double m_pmtemp1[3]{m_pt->GetPoint(0)[0], m_asis->GetPoint(0)[1], m_asis->GetPoint(0)[2] - 100};
  mitk::PointSet::Pointer m_pm = GetDataStorage()->GetNamedObject<mitk::PointSet>("pelvis_midline");
  if (m_pm == nullptr)
  {
    m_pm = mitk::PointSet::New();
    m_pm->InsertPoint(0, m_pmtemp0);
    m_pm->InsertPoint(1, m_pmtemp1);
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("pelvis_midline");
    pNode->SetData(m_pm);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetBoolProperty("show contour", true);
    pNode->SetBoolProperty("show points", false);
    pNode->SetProperty("contourcolor", mitk::ColorProperty::New(1, 1, 0));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  else
  {
    m_pm->SetPoint(0, m_pmtemp0);
    m_pm->SetPoint(1, m_pmtemp1);
  }

  double m_lltemp0[3]{m_trochanterL->GetPoint(0)[0], m_asis->GetPoint(0)[1], m_asis->GetPoint(0)[2]};
  double m_lltemp1[3]{m_trochanterL->GetPoint(0)[0], m_asis->GetPoint(0)[1], m_trochanterL->GetPoint(0)[2]};
  mitk::PointSet::Pointer m_ll = GetDataStorage()->GetNamedObject<mitk::PointSet>("hip_length_left");
  if (m_ll == nullptr)
  {
    m_ll = mitk::PointSet::New();
    m_ll->InsertPoint(0, m_lltemp0);
    m_ll->InsertPoint(1, m_lltemp1);
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("hip_length_left");
    pNode->SetData(m_ll);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetBoolProperty("show contour", true);
    pNode->SetBoolProperty("show points", false);
    pNode->SetProperty("contourcolor", mitk::ColorProperty::New(1, 1, 0));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  else
  {
    m_ll->SetPoint(0, m_lltemp0);
    m_ll->SetPoint(1, m_lltemp1);
  }

  double m_lrtemp0[3]{m_trochanterR->GetPoint(0)[0], m_asis->GetPoint(0)[1], m_asis->GetPoint(0)[2]};
  double m_lrtemp1[3]{m_trochanterR->GetPoint(0)[0], m_asis->GetPoint(0)[1], m_trochanterR->GetPoint(0)[2]};
  mitk::PointSet::Pointer m_lr = GetDataStorage()->GetNamedObject<mitk::PointSet>("hip_length_right");
  if (m_lr == nullptr)
  {
    m_lr = mitk::PointSet::New();
    m_lr->InsertPoint(0, m_lrtemp0);
    m_lr->InsertPoint(1, m_lrtemp1);
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("hip_length_right");
    pNode->SetData(m_lr);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetBoolProperty("show contour", true);
    pNode->SetBoolProperty("show points", false);
    pNode->SetProperty("contourcolor", mitk::ColorProperty::New(1, 1, 0));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  else
  {
    m_lr->SetPoint(0, m_lrtemp0);
    m_lr->SetPoint(1, m_lrtemp1);
  }

  double m_trotemp0[3]{m_trochanterR->GetPoint(0)[0], m_trochanterR->GetPoint(0)[1], m_trochanterR->GetPoint(0)[2]};
  double m_trotemp1[3]{m_trochanterL->GetPoint(0)[0], m_trochanterL->GetPoint(0)[1], m_trochanterL->GetPoint(0)[2]};
  mitk::PointSet::Pointer m_tro = GetDataStorage()->GetNamedObject<mitk::PointSet>("trochanter");
  if (m_tro == nullptr)
  {
    m_tro = mitk::PointSet::New();
    m_tro->InsertPoint(0, m_trotemp0);
    m_tro->InsertPoint(1, m_trotemp1);
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("trochanter");
    pNode->SetData(m_tro);
    pNode->SetFloatProperty("pointsize", 6);
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  else
  {
    m_tro->SetPoint(0, m_trotemp0);
    m_tro->SetPoint(1, m_trotemp1);
  }

  double m_fcortemp0[3]{m_fhcR->GetPoint(0)[0], m_fhcR->GetPoint(0)[1], m_fhcR->GetPoint(0)[2]};
  double m_fcortemp1[3]{m_fhcL->GetPoint(0)[0], m_fhcL->GetPoint(0)[1], m_fhcL->GetPoint(0)[2]};
  mitk::PointSet::Pointer m_fcor = GetDataStorage()->GetNamedObject<mitk::PointSet>("femurCOR");
  if (m_fcor == nullptr)
  {
    m_fcor = mitk::PointSet::New();
    m_fcor->InsertPoint(0, m_fcortemp0);
    m_fcor->InsertPoint(1, m_fcortemp1);
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("femurCOR");
    pNode->SetData(m_fcor);
    pNode->SetFloatProperty("pointsize", 6);
    pNode->SetColor(0, 0, 1);
    pNode->SetFloatProperty("point 2D size", 8);
    pNode->SetProperty("Pointset.2D.shape", mitk::PointSetShapeProperty::New(7));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  else
  {
    m_fcor->SetPoint(0, m_fcortemp0);
    m_fcor->SetPoint(1, m_fcortemp1);
  }

  mitk::PointSet::Pointer m_pcor = GetDataStorage()->GetNamedObject<mitk::PointSet>("pelvisCOR");
  double m_clrtemp0[3]{m_pcor->GetPoint(0)[0], m_pcor->GetPoint(0)[1], m_pcor->GetPoint(0)[2]};
  double m_clrtemp1[3]{m_pcor->GetPoint(0)[0] - 40, m_pcor->GetPoint(0)[1], m_pcor->GetPoint(0)[2]};
  double m_clrtemp2[3]{m_pcor->GetPoint(0)[0], m_pcor->GetPoint(0)[1], m_pcor->GetPoint(0)[2] + 20};
  mitk::PointSet::Pointer m_clr = GetDataStorage()->GetNamedObject<mitk::PointSet>("CupLine_right");
  if (m_clr == nullptr)
  {
    m_clr = mitk::PointSet::New();
    m_clr->InsertPoint(0, m_clrtemp0);
    m_clr->InsertPoint(1, m_clrtemp1);
    m_clr->InsertPoint(2, m_clrtemp2);
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("CupLine_right");
    pNode->SetData(m_clr);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  else
  {
    m_clr->SetPoint(0, m_clrtemp0);
    m_clr->SetPoint(1, m_clrtemp1);
    m_clr->SetPoint(2, m_clrtemp2);
  }

  double m_clltemp0[3]{m_pcor->GetPoint(1)[0], m_pcor->GetPoint(1)[1], m_pcor->GetPoint(1)[2]};
  double m_clltemp1[3]{m_pcor->GetPoint(1)[0] + 40, m_pcor->GetPoint(1)[1], m_pcor->GetPoint(1)[2]};
  double m_clltemp2[3]{m_pcor->GetPoint(1)[0], m_pcor->GetPoint(1)[1], m_pcor->GetPoint(1)[2] + 20};
  mitk::PointSet::Pointer m_cll = GetDataStorage()->GetNamedObject<mitk::PointSet>("CupLine_left");
  if (m_cll == nullptr)
  {
    m_cll = mitk::PointSet::New();
    m_cll->InsertPoint(0, m_clltemp0);
    m_cll->InsertPoint(1, m_clltemp1);
    m_cll->InsertPoint(2, m_clltemp2);
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("CupLine_left");
    pNode->SetData(m_cll);
    pNode->SetFloatProperty("pointsize", 5);
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  else
  {
    m_cll->SetPoint(0, m_clltemp0);
    m_cll->SetPoint(1, m_clltemp1);
    m_cll->SetPoint(2, m_clltemp2);
  }

  double m_rltemp0[3]{0, 0, 0};
  double m_rltemp1[3]{0, 0, -20};
  mitk::PointSet::Pointer m_rl = GetDataStorage()->GetNamedObject<mitk::PointSet>("RealLine");
  if (m_rl == nullptr)
  {
    m_rl = mitk::PointSet::New();
    m_rl->InsertPoint(0, m_rltemp0);
    m_rl->InsertPoint(1, m_rltemp1);
    mitk::DataNode::Pointer pNode = mitk::DataNode::New();
    pNode->SetName("RealLine");
    pNode->SetData(m_rl);
    pNode->SetFloatProperty("pointsize", 4);
    pNode->SetColor(0, 0, 1);
    pNode->SetBoolProperty("show contour", true);
    pNode->SetProperty("contourcolor", mitk::ColorProperty::New(0, 0, 1));
    pNode->SetVisibility(true);
    GetDataStorage()->Add(pNode);
  }
  else
  {
    m_rl->SetPoint(0, m_rltemp0);
    m_rl->SetPoint(1, m_rltemp1);
  }

  mitk::DataNode::Pointer pNode1 = this->GetDataStorage()->GetNamedNode("PT");
  if (pNode1 != nullptr)
  {
    this->GetDataStorage()->Remove(pNode1);
  }

  mitk::DataNode::Pointer pNode2 = this->GetDataStorage()->GetNamedNode("CalFemurCOR_right");
  if (pNode2 != nullptr)
  {
    this->GetDataStorage()->Remove(pNode2);
  }

  mitk::DataNode::Pointer pNode3 = this->GetDataStorage()->GetNamedNode("CalFemurCOR_left");
  if (pNode3 != nullptr)
  {
    this->GetDataStorage()->Remove(pNode3);
  }

  mitk::DataNode::Pointer pNode4 = this->GetDataStorage()->GetNamedNode("trochanter_left");
  if (pNode4 != nullptr)
  {
    this->GetDataStorage()->Remove(pNode4);
  }

  mitk::DataNode::Pointer pNode5 = this->GetDataStorage()->GetNamedNode("trochanter_right");
  if (pNode5 != nullptr)
  {
    this->GetDataStorage()->Remove(pNode5);
  }

  mitk::DataNode::Pointer pNode6 = this->GetDataStorage()->GetNamedNode("femurCOR_left");
  if (pNode6 != nullptr)
  {
    this->GetDataStorage()->Remove(pNode6);
  }

  mitk::DataNode::Pointer pNode7 = this->GetDataStorage()->GetNamedNode("femurCOR_right");
  if (pNode7 != nullptr)
  {
    this->GetDataStorage()->Remove(pNode7);
  }

  if (THA_MODEL.GetOprationSide() == ESide::left)
  {
    mitk::DataNode::Pointer pNode8 = this->GetDataStorage()->GetNamedNode("VerifyCOR_right");
    if (pNode8 != nullptr)
    {
      this->GetDataStorage()->Remove(pNode8);
    }

    mitk::DataNode::Pointer pNode9 = this->GetDataStorage()->GetNamedNode("CupLine_right");
    if (pNode9 != nullptr)
    {
      this->GetDataStorage()->Remove(pNode9);
    }

    mitk::DataNode::Pointer pNode10 = this->GetDataStorage()->GetNamedNode("VerifyCOR_left");
    pNode10->SetColor(0, 0, 1);
    pNode10->SetProperty("selectedcolor", mitk::ColorProperty::New(1, 1, 1));

	mitk::DataNode::Pointer pNode11 = this->GetDataStorage()->GetNamedNode("srcimage");
    if (pNode11 != nullptr)
    {
      this->GetDataStorage()->Remove(pNode11);
    }
    mitk::DataNode::Pointer pNode12 = this->GetDataStorage()->GetNamedNode("pelvis_overlay");
    if (pNode12 != nullptr)
	{
      this->GetDataStorage()->Remove(pNode12);
	}
    mitk::DataNode::Pointer pNode13 = this->GetDataStorage()->GetNamedNode("pelvis_overlay_side");
	if (pNode13 != nullptr)
	{
		pNode13->SetName("pelvis_overlay_left");
	}

    mitk::PointSet::Pointer point0 = GetDataStorage()->GetNamedObject<mitk::PointSet>("VerifyCOR_left");
    for (int i = point0->GetSize(); i > 8; i--)
    {
      point0->RemovePointAtEnd();
    }

    double m_temp[3]{0, 0, 0};
    mitk::PointSet::Pointer m_posterior = GetDataStorage()->GetNamedObject<mitk::PointSet>("posterior_landmark_left");
    if (m_posterior == nullptr)
    {
      m_posterior = mitk::PointSet::New();
      m_posterior->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("posterior_landmark_left");
      pNode->SetData(m_posterior);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_anterior = GetDataStorage()->GetNamedObject<mitk::PointSet>("anterior_landmark_left");
    if (m_anterior == nullptr)
    {
      m_anterior = mitk::PointSet::New();
      m_anterior->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("anterior_landmark_left");
      pNode->SetData(m_anterior);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_superior = GetDataStorage()->GetNamedObject<mitk::PointSet>("superior_landmark_left");
    if (m_superior == nullptr)
    {
      m_superior = mitk::PointSet::New();
      m_superior->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("superior_landmark_left");
      pNode->SetData(m_superior);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_other = GetDataStorage()->GetNamedObject<mitk::PointSet>("other_landmark_left");
    if (m_other == nullptr)
    {
      m_other = mitk::PointSet::New();
      m_other->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("other_landmark_left");
      pNode->SetData(m_other);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_registration =
      GetDataStorage()->GetNamedObject<mitk::PointSet>("registration_points_left");
    if (m_registration == nullptr)
    {
      m_registration = mitk::PointSet::New();
      m_registration->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("registration_points_left");
      pNode->SetData(m_registration);
      pNode->SetColor(1, 1, 1);
      pNode->SetProperty("selectedcolor", mitk::ColorProperty::New(0, 0, 1));
      pNode->SetFloatProperty("pointsize", 4);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_saddle = GetDataStorage()->GetNamedObject<mitk::PointSet>("saddle_landmark_left");
    if (m_saddle == nullptr)
    {
      m_saddle = mitk::PointSet::New();
      m_saddle->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("saddle_landmark_left");
      pNode->SetData(m_saddle);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_lower = GetDataStorage()->GetNamedObject<mitk::PointSet>("lower_landmark_left");
    if (m_lower == nullptr)
    {
      m_lower = mitk::PointSet::New();
      m_lower->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("lower_landmark_left");
      pNode->SetData(m_lower);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_trochanterF = GetDataStorage()->GetNamedObject<mitk::PointSet>("trochanter_landmark_left");
    if (m_trochanterF == nullptr)
    {
      m_trochanterF = mitk::PointSet::New();
      m_trochanterF->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("trochanter_landmark_left");
      pNode->SetData(m_trochanterF);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_femurregistration =
      GetDataStorage()->GetNamedObject<mitk::PointSet>("femurregistration_points_left");
    if (m_femurregistration == nullptr)
    {
      m_femurregistration = mitk::PointSet::New();
      m_femurregistration->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("femurregistration_points_left");
      pNode->SetData(m_femurregistration);
      pNode->SetColor(1, 1, 1);
      pNode->SetProperty("selectedcolor", mitk::ColorProperty::New(0, 0, 1));
      pNode->SetFloatProperty("pointsize", 4);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

	    mitk::PointSet::Pointer m_verifyF =
      GetDataStorage()->GetNamedObject<mitk::PointSet>("VerifyFemur_left");
    if (m_verifyF == nullptr)
    {
      m_verifyF = mitk::PointSet::New();
      m_verifyF->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("VerifyFemur_left");
      pNode->SetData(m_verifyF);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 5);
      pNode->SetProperty("selectedcolor", mitk::ColorProperty::New(1, 1, 1));
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }
  }
  else
  {
    mitk::DataNode::Pointer pNode8 = this->GetDataStorage()->GetNamedNode("CupLine_left");
    this->GetDataStorage()->Remove(pNode8);
    mitk::DataNode::Pointer pNode9 = this->GetDataStorage()->GetNamedNode("VerifyCOR_left");
    this->GetDataStorage()->Remove(pNode9);

    mitk::DataNode::Pointer pNode10 = this->GetDataStorage()->GetNamedNode("VerifyCOR_right");
    pNode10->SetColor(0, 0, 1);
    pNode10->SetProperty("selectedcolor", mitk::ColorProperty::New(1, 1, 1));

	mitk::DataNode::Pointer pNode11 = this->GetDataStorage()->GetNamedNode("srcimage");
    if (pNode11 != nullptr)
    {
      this->GetDataStorage()->Remove(pNode11);
    }
    mitk::DataNode::Pointer pNode12 = this->GetDataStorage()->GetNamedNode("pelvis_overlay");
    if (pNode12 != nullptr)
    {
      this->GetDataStorage()->Remove(pNode12);
    }
    mitk::DataNode::Pointer pNode13 = this->GetDataStorage()->GetNamedNode("pelvis_overlay_side");
    if (pNode13 != nullptr)
    {
      pNode13->SetName("pelvis_overlay_right");
    }

    mitk::PointSet::Pointer point0 = GetDataStorage()->GetNamedObject<mitk::PointSet>("VerifyCOR_right");
    for (int i = point0->GetSize(); i > 8; i--)
    {
      point0->RemovePointAtEnd();
    }

    mitk::PointSet::Pointer m_clr = GetDataStorage()->GetNamedObject<mitk::PointSet>("CupLine_right");
    if (m_clr == nullptr)
    {
      m_clr = mitk::PointSet::New();
      m_clr->InsertPoint(0, m_clrtemp0);
      m_clr->InsertPoint(1, m_clrtemp1);
      m_clr->InsertPoint(2, m_clrtemp2);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("CupLine_right");
      pNode->SetData(m_clr);
      pNode->SetFloatProperty("pointsize", 5);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    double m_temp[3]{0, 0, 0};
    mitk::PointSet::Pointer m_posterior = GetDataStorage()->GetNamedObject<mitk::PointSet>("posterior_landmark_right");
    if (m_posterior == nullptr)
    {
      m_posterior = mitk::PointSet::New();
      m_posterior->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("posterior_landmark_right");
      pNode->SetData(m_posterior);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_anterior = GetDataStorage()->GetNamedObject<mitk::PointSet>("anterior_landmark_right");
    if (m_anterior == nullptr)
    {
      m_anterior = mitk::PointSet::New();
      m_anterior->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("anterior_landmark_right");
      pNode->SetData(m_anterior);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_superior = GetDataStorage()->GetNamedObject<mitk::PointSet>("superior_landmark_right");
    if (m_superior == nullptr)
    {
      m_superior = mitk::PointSet::New();
      m_superior->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("superior_landmark_right");
      pNode->SetData(m_superior);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_other = GetDataStorage()->GetNamedObject<mitk::PointSet>("other_landmark_right");
    if (m_other == nullptr)
    {
      m_other = mitk::PointSet::New();
      m_other->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("other_landmark_right");
      pNode->SetData(m_other);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_registration =
      GetDataStorage()->GetNamedObject<mitk::PointSet>("registration_points_right");
    if (m_registration == nullptr)
    {
      m_registration = mitk::PointSet::New();
      m_registration->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("registration_points_right");
      pNode->SetData(m_registration);
      pNode->SetColor(1, 1, 1);
      pNode->SetProperty("selectedcolor", mitk::ColorProperty::New(0, 0, 1));
      pNode->SetFloatProperty("pointsize", 4);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

	    mitk::PointSet::Pointer m_saddle = GetDataStorage()->GetNamedObject<mitk::PointSet>("saddle_landmark_right");
    if (m_saddle == nullptr)
    {
      m_saddle = mitk::PointSet::New();
      m_saddle->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("saddle_landmark_right");
      pNode->SetData(m_saddle);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_lower = GetDataStorage()->GetNamedObject<mitk::PointSet>("lower_landmark_right");
    if (m_lower == nullptr)
    {
      m_lower = mitk::PointSet::New();
      m_lower->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("lower_landmark_right");
      pNode->SetData(m_lower);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_trochanterF =
      GetDataStorage()->GetNamedObject<mitk::PointSet>("trochanter_landmark_right");
    if (m_trochanterF == nullptr)
    {
      m_trochanterF = mitk::PointSet::New();
      m_trochanterF->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("trochanter_landmark_right");
      pNode->SetData(m_trochanterF);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 8);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_femurregistration =
      GetDataStorage()->GetNamedObject<mitk::PointSet>("femurregistration_points_right");
    if (m_femurregistration == nullptr)
    {
      m_femurregistration = mitk::PointSet::New();
      m_femurregistration->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("femurregistration_points_right");
      pNode->SetData(m_femurregistration);
      pNode->SetColor(1, 1, 1);
      pNode->SetProperty("selectedcolor", mitk::ColorProperty::New(0, 0, 1));
      pNode->SetFloatProperty("pointsize", 4);
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

    mitk::PointSet::Pointer m_verifyF = GetDataStorage()->GetNamedObject<mitk::PointSet>("VerifyFemur_right");
    if (m_verifyF == nullptr)
    {
      m_verifyF = mitk::PointSet::New();
      m_verifyF->InsertPoint(0, m_temp);
      mitk::DataNode::Pointer pNode = mitk::DataNode::New();
      pNode->SetName("VerifyFemur_right");
      pNode->SetData(m_verifyF);
      pNode->SetColor(0, 0, 1);
      pNode->SetFloatProperty("pointsize", 5);
      pNode->SetProperty("selectedcolor", mitk::ColorProperty::New(1, 1, 1));
      pNode->SetVisibility(true);
      GetDataStorage()->Add(pNode);
    }

  }

  mitk::DataNode::Pointer pNodepelvis = this->GetDataStorage()->GetNamedNode("pelvis");
  if (pNodepelvis != nullptr)
  {
    pNodepelvis->SetOpacity(0.5);
    pNodepelvis->SetProperty("Depth Sorting", mitk::BoolProperty::New(true));
  }

  mitk::DataNode::Pointer pNodefemurL = this->GetDataStorage()->GetNamedNode("femur_left");
  if (pNodefemurL != nullptr)
  {
    pNodefemurL->SetOpacity(0.5);
    pNodefemurL->SetProperty("Depth Sorting", mitk::BoolProperty::New(true));
  }

  mitk::DataNode::Pointer pNodefemurR = this->GetDataStorage()->GetNamedNode("femur_right");
  if (pNodefemurR != nullptr)
  {
    pNodefemurR->SetOpacity(0.5);
    pNodefemurR->SetProperty("Depth Sorting", mitk::BoolProperty::New(true));
  }

  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void VCView::OnpushButton_FemoralVersionAngle()
{
  if (m_BtrochanterL == nullptr || m_BtrochanterR == nullptr || m_condylesL == nullptr || m_condylesR == nullptr)
  {
    MITK_ERROR << "OnpushButton_FemoralVersionAngle == nullptr";
    return;
  }
  // left
  mitk::Point3D LPFCA = m_canalAxisL->GetPoint(0);
  mitk::Point3D LDFCA = m_canalAxisL->GetPoint(1);
  mitk::Point3D LFHC = m_fhcL->GetPoint(0);
  mitk::Point3D LBT = m_BtrochanterL->GetPoint(0);
  mitk::Point3D LMC = m_condylesL->GetPoint(0); //内髁
  mitk::Point3D LLC = m_condylesL->GetPoint(1); //外髁

  double m_LPFCA[3]{LPFCA[0], LPFCA[1], LPFCA[2]}, m_LDFCA[3]{LDFCA[0], LDFCA[1], LDFCA[2]},
    m_LFHC[3]{LFHC[0], LFHC[1], LFHC[2]}, m_LBT[3]{LBT[0], LBT[1], LBT[2]}, m_LMC[3]{LMC[0], LMC[1], LMC[2]},
    m_LLC[3]{LLC[0], LLC[1], LLC[2]};
  double m_FemurangleL = lancetAlgorithm::FemoralVersionAngle(m_LMC, m_LLC, m_LBT, m_LFHC, m_LDFCA, m_LPFCA);
  m_Controls.lineEdit_FemoralVersionAngleL->setText("Left:" + QString::number(m_FemurangleL));

  // right
  mitk::Point3D RPFCA = m_canalAxisR->GetPoint(0);
  mitk::Point3D RDFCA = m_canalAxisR->GetPoint(1);
  mitk::Point3D RFHC = m_fhcR->GetPoint(0);
  mitk::Point3D RBT = m_BtrochanterR->GetPoint(0);
  mitk::Point3D RMC = m_condylesR->GetPoint(0); //内髁
  mitk::Point3D RLC = m_condylesR->GetPoint(1); //外髁

  double m_RPFCA[3]{RPFCA[0], RPFCA[1], RPFCA[2]}, m_RDFCA[3]{RDFCA[0], RDFCA[1], RDFCA[2]},
    m_RFHC[3]{RFHC[0], RFHC[1], RFHC[2]}, m_RBT[3]{RBT[0], RBT[1], RBT[2]}, m_RMC[3]{RMC[0], RMC[1], RMC[2]},
    m_RLC[3]{RLC[0], RLC[1], RLC[2]};
  double m_FemurangleR = lancetAlgorithm::FemoralVersionAngle(m_RMC, m_RLC, m_RBT, m_RFHC, m_RDFCA, m_RPFCA);
  m_Controls.lineEdit_FemoralVersionAngleR->setText("Right:" + QString::number(m_FemurangleR));
}

void VCView::OnpushButton_PelvisVersionAngle()
{
  MITK_INFO << "OnpushButton_PelvisVersionAngle";
  double pt_angle = 0;
  THA_MODEL.Pelvis()->GetResult(EResult::f_PT, pt_angle);
  MITK_INFO << pt_angle;
  m_Controls.lineEdit_pelvisAngle->setText("Angel:" + QString::number(pt_angle));
}

void VCView::rotateZm()
{
  if (m_surface_transform)
  {
    double axis[3]{0, 0, -1};
    rotate(m_surface_transform->GetGeometry()->GetCenter().GetDataPointer(), axis, m_surface_transform);
  }
}

void VCView::ClipFemur()
{
  MITK_INFO << "VCView::ClipFemur";
  mitk::Surface::Pointer m_femur = GetDataStorage()->GetNamedObject<mitk::Surface>("femur_left");

  vtkSmartPointer<vtkPlane> planeNew = vtkPlane::New();
  planeNew->SetOrigin(65.556, -135.381, 1194.230);
  planeNew->SetNormal(0, 0, 1);

  vtkClipPolyData *clipper = vtkClipPolyData::New(); //裁剪polydata
  clipper->SetInputData(m_femur->GetVtkPolyData());
  clipper->SetClipFunction(planeNew);  //!!!!很重要的一步，添加自定义隐函数
  clipper->GenerateClippedOutputOff(); // important to NOT generate normals data for clipped part
  clipper->InsideOutOn();
  clipper->SetValue(0.5);
  clipper->Update();

  m_femur->SetVtkPolyData(clipper->GetOutput());

  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}
