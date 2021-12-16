/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef VCView_h
#define VCView_h

#include "mitkImage.h"
#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_VCViewControls.h"

namespace mitk {
  class PointSet;
  class Surface;
}

typedef std::vector<mitk::BaseData*> OperateGroup;

class StemParameter
{
public:
  StemParameter(unsigned size_ref, double shaft_angle, double offset)
    : sizeRef(size_ref),
      shaftAngle(shaft_angle),
      Offset(offset)
  {
  }


  StemParameter() = default;
  unsigned int sizeRef;
    double shaftAngle;
    double Offset;
};
/**
  \brief VCView

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class VCView : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;

protected:
  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;
  void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);

  /// \brief called by QmitkFunctionality when DataManager's selection has changed
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
                                  const QList<mitk::DataNode::Pointer> &nodes) override;

  void CutImage();
  void AddPoint();
  void AutoDetect();
  void Initialize();
  /// \brief Called when the user clicks the GUI button
  void ApplyPelvicVC();

  void ApplyFemurVC();

  void SetOpSide(QString side);

  void BuildPelvis();

  void BuildFemurL();

  void BuildFemurR();

  void calRotation();

  void calMechanCorrectionL();

  void AdjustPelvisTilt(bool positive);

  void RoTationPelvisXAdd();
  void RoTationPelvisXSub();

  void SetRoTationPelvisXAddValue();
  void SetRoTationPelvisXSubValue();

  mitk::Image::Pointer SurfaceCutImage(mitk::Surface* surface, mitk::Image* image, bool isReverseStencil =false, bool isSmallestCut= false);
  void StlCutImage();
  void StlCutLeftImage();
  void StlCutRightImage();
  void StlCutOverlayImage();

  void CutLeftFemur();
  void CutRightFemur();

  void cutLFemur();
  void cutRFemur();

  // Build COR
  void OnpushButton_PLC();
  void OnpushButton_PRC();
  void OnpushButton_FLC();
  void OnpushButton_FRC();
  mitk::Vector3D CalCOR(mitk::PointSet *inp_pointset, double &radius);

  void OnpushButton_Updata();
  void OnpushButton_FemoralVersionAngle();
  void OnpushButton_PelvisVersionAngle();

  //node op
  void EnableTransform();

  void move(double d[3] , mitk::Surface* surface);
  void rotate(double center[3], double axis[3],mitk::Surface* surface);

  void moveXp();
  void moveXm();
  void moveYm();
  void moveYp();
  void moveZm();
  void moveZp();

  void rotateXp();
  void rotateXm();
  void rotateYm();
  void rotateYp();
  void rotateZm();
  void rotateZp();

  void InitStemPos();

  void SelectStem(int index);

  void ClipFemur();

  void updateStemCenter();

  void upSampling();

  void sealImage();

  mitk::Point3D CalStemAssemblyCenter(StemParameter para, vtkMatrix4x4* pos);

  	// itk To VTK Transform
  void TransferItkTransformToVtkMatrix(mitk::AffineTransform3D *itkTransform, vtkMatrix4x4 *vtkmatrix);
  std::vector<double> vtkmatrix2angle(vtkMatrix4x4 *pMatrix);

  //landmark transform
  void onSourcePsetChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void onTargetPsetChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void onMovingNodeChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void OnPushButtonApplyTrans();
  //5cut
  void on_widget_plane_distal_Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void on_widget_plane_backOblique_Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void on_widget_plane_back_Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void on_widget_plane_frontOblique_Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void on_widget_plane_front_Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);

  void OnpushButton_calNormalPlaneRMS();
  void OnpushButtonCalAngle();

  //pad iamge
  void padImage();

 

  Ui::VCViewControls m_Controls;

  //pelvis
  mitk::Surface* m_pelvis{nullptr};
  mitk::Surface* m_tmpSurface{ nullptr };
  mitk::Surface* m_surface_transform{ nullptr };
  mitk::PointSet* m_asis{ nullptr };
  mitk::PointSet* m_pt{ nullptr };

  //femur left
  mitk::Surface* m_femurL{nullptr};
  mitk::Surface *m_femurClipL{nullptr};
  mitk::Image *m_femurImageL{nullptr};
  mitk::Surface *m_kneeL{nullptr};
  mitk::PointSet* m_canalAxisL{ nullptr };
  mitk::PointSet* m_trochanterL{ nullptr };
  mitk::PointSet* m_fhcL{ nullptr };
  mitk::PointSet *m_BtrochanterL{nullptr};
  mitk::PointSet *m_condylesL{nullptr};

  OperateGroup m_femurLeftGroup{8};

  //femur right
  mitk::Surface* m_femurR{ nullptr };
  mitk::Surface *m_femurClipR{nullptr};
  mitk::Image *m_femurImageR{nullptr};
  mitk::Surface *m_kneeR{nullptr};
  mitk::PointSet* m_canalAxisR{ nullptr };
  mitk::PointSet* m_trochanterR{ nullptr };
  mitk::PointSet* m_fhcR{ nullptr };
  mitk::PointSet *m_BtrochanterR{nullptr};
  mitk::PointSet *m_condylesR{nullptr};
  
  OperateGroup m_femurRightGroup{8};

  //stem
  mitk::PointSet* m_stemCenter{ nullptr };
  mitk::Surface* m_stem{ nullptr };
  StemParameter m_stempara{};

  OperateGroup m_stemGroup{2};

  double m_rotate_XAdd{1};
  double m_rotate_XSub{1};
  double m_rotate_X{ 0 };
  double m_rotate_Y{ 0 };
  double m_rotate_Z{ 0 };

  double m_rotateFemurLeftAngle{ 0 };
  double m_rotateFemurLeftAxis[3]{};

  double m_rotateFemurRightAngle{ 0 };
  double m_rotateFemurRightAxis[3]{};

  bool m_builtFemurR{ false };
  bool m_builtFemurL{ false };
  bool m_builtPelvis{ false };

  //landmark trans
  mitk::DataNode* m_sourcePset{ nullptr };
  mitk::DataNode* m_targetPset{ nullptr };
  mitk::DataNode* m_movingNode{ nullptr };


  //5Cut
  mitk::DataNode* m_distalCut{ nullptr };
  mitk::DataNode* m_backObliCut{ nullptr };
  mitk::DataNode* m_backCut{ nullptr };
  mitk::DataNode* m_frontObliCut{ nullptr };
  mitk::DataNode* m_frontCut{ nullptr };
};

inline void VCView::onMovingNodeChanged(QmitkSingleNodeSelectionWidget::NodeList)
{
    m_movingNode = m_Controls.widget_MovingNode->GetSelectedNode();
}

inline void VCView::onTargetPsetChanged(QmitkSingleNodeSelectionWidget::NodeList)
{
    m_targetPset = m_Controls.widget_TargetPset->GetSelectedNode();
}

inline void VCView::onSourcePsetChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
    m_sourcePset = m_Controls.widget_SourcePset->GetSelectedNode();
}

#endif // VCView_h
