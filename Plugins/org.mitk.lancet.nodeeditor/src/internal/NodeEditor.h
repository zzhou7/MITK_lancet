/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef NodeEditor_h
#define NodeEditor_h

#include "mitkImage.h"
#include "nodebinder.h"
#include "surfaceregistraion.h"
#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_NodeEditorControls.h"

/**
  \brief NodeEditor

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class NodeEditor : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;

protected:
  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;

  Ui::NodeEditorControls m_Controls;

  void InitPointSetSelector(QmitkSingleNodeSelectionWidget *widget);
  void InitNodeSelector(QmitkSingleNodeSelectionWidget *widget);

  // DRR
  void SetUiDefault();
  void Drr();
  void DrrVisualization();
  void DrrGenerateData();
  mitk::DataNode *m_DrrCtImageDataNode{nullptr};
  void DrrCtImageChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);

  // new DRR
  void V1DrrGenerateData();
  void V2DrrGenerateData();
  void VisualizeDrrProjectionModel();
  mitk::DataNode *m_NewDrrCtImageDataNode{nullptr};
  void NewDrrCtImageChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);

  // TwoProjection registration
  void Register();
  void InitialMetric();
  mitk::DataNode *m_RegistrationCtImageDataNode{nullptr};
  mitk::DataNode *m_InputDrrImageDataNode_1{nullptr};
  mitk::DataNode *m_InputDrrImageDataNode_2{nullptr};
  void RegistrationCtImageChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void InputDrrImageChanged_1(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void InputDrrImageChanged_2(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);

  // Node operations
  void TranslateImage(double direction[3], mitk::Image *mitkImage);
  void RotateImage(double center[3], double axis[3], double degree, mitk::Image *mitkImage);

  // Evaluate the registration result
  void EvaluateRegistration();
  mitk::DataNode *m_RawCtImageDataNode{nullptr};
  mitk::DataNode *m_EvaluationPointsDataNode{nullptr};
  void RawCtImageChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void EvaluationPointsChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);

  // Convert stl polydata surface to image data
  void ConvertPolyDataToImage();
  void PolyDataToImageWithWhiteBackGround();
  void GenerateWhiteImage();
  void PolyDataToImageData();
  mitk::DataNode *m_InputSurfaceDataNode{nullptr};
  mitk::DataNode *m_InputImageToCropDataNode{nullptr};
  void InputSurfaceChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void InputImageToCropChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
};

#endif // NodeEditor_h
