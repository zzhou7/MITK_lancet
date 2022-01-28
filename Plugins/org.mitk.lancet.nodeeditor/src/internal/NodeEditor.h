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

  /// \brief called by QmitkFunctionality when DataManager's selection has changed
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
                                  const QList<mitk::DataNode::Pointer> &nodes) override;

  /// \brief Called when the user clicks the GUI button
  //void DoImageProcessing();

  Ui::NodeEditorControls m_Controls;


    // landmark ICP transform
  // void onSourcePsetChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  // void onTargetPsetChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  // void onMovingNodeChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  // void onIcpPsetChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  // void OnPushButtonApplyLandMarks();
  // void OnPushButtonApplyICP();
  // void OnPushButtonUndo();

  // landmark trans
  mitk::DataNode *m_sourcePset{nullptr};
  mitk::DataNode *m_targetPset{nullptr};
  mitk::DataNode *m_registSrc{nullptr};
  mitk::DataNode *m_icpPset{nullptr};

  mitk::SurfaceRegistration::Pointer m_surfaceRegistration;

  void InitPointSetSelector(QmitkSingleNodeSelectionWidget *widget);
  void InitNodeSelector(QmitkSingleNodeSelectionWidget *widget);

  // DRR generation test
  void setDefault();
  void drrCustom();
  void drrGenerateVisual();
  void drrGenerateData();
  mitk::DataNode *m_inputDRR{nullptr};
  void inputDRRChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  
  // void MoveCT(mitk::Image::ConstPointer inputImage, double RX,
  //             double RY,
  //             double RZ,
  //             double TX,
  //             double TY,
  //             double TZ); // for demonstration purpose

  // twoProjection registration test
  void twoProjectRegister();
  mitk::DataNode *m_inputCT{nullptr};
  mitk::DataNode *m_inputDRR1{nullptr};
  mitk::DataNode *m_inputDRR2{nullptr};
  void inputCTChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void inputDRR1Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void inputDRR2Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);

  // Node operations
  void move(double d[3], mitk::Image *image_viual);
  void rotate(double center[3], double axis[3], double degree, mitk::Image *image_viual);

  // Convert stl polydata to image data
  // mitk::DataNode *m_inputstldata{nullptr};
  // void inputstldataChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void Polystl2Image();

};




// DRR generation test
// inline void NodeEditor::inputstldataChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
// {
//   m_inputstldata = m_Controls.widget_stl->GetSelectedNode();
// }

inline void NodeEditor::inputDRRChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_inputDRR = m_Controls.widget_DRR->GetSelectedNode();
}

inline void NodeEditor::inputCTChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_inputCT = m_Controls.widget_inputCT_regis->GetSelectedNode();
}

inline void NodeEditor::inputDRR1Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_inputDRR1 = m_Controls.widget_inputDRR1_regis->GetSelectedNode();
}

inline void NodeEditor::inputDRR2Changed(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/)
{
  m_inputDRR2 = m_Controls.widget_inputDRR2_regis->GetSelectedNode();
}



#endif // NodeEditor_h
