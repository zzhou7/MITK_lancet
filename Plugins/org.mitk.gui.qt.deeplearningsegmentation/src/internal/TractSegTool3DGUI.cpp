/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#include "TractSegTool3DGUI.h"

MITK_TOOL_GUI_MACRO(DEEPLEARNINGSEGMENTATION_EXPORT, TractSegTool3DGUI, "")

void TractSegTool3DGUI::OnNewToolAssociated(mitk::Tool* tool)
{
  m_SegTool = dynamic_cast<mitk::TractSegTool3D *>(tool);
}

void TractSegTool3DGUI::SetUpUI() 
{
  m_Ui->buttonLoadTrainedNetwork->setVisible(false);
  m_Ui->buttonPerformImageProcessing->setEnabled(true);
  m_Ui->labelWarning->setVisible(false);
  m_TrainedNet = "None";

  QWidget *thresholdWidget = new QWidget;
  QLabel *thresholdLabel = new QLabel("Segmentation Threshold");
  m_ThresholdSpinBox = new QDoubleSpinBox();
  m_ThresholdSpinBox->setRange(0, 1);
  m_ThresholdSpinBox->setValue(0.5);
  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(thresholdLabel);
  layout->addWidget(m_ThresholdSpinBox);
  thresholdWidget->setLayout(layout);
  m_Ui->verticalLayout->addWidget(thresholdWidget); 


  DeepLearningSegmentationGUI::SetUpUI();
}

void TractSegTool3DGUI::OnDoSegmentation() 
{
  dynamic_cast<mitk::TractSegTool3D *>(m_SegTool)->SetThreshold(m_ThresholdSpinBox->value());
  DeepLearningSegmentationGUI::OnDoSegmentation();
}
