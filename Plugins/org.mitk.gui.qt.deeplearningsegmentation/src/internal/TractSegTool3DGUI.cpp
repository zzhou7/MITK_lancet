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

  DeepLearningSegmentationGUI::SetUpUI();
}