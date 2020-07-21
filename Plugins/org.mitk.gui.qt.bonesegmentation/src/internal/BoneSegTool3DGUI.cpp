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
#include "BoneSegTool3DGUI.h"

// Qt
#include <QMessageBox>

// mitk image
#include <mitkImage.h>

MITK_TOOL_GUI_MACRO(BONESEGMENTATION_EXPORT, BoneSegTool3DGUI, "")

BoneSegTool3DGUI::BoneSegTool3DGUI() : m_Ui(new Ui::BoneSegTool3DGUI)
{
  m_Ui->setupUi(this);

  connect(m_Ui->buttonPerformImageProcessing, &QPushButton::clicked, this, &BoneSegTool3DGUI::OnDoSegmentation);
}

BoneSegTool3DGUI::~BoneSegTool3DGUI() {
}

void BoneSegTool3DGUI::OnDoSegmentation()
{

}

