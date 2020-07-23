/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#include "BoneSegTool3DGUI.h"

MITK_TOOL_GUI_MACRO(DEEPLEARNINGSEGMENTATION_EXPORT, BoneSegTool3DGUI, "")

void BoneSegTool3DGUI::OnNewToolAssociated(mitk::Tool* tool) 
{
  m_SegTool = dynamic_cast<mitk::BoneSegTool3D *>(tool);
}

