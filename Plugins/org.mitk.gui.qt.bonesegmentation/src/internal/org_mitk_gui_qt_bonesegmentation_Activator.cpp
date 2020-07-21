/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#include "org_mitk_gui_qt_bonesegmentation_Activator.h"
#include "BoneSegTool3DGUI.h"
#include<BoneSegTool3D.h>

namespace mitk
{
  void org_mitk_gui_qt_bonesegmentation_Activator::start(ctkPluginContext *context)
  {
    BoneSegTool3D *seg = new mitk::BoneSegTool3D;
    BERRY_REGISTER_EXTENSION_CLASS(BoneSegTool3DGUI, context)
  }

  void org_mitk_gui_qt_bonesegmentation_Activator::stop(ctkPluginContext *context) { Q_UNUSED(context) }
}
