/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef BoneSegTool3DGUI_h
#define BoneSegTool3DGUI_h

#include <org_mitk_gui_qt_deeplearningsegmentation_Export.h>
#include"DeepLearningSegmentationGUI.h"
#include"BoneSegTool3D.h"

namespace Ui {
class BoneSegTool3DGUI;
}

class DEEPLEARNINGSEGMENTATION_EXPORT BoneSegTool3DGUI : public DeepLearningSegmentationGUI
{
  Q_OBJECT

public:
  mitkClassMacro(BoneSegTool3DGUI, QmitkToolGUI)
  itkFactorylessNewMacro(Self) 

  protected slots:
    void OnNewToolAssociated(mitk::Tool *);
};

#endif // BoneSegTool3DGUI_h
