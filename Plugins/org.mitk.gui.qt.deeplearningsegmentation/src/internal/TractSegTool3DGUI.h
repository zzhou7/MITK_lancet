/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef TractSegTool3DGUI_h
#define TractSegTool3DGUI_h

#include <org_mitk_gui_qt_deeplearningsegmentation_Export.h>
#include"DeepLearningSegmentationGUI.h"
#include"TractSegTool3D.h"
#include <QDoubleSpinBox>

namespace Ui {
class TractSegTool3DGUI;
}

class DEEPLEARNINGSEGMENTATION_EXPORT TractSegTool3DGUI : public DeepLearningSegmentationGUI
{
  Q_OBJECT

public:
  mitkClassMacro(TractSegTool3DGUI, QmitkToolGUI)
  itkFactorylessNewMacro(Self) 

  protected slots:
    void OnNewToolAssociated(mitk::Tool *);
    void SetUpUI() override;
  void OnDoSegmentation() override;

  private:
  QDoubleSpinBox *m_ThresholdSpinBox;
};

#endif // TractSegTool3DGUI_h
