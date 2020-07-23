/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef BoneSegTool3DGUI_h
#define BoneSegTool3DGUI_h

#include <org_mitk_gui_qt_bonesegmentation_Export.h>


#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_BoneSegTool3DGUI.h"

#include <QmitkToolGUI.h>

#include "BoneSegTool3D.h"

#include <QThread>
#include"SegmentationWorker.h"
#include"SegmentationResultHandler.h"

namespace Ui {
class BoneSegTool3DGUI;
}

class BONESEGMENTATION_EXPORT BoneSegTool3DGUI : public QmitkToolGUI
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  //static const std::string VIEW_ID;
  BoneSegTool3DGUI();
  ~BoneSegTool3DGUI() override;
  mitkClassMacro(BoneSegTool3DGUI, QmitkToolGUI)
  itkFactorylessNewMacro(Self) 

signals: 
    void Operate(mitk::BoneSegTool3D::Pointer tool, SegmentationResultHandler* guiSetter, QString networkPath);

  protected slots:
    void OnDoSegmentation();
    void DoLoadTrainedNet();
    void OnNewToolAssociated(mitk::Tool *);
    //void DoSegmentationProcessFinished(mitk::LabelSetImage::Pointer result);
    //void DoSegmentationProcessFailed();


  private:
    QScopedPointer<Ui::BoneSegTool3DGUI> m_Ui;

    QThread *m_SegmentationThread;
    SegmentationWorker *m_Worker;
    
    QString m_TrainedNet;
    mitk::BoneSegTool3D::Pointer m_BoneSegTool;
};

#endif // BoneSegTool3DGUI_h
