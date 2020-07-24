/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef DeepLearningSegmentationGUI_h
#define DeepLearningSegmentationGUI_h

#include <org_mitk_gui_qt_deeplearningsegmentation_Export.h>


#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_DeepLearningSegmentationGUI.h"

#include <QmitkToolGUI.h>

#include "DeepLearningSegmentationTool.h"

#include <QThread>
#include"SegmentationWorker.h"
#include"SegmentationResultHandler.h"

namespace Ui {
class DeepLearningSegmentationGUI;
}

class DEEPLEARNINGSEGMENTATION_EXPORT DeepLearningSegmentationGUI : public QmitkToolGUI
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  //static const std::string VIEW_ID;
  DeepLearningSegmentationGUI();
  ~DeepLearningSegmentationGUI() override;

  void SegmentationRunning();

signals: 
    void Operate(mitk::DeepLearningSegmentationTool* tool, SegmentationResultHandler* guiSetter, QString networkPath);
  void Wait(mitk::DeepLearningSegmentationTool *tool);

  protected slots:
    void SetUpUI();
    void OnDoSegmentation();
    void DoLoadTrainedNet();
    virtual void OnNewToolAssociated(mitk::Tool *) = 0;
    void DoSegmentationProcessFinished();

  protected:
    mitk::DeepLearningSegmentationTool* m_SegTool;

  private:
    QScopedPointer<Ui::DeepLearningSegmentationGUI> m_Ui;

    QThread *m_SegmentationThread;
    SegmentationWorker *m_Worker;
    
    QString m_TrainedNet;
};

#endif // DeepLearningSegmentationGUI_h
