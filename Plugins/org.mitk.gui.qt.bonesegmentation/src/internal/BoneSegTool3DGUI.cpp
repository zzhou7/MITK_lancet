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
#include <QFileDialog>
// mitk image
#include <mitkImage.h>
#include <mitkIOUtil.h>


MITK_TOOL_GUI_MACRO(BONESEGMENTATION_EXPORT, BoneSegTool3DGUI, "")

BoneSegTool3DGUI::BoneSegTool3DGUI() : m_Ui(new Ui::BoneSegTool3DGUI)
{
  qRegisterMetaType<mitk::BoneSegTool3D::Pointer>();
  m_Ui->setupUi(this);

  connect(m_Ui->buttonPerformImageProcessing, &QPushButton::clicked, this, &BoneSegTool3DGUI::OnDoSegmentation);
  connect(m_Ui->buttonLoadTrainedNetwork, &QPushButton::clicked, this, &BoneSegTool3DGUI::DoLoadTrainedNet);
  m_Ui->buttonPerformImageProcessing->setEnabled(false);

  m_SegmentationThread = new QThread;
  m_Worker = new SegmentationWorker;
  m_Worker->moveToThread(m_SegmentationThread);
  // Signal/Slot connects
  connect(this, &BoneSegTool3DGUI::Operate, m_Worker, &SegmentationWorker::DoWork);
  connect(m_Worker, &SegmentationWorker::Finished, this, &BoneSegTool3DGUI::DoSegmentationProcessFinished);
  connect(m_Worker, &SegmentationWorker::Failed, this, &BoneSegTool3DGUI::DoSegmentationProcessFailed);
  connect(this, SIGNAL(NewToolAssociated(mitk::Tool *)), this, SLOT(OnNewToolAssociated(mitk::Tool *)));
}

BoneSegTool3DGUI::~BoneSegTool3DGUI() {
}

void BoneSegTool3DGUI::OnNewToolAssociated(mitk::Tool* tool) 
{
  m_BoneSegTool = dynamic_cast<mitk::BoneSegTool3D *>(tool);
}

void BoneSegTool3DGUI::DoLoadTrainedNet()
{
  QString tempPath = QString::fromStdString(mitk::IOUtil::GetTempPathA());
  QString pretrainedNetResourcesPath =
    QFileDialog::getOpenFileName(nullptr, tr("Open File"), tempPath, tr("Images (*.pth.tar)"));

  m_TrainedNet = pretrainedNetResourcesPath;

  if (m_TrainedNet != "")
  {
    m_Ui->labelWarning->setVisible(false);
    m_Ui->buttonPerformImageProcessing->setEnabled(true);
  }
}

void BoneSegTool3DGUI::OnDoSegmentation()
{
    MITK_INFO << "[Start] Segmentation";
    m_SegmentationThread->start();
    emit Operate(m_BoneSegTool, m_TrainedNet);
}

void BoneSegTool3DGUI::DoSegmentationProcessFinished() 
{
  QMessageBox::information(nullptr,
                       "Segmentation finished",
                       "Segmentation finished.");
}

void BoneSegTool3DGUI::DoSegmentationProcessFailed()
{
  QMessageBox::warning(nullptr,
                       "Error in segmentation",
                       "There was an error in the segmentation process. No resulting segmentation can be loaded.");
}
