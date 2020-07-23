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
#include "DeepLearningSegmentationGUI.h"

// Qt
#include <QFileDialog>
// mitk image
#include <mitkImage.h>
#include <mitkIOUtil.h>

DeepLearningSegmentationGUI::DeepLearningSegmentationGUI() : m_Ui(new Ui::DeepLearningSegmentationGUI)
{
  qRegisterMetaType<mitk::DeepLearningSegmentationTool*>();
  qRegisterMetaType<mitk::LabelSetImage::Pointer>();
  qRegisterMetaType<QVector<int>>();
  m_Ui->setupUi(this);

  connect(m_Ui->buttonPerformImageProcessing, &QPushButton::clicked, this, &DeepLearningSegmentationGUI::OnDoSegmentation);
  connect(m_Ui->buttonLoadTrainedNetwork, &QPushButton::clicked, this, &DeepLearningSegmentationGUI::DoLoadTrainedNet);
  m_Ui->buttonPerformImageProcessing->setEnabled(false);

  m_SegmentationThread = new QThread;
  m_Worker = new SegmentationWorker;
  m_Worker->moveToThread(m_SegmentationThread);
  // Signal/Slot connects
  connect(this, &DeepLearningSegmentationGUI::Operate, m_Worker, &SegmentationWorker::DoWork);
  connect(this, SIGNAL(NewToolAssociated(mitk::Tool *)), this, SLOT(OnNewToolAssociated(mitk::Tool *)));
}

DeepLearningSegmentationGUI::~DeepLearningSegmentationGUI() {
}

void DeepLearningSegmentationGUI::DoLoadTrainedNet()
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

void DeepLearningSegmentationGUI::OnDoSegmentation()
{
    MITK_INFO << "[Start] Segmentation";
    SegmentationResultHandler *resultSetter = new SegmentationResultHandler;
    m_SegmentationThread->start();
    emit Operate(m_SegTool, resultSetter, m_TrainedNet);
}

