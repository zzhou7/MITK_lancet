/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#include"SegmentationWorker.h"

SegmentationWorker::SegmentationWorker() 
{
}
void SegmentationWorker::DoWork(mitk::DeepLearningSegmentationTool* segTool,
                                SegmentationResultHandler *resultSetter,
                                QString networkPath)
{
  //connect signals/slots with the result setter which sets the result in the main thread afterwards
  connect(this, &SegmentationWorker::Finished, resultSetter, &SegmentationResultHandler::SetResult);
  connect(this, &SegmentationWorker::Failed, resultSetter, &SegmentationResultHandler::SegmentationProcessFailed);

  try
  {
      //execute segmentation with segmentation tool
    mitk::LabelSetImage::Pointer result = segTool->DoSegmentation(networkPath.toStdString());
    MITK_INFO << "Back in Worker";
    emit Finished(result, segTool);
    //disconnect from result setter. Otherwise, the result is set twice after second execution,
    //three times after third execution,...
    disconnect(this, &SegmentationWorker::Finished, resultSetter, &SegmentationResultHandler::SetResult);
    disconnect(this, &SegmentationWorker::Failed, resultSetter, &SegmentationResultHandler::SegmentationProcessFailed);
  }
  catch (const mitk::Exception &e)
  {
    MITK_INFO << e.GetDescription();
    emit Failed();
    // disconnect from result setter. Otherwise, the result is set twice after second execution, 
    // three times after third execution,...
    disconnect(this, &SegmentationWorker::Finished, resultSetter, &SegmentationResultHandler::SetResult);
    disconnect(this, &SegmentationWorker::Failed, resultSetter, &SegmentationResultHandler::SegmentationProcessFailed);
  }
}

void SegmentationWorker::WaitForSegmentationToFinish(mitk::DeepLearningSegmentationTool *segTool) 
{
  while (segTool->IsSegmentationRunning())
  {
      //Wait
  }
  emit PreviousSegmentationFinished();
}
