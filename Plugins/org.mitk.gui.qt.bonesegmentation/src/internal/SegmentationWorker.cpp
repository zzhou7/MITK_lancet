/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#include"SegmentationWorker.h"
#include <mitkIOUtil.h>
#include<QVector>

SegmentationWorker::SegmentationWorker() 
{
}
void SegmentationWorker::DoWork(mitk::BoneSegTool3D::Pointer boneSegTool, SegmentationResultGUI *resultSetter, QString networkPath)
{
  connect(this, &SegmentationWorker::Finished, resultSetter, &SegmentationResultGUI::SetResult);
  connect(this, &SegmentationWorker::Failed, resultSetter, &SegmentationResultGUI::SegmentationProcessFailed);

  try
  {
    mitk::LabelSetImage::Pointer result = boneSegTool->DoSegmentation(networkPath.toStdString());
    MITK_INFO << "Back in Worker";
    emit Finished(result, boneSegTool);
  }
  catch (const mitk::Exception &e)
  {
    MITK_INFO << e.GetDescription();
    emit Failed();
  }
}
