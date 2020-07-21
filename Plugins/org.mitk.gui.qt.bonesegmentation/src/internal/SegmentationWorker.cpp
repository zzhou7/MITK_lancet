/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#include"SegmentationWorker.h"
#include <mitkIOUtil.h>

SegmentationWorker::SegmentationWorker() {
}
void SegmentationWorker::DoWork(mitk::BoneSegTool3D::Pointer boneSegTool, QString networkPath)
{
  try
  {
    boneSegTool->DoSegmentation(networkPath.toStdString());
    emit Finished();
  }
  catch (const mitk::Exception &e)
  {
    MITK_INFO << e.GetDescription();
    emit Failed();
  }
}
