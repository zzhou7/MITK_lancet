/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#include"SegmentationResultHandler.h"
#include <mitkIOUtil.h>
#include<QVector>
#include <QMessageBox>

SegmentationResultHandler::SegmentationResultHandler() {
}

SegmentationResultHandler::~SegmentationResultHandler(){
}

void SegmentationResultHandler::SetResult(mitk::LabelSetImage::Pointer resultSegmentation,
                                      mitk::BoneSegTool3D::Pointer boneSegTool)
{
  try
  {
    mitk::DataNode::Pointer outputNode = mitk::DataNode::New();
    outputNode->SetName("Bone_seg");
    outputNode->SetData(resultSegmentation);
    boneSegTool->GetDataStorage()->Add(outputNode, boneSegTool->GetReferenceData());
    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
  }
  catch (const mitk::Exception &e)
  {
    MITK_INFO << e.GetDescription();
  }
}

void SegmentationResultHandler::SegmentationProcessFailed()
{
    QMessageBox::warning(nullptr,
                         "Error in segmentation",
                         "There was an error in the segmentation process. No resulting segmentation can be loaded.");
}
