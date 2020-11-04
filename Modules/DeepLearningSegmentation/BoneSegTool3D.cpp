/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include "BoneSegTool3D.h"

namespace mitk
{
  MITK_TOOL_MACRO(MITKDEEPLEARNINGSEGMENTATION_EXPORT, BoneSegTool3D, "Bone Segmentation tool");
}

mitk::BoneSegTool3D::BoneSegTool3D() 
    : DeepLearningSegmentationTool("Bone Segmentation", "icon_bone.svg", "bone_seg_ct_basic_unet", "sitk_image", "segment.py", "output_image", DeepLearningSegmentationTool::SimpleITKImage)
{
}

mitk::BoneSegTool3D::~BoneSegTool3D() {
}

