/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include "BoneSegTool3D.h"
#include <usGetModuleContext.h>
#include <usModuleResource.h>

namespace mitk
{
  MITK_TOOL_MACRO(MITKDEEPLEARNINGSEGMENTATION_EXPORT, BoneSegTool3D, "Bone Segmentation tool");
}

mitk::BoneSegTool3D::BoneSegTool3D() 
    : DeepLearningSegmentationTool("bone_seg_ct_basic_unet", "sitk_image", "segment.py","output_image")
{
}

mitk::BoneSegTool3D::~BoneSegTool3D() {
}

us::ModuleResource mitk::BoneSegTool3D::GetIconResource() const {
  auto moduleContext = us::GetModuleContext();
  auto module = moduleContext->GetModule();
  auto resource = module->GetResource("icon_bone.svg");
  return resource;
}

const char *mitk::BoneSegTool3D::GetName() const
{
  return "Bone Segmentation";
}

