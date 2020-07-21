/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include "BoneSegTool3D.h"
#include <mitkImageAccessByItk.h>
#include <mitkImageCast.h>
#include <mitkProgressBar.h>
#include <mitkToolManager.h>
#include <mitkColorSequenceRainbow.h>
#include <usGetModuleContext.h>
#include <usModuleResource.h>

namespace mitk
{
  MITK_TOOL_MACRO(MITKBONESEGMENTATION_EXPORT, BoneSegTool3D, "Bone Segmentation tool");
}



mitk::BoneSegTool3D::BoneSegTool3D() {
}

mitk::BoneSegTool3D::~BoneSegTool3D() {
}

us::ModuleResource mitk::BoneSegTool3D::GetIconResource() const {
  auto moduleContext = us::GetModuleContext();
  auto module = moduleContext->GetModule();
  auto resource = module->GetResource("icon.svg");
  return resource;
}

bool mitk::BoneSegTool3D::CanHandle(mitk::BaseData *referenceData) const
{
  if (referenceData == nullptr)
    return false;

  auto *image = dynamic_cast<mitk::Image *>(referenceData);
  if (image == nullptr)
    return false;

  if (image->GetDimension() != 3)
    return false;

  return true;
}


const char *mitk::BoneSegTool3D::GetName() const
{
  return "Bone Segmentation";
}

const char **mitk::BoneSegTool3D::GetXPM() const
{
  return nullptr;
}

void mitk::BoneSegTool3D::Activated()
{
  Superclass::Activated();
}

void mitk::BoneSegTool3D::Deactivated()
{
  Superclass::Deactivated();
}

void mitk::BoneSegTool3D::DoSegmentation(std::string networkPath) 
{
  mitk::DataNode::Pointer referenceData = m_ToolManager->GetReferenceData(0);
  mitk::Image::Pointer input = dynamic_cast<mitk::Image *>(referenceData->GetData());
  if (input.IsNull())
  {
    MITK_INFO << "Here";
    return;
  }
  unsigned int timestep = mitk::RenderingManager::GetInstance()->GetTimeNavigationController()->GetTime()->GetPos();
  input = Get3DImage(input, timestep);
  if (!input.IsNull())
  {
    MITK_INFO << networkPath;
  }
}
