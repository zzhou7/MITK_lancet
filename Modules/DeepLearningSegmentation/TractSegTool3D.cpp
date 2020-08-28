/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include "TractSegTool3D.h"
#include <mitkIPythonService.h>
#include <usGetModuleContext.h>

namespace mitk
{
  MITK_TOOL_MACRO(MITKDEEPLEARNINGSEGMENTATION_EXPORT, TractSegTool3D, "TractSeg tool");
}

mitk::TractSegTool3D::TractSegTool3D()
    : DeepLearningSegmentationTool("Tract Segmentation", "tract_seg.svg", "tract_seg", "in_image", "tractseg.py","segList", DeepLearningSegmentationTool::MITKImage, true)
{
}

mitk::TractSegTool3D::~TractSegTool3D() {
}

void mitk::TractSegTool3D::SetThreshold(double threshold)
{
  mitk::IPythonService::ForceLoadModule();
  us::ModuleContext *context = us::GetModuleContext();
  std::string filter = "(Name=PythonService)";
  auto pythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

  if (!pythonServiceRefs.empty())
  {
    mitk::IPythonService *pythonService =
      dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(pythonServiceRefs.front()));

    pythonService->Execute("threshold = " + std::to_string(threshold));
  }
}

