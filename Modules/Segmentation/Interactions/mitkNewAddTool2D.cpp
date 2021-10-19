/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

//#include <mitkContourModelUtils.h>
#include <mitkNewAddTool2D.h>
#include <mitkToolManager.h>

#include <usGetModuleContext.h>
#include <usModuleResource.h>

#include <type_traits>

namespace mitk
{
  MITK_TOOL_MACRO(MITKSEGMENTATION_EXPORT, NewAddTool2D, "New Add tool");
}

mitk::NewAddTool2D::NewAddTool2D() : ClosedContourTool(false)
{
}

mitk::NewAddTool2D::~NewAddTool2D()
{
  this->ClearSegmentation();
}

us::ModuleResource mitk::NewAddTool2D::GetIconResource() const
{
  return us::GetModuleContext()->GetModule()->GetResource("NewAdd_48x48.png");
}

us::ModuleResource mitk::NewAddTool2D::GetCursorIconResource() const
{
  return us::GetModuleContext()->GetModule()->GetResource("NewAdd_Cursor_32x32.png");
}

const char *mitk::NewAddTool2D::GetName() const
{
  return "New Add";
}

const char **mitk::NewAddTool2D::GetXPM() const
{
  return nullptr;
}
