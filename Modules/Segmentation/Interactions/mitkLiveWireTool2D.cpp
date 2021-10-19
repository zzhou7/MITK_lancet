/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

//#include <mitkContourModelUtils.h>
#include <mitkLiveWireTool2D.h>
#include <mitkLiveWireTool2D.xpm>
#include <mitkToolManager.h>

#include <usGetModuleContext.h>
#include <usModuleResource.h>

#include <type_traits>

namespace mitk
{
  MITK_TOOL_MACRO(MITKSEGMENTATION_EXPORT, LiveWireTool2D, "LiveWire tool");
}

mitk::LiveWireTool2D::LiveWireTool2D() : ClosedContourTool(true)
{
}

mitk::LiveWireTool2D::~LiveWireTool2D()
{
  this->ClearSegmentation();
}

const char **mitk::LiveWireTool2D::GetXPM() const
{
  return mitkLiveWireTool2D_xpm;
}

us::ModuleResource mitk::LiveWireTool2D::GetIconResource() const
{
  return us::GetModuleContext()->GetModule()->GetResource("LiveWire_48x48.png");
}

us::ModuleResource mitk::LiveWireTool2D::GetCursorIconResource() const
{
  return us::GetModuleContext()->GetModule()->GetResource("LiveWire_Cursor_32x32.png");
}

const char *mitk::LiveWireTool2D::GetName() const
{
  return "Live Wire";
}
