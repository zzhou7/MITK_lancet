/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef BoneSegTool3D_h
#define BoneSegTool3D_h

#include <mitkAutoSegmentationTool.h>
#include <mitkDataNode.h>
#include <mitkPointSet.h>
#include <mitkSinglePointDataInteractor.h>

#include <itkImage.h>

#include <MitkBoneSegmentationExports.h>
#include <map>

namespace us {
class ModuleResource;
}

namespace mitk
{
  class MITKBONESEGMENTATION_EXPORT BoneSegTool3D : public mitk::AutoSegmentationTool
  {
  public:
    mitkClassMacro(BoneSegTool3D, AutoSegmentationTool);
    itkFactorylessNewMacro(Self);

    us::ModuleResource GetIconResource() const override;

    bool CanHandle(mitk::BaseData *referenceData) const override;
    const char *GetName() const override;
    const char **GetXPM() const override;

    BoneSegTool3D();
    ~BoneSegTool3D() override;

    void Activated() override;
    void Deactivated() override;

    void DoSegmentation(std::string networkPath);

  private:
  };
} // namespace mitk

#endif
