/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef BoneSegTool3D_h
#define BoneSegTool3D_h

#include "DeepLearningSegmentationTool.h"
#include <MitkDeepLearningSegmentationExports.h>

namespace us {
class ModuleResource;
}

namespace mitk
{
  class MITKDEEPLEARNINGSEGMENTATION_EXPORT BoneSegTool3D : public mitk::DeepLearningSegmentationTool
  {
  public:
    mitkClassMacro(BoneSegTool3D, AutoSegmentationTool);
    itkFactorylessNewMacro(Self);

    BoneSegTool3D();
    ~BoneSegTool3D() override;
  private:
  };
} // namespace mitk

#endif
