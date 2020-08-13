/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef TractSegTool3D_h
#define TractSegTool3D_h

#include "DeepLearningSegmentationTool.h"
#include <MitkDeepLearningSegmentationExports.h>

namespace us {
class ModuleResource;
}

namespace mitk
{
  class MITKDEEPLEARNINGSEGMENTATION_EXPORT TractSegTool3D : public mitk::DeepLearningSegmentationTool
  {
  public:
    mitkClassMacro(TractSegTool3D, AutoSegmentationTool);
    itkFactorylessNewMacro(Self);

    TractSegTool3D();
    ~TractSegTool3D() override;
  private:
  };
} // namespace mitk

#endif
