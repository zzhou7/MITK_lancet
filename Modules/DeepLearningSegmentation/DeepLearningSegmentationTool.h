/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef DeepLearningSegmentationTool_h
#define DeepLearningSegmentationTool_h

#include <mitkAutoSegmentationTool.h>
#include <MitkDeepLearningSegmentationExports.h>

namespace us {
class ModuleResource;
}

namespace mitk
{
  class MITKDEEPLEARNINGSEGMENTATION_EXPORT DeepLearningSegmentationTool : public mitk::AutoSegmentationTool
  {
  public:

    us::ModuleResource GetIconResource() const override = 0;

    bool CanHandle(mitk::BaseData *referenceData) const override;
    const char *GetName() const override = 0;
    const char **GetXPM() const override;

    DeepLearningSegmentationTool(std::string pythonFolder,
                                 std::string inputImageVarName,
                                 std::string pythonFileName,
                                 std::string outputImageVarName);
    ~DeepLearningSegmentationTool() override;

    void Activated() override;
    void Deactivated() override;

    mitk::LabelSetImage::Pointer DoSegmentation(std::string networkPath);
    mitk::Image::Pointer GetInputImage();
    bool IsSegmentationRunning();

    mitk::DataStorage* GetDataStorage();
    mitk::DataNode *GetReferenceData();

  protected:
    std::string m_PythonProjectPath;
    std::string m_InputImageVarName;
    std::string m_PythonFileName;
    std::string m_OutputImageVarName;

  private:
    bool m_SegmentationRunning;
  };
} // namespace mitk

#endif
