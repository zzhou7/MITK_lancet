/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#ifndef SegmentationResultGUI_h
#define SegmentationResultGUI_h

#include <QObject>
#include<BoneSegTool3D.h>

class SegmentationResultGUI : public QObject
{
  Q_OBJECT
public:
  SegmentationResultGUI();
  ~SegmentationResultGUI();
public slots:
  void SetResult(mitk::LabelSetImage::Pointer resultSegmentation, mitk::BoneSegTool3D::Pointer boneSegTool);
  void SegmentationProcessFailed();

signals:

};

#endif
