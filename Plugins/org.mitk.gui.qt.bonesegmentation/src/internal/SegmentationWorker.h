/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#ifndef SegmentationWorker_h
#define SegmentationWorker_h

#include <QObject>
#include<BoneSegTool3D.h>

Q_DECLARE_METATYPE(mitk::BoneSegTool3D::Pointer)
Q_DECLARE_METATYPE(mitk::LabelSetImage::Pointer)

class SegmentationWorker : public QObject
{
  Q_OBJECT
public:
  SegmentationWorker();
public slots:
  void DoWork(mitk::BoneSegTool3D::Pointer boneSegTool, QString networkPath);

signals:
  void Finished(mitk::LabelSetImage::Pointer result);
  void Failed();


};

#endif
