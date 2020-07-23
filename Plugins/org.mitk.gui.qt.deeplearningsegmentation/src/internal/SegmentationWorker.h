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
#include<DeepLearningSegmentationTool.h>
#include"SegmentationResultHandler.h"

Q_DECLARE_METATYPE(mitk::DeepLearningSegmentationTool*)
Q_DECLARE_METATYPE(mitk::LabelSetImage::Pointer)

class SegmentationWorker : public QObject
{
  Q_OBJECT
public:
  SegmentationWorker();
public slots:
  void DoWork(mitk::DeepLearningSegmentationTool* segTool,
              SegmentationResultHandler *resultSetter,
              QString networkPath);

signals:
  void Finished(mitk::LabelSetImage::Pointer result, mitk::DeepLearningSegmentationTool* segTool);
  void Failed();
};

#endif
