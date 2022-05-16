/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef SpineCTRegistration_h
#define SpineCTRegistration_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_SpineCTRegistrationControls.h"

/**
  \brief SpineCTRegistration

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class SpineCTRegistration : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;

protected:
  virtual void CreateQtPartControl(QWidget *parent) override;
  virtual void SetFocus() override;
  void InitNodeSelector(QmitkSingleNodeSelectionWidget *widget);

  // Member variables
  Ui::SpineCTRegistrationControls m_Controls;
  mitk::DataNode *m_CtImageDataNode{nullptr};


  // QT slots

  // User selects a node
  void ChangeCtImage(QmitkSingleNodeSelectionWidget::NodeList);

  // Extract steelball centers as a pointset
  void GetSteelballCenters();

};

#endif // SpineCTRegistration_h
