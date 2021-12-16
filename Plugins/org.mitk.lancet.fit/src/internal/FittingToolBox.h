/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef FittingToolBox_h
#define FittingToolBox_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_FittingToolBoxControls.h"

/**
  \brief FittingToolBox

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class FittingToolBox : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;

protected:
  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;

  /// \brief called by QmitkFunctionality when DataManager's selection has changed
  /*virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
                                  const QList<mitk::DataNode::Pointer> &nodes) override;*/

  /// \brief Called when the user clicks the GUI button
  void DoImageProcessing();

  /// \brief Init QmitkSingleNodeSelectionWidget to only show PointSet
  void InitPointSetSelector(QmitkSingleNodeSelectionWidget *widget);

  /// \brief Called when the SingleNodeSelector selection changed
  void OnNodeSelectionChanged(QmitkSingleNodeSelectionWidget::NodeList);

  /// \brief Called when the pushButton_Plane clicked
  void OnPushButtonFitPlaneClicked();

  /// \brief Called when the pushButton_Rectangle clicked
  void OnPushButtonFitRectangleClicked();

  void OnPushButtonFitSphereClicked();



  /// \brief add a Arrow to dataStorage
  void showArrow(double p_start[], double p_end[], int factor, int color,std::string name = "arrow", bool overwrite = true);
  /// \brief add a Rectangle to dataStorage
  void showRectangle(std::array<double, 3> center,
       std::array<double, 3> normal,
       std::array<double, 3> x_axis,
       std::array<double, 3> y_aixs,
       double length,
       double width,
       int color,
       std::string name = "plane",
       bool overwrite = true);

  void showSphere(std::array<double, 3> center, double r, int color, std::string name = "sphere", bool overwrite = true);

  Ui::FittingToolBoxControls m_Controls;

  //data
  mitk::DataNode* m_PointSet{ nullptr };
};

#endif // FittingToolBox_h
