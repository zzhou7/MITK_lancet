/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef QmitkIGTFiducialRegistration_h
#define QmitkIGTFiducialRegistration_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_QmitkIGTFiducialRegistrationControls.h"


#include <ctkServiceEvent.h>

/*!
\brief QmitkIGTFiducialRegistration

\sa QmitkFunctionality
\ingroup ${plugin_target}_internal
*/
class QmitkIGTFiducialRegistration : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

  public:

    void SetFocus() override;

    static const std::string VIEW_ID;

    void CreateQtPartControl(QWidget *parent) override;

    QmitkIGTFiducialRegistration();
    ~QmitkIGTFiducialRegistration() override;

  public slots:


  protected slots:

  void PointerSelectionChanged();
  void ImageSelectionChanged();

  void ProbeSelect();
  void ReferenceSelect();
  void ToolSelect();

  void SnapShotTool();
  void ProbeSawPit();
  void ProbeSawPlane();

  void Calibrate();

  void calibrateGooseSaw(double MatrixRefToPointAcoordinate[16],
                         double sawPointD[3],
                         double sawPlanePointP[3],
                         double sawPlanePointQ[3],
                         double sawPlanePointS[3]);

protected:
  
  void getReferenceMatrix4x4(vtkMatrix4x4 *Mainmatrix, vtkMatrix4x4 *Refmatrix, vtkMatrix4x4 *Returnmatrix)
  {
    Refmatrix->Invert();
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->PostMultiply();
    transform->SetMatrix(Mainmatrix);
    transform->Concatenate(Refmatrix);
    transform->Update();
    transform->GetMatrix(Returnmatrix);
  }

  vtkMatrix4x4 *getVtkMatrix4x4(mitk::NavigationData::Pointer nd);
    
    void InitializeRegistration();

    Ui::IGTFiducialRegistrationControls m_Controls;

    mitk::NavigationData::Pointer m_TrackingPointer;

    mitk::NavigationData::Pointer m_ProbeNDPointer;
    mitk::NavigationData::Pointer m_ToolNDPointer;
    mitk::NavigationData::Pointer m_ReferenceNDPointer;

	vtkMatrix4x4 *m_toolMatrixInRef;
    mitk::PointSet::Pointer m_ProbePitPointInRef;
    mitk::PointSet::Pointer m_probePlanePointInRef;
};

#endif // IGTFiducialRegistration_h
