/*===================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center,
Division of Medical and Biological Informatics.
All rights reserved.

This software is distributed WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See LICENSE.txt or http://www.mitk.org for details.

===================================================================*/

#ifndef DicomWebView_h
#define DicomWebView_h

#include <berryISelectionListener.h>
#include <QmitkAbstractView.h>
#include <mitkILifecycleAwarePart.h>
#include <mitkIRenderWindowPartListener.h>
#include "DicomWebRequestHandler.h"
#include <mitkIRESTManager.h>
#include <berryIPartListener.h>





#include "ui_DicomWebViewControls.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateAnd.h"
#include "mitkNodePredicateOr.h"
/**
  @brief DicomWebView

  This class represents the view to make connections to a PACS server and show some information to do a manual DicomWeb
  upon two existing segmentation volumes.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class DicomWebView : public QmitkAbstractView, public mitk::ILifecycleAwarePart
//, public mitk::IRenderWindowPartListener
{
  Q_OBJECT

public:
  static const std::string VIEW_ID;

  /**
   * @brief Loads the files given by a list of file paths.
   *
   * @param filePathList a list of absolute file paths
   * @return set of objects containing the newly loaded data nodes
   */
  mitk::DataStorage::SetOfObjects::Pointer LoadData(std::vector<std::string> filePathList);

  /**
   * @brief Progress is added to the progressbar in a percent value and the given status is displayed. The progress will
   * not exceed 100 points.
   *
   * @param progress the progress in percent points to be added to the bar
   * @param status the status to be displayed after achieving the progress
   */
  void AddProgress(int progress, QString status);

  /**
   * @brief Initializes the DICOM meta data used by this view.
   *
   * @param dto the data transfer object for received DICOM meta data.
   */
  void InitializeDcmMeta(DicomWebRequestHandler::DicomDTO dto);

  virtual void Activated() override;
  virtual void Deactivated() override;
  virtual void Visible() override;
  virtual void Hidden() override;

signals:
  void InvokeProgress(int, QString status);

protected slots:
  void OnPatientSelectionChanged(QList<mitk::DataNode::Pointer> nodes);
  void OnSegmentationSelectionChanged(QList<mitk::DataNode::Pointer> nodes);
  void OnUseSystemProxyChanged(bool toggled);


protected:
  virtual void CreateQtPartControl(QWidget *parent) override;

	void SetPredicates();
  virtual void SetFocus() override;

  void CleanDicomFolder();
  void UploadNewSegmentation();
  void RestartConnection(utility::string_t newHost);
  void OnRestartConnection();
  pplx::task<bool> TestConnection();

  Ui::DicomWebViewControls m_Controls;
  mitk::NodePredicateNot::Pointer m_IsNotAHelperObject;
  mitk::NodePredicateAnd::Pointer m_IsOfTypeImagePredicate;
  mitk::NodePredicateOr::Pointer m_IsASegmentationImagePredicate;
  mitk::NodePredicateAnd::Pointer m_IsAPatientImagePredicate;
  

private:
  // std::vector<unsigned int> CreateSegmentation(mitk::Image::Pointer baseSegmentation, double threshold);

  //std::string GetAlgorithmOfSegByPath(std::string path);

  void SetSimilarityGraph(std::vector<double> simScoreArray, int sliceMinStart);

  void StartServer();

  mitk::IRESTManager *m_ManagerService;
  DicomWebRequestHandler *m_RequestHandler;

  std::string m_CurrentStudyUID;
  std::string m_SRUID;

  std::string m_DownloadBaseDir;
  std::string m_UploadBaseDir;

  mitk::DataNode::Pointer m_Image;
  mitk::DataNode::Pointer m_SegA;
  mitk::DataNode::Pointer m_SegB;
  mitk::DataNode::Pointer m_SegC;
  std::map<double, double> m_ScoreMap;
  std::string m_GroundTruth;
  std::string m_thresholdLabel;

  QWidget *m_Parent;

  utility::string_t m_restURL;
  utility::string_t m_HostURL;
};

#endif // DicomWebView_h
