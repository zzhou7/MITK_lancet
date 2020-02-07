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

#include "ui_DicomWebViewControls.h"

#include "DicomWebRequestHandler.h"
#include <mitkIRESTManager.h>
#include <mitkImage.h>
#include <mitkLabelSetImage.h>
#include <berryIPartListener.h>
#include <mitkILifecycleAwarePart.h>

/**
  @brief DicomWebView

  This class represents the view to make connections to a PACS server and show some information to do a manual DicomWeb
  upon two existing segmentation volumes.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class DicomWebView : public QmitkAbstractView, public mitk::ILifecycleAwarePart
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
   * @brief Loads the files given by a list of file paths. This method expects exactly three items in the list (image
   * series, seg series a, seg series b). Additional information for segmentation DicomWeb is displayed.
   *
   *
   * @param filePathList a list of absolute file paths
   */
  void LoadDataSegDicomWeb(std::vector<std::string> filePathList);

  /**
   * @brief Calculates how many slices would be delete if the given threshold would be applied to the segmentation
   * volume. The information about the segmentation similarity is used and slices below the threshold are to be
   * discarded. Updates the display.
   *
   * @param value the threshold value (between 0 and 1).
   */
  void OnSliderWidgetChanged(double value);

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

protected:
  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;

  void CreateNewSegmentationC();

  void CleanDicomFolder();
  void UploadNewSegmentation();
  void RestartConnection(utility::string_t newHost);
  void OnRestartConnection();
  pplx::task<bool> TestConnection();
  void OnIndividualCheckChange(int state);

  Ui::DicomWebViewControls m_Controls;

private:
  std::vector<unsigned int> CreateSegmentation(mitk::Image::Pointer baseSegmentation, double threshold);

  std::string GetAlgorithmOfSegByPath(std::string path);

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
  mitk::DICOMweb m_DicomWeb;

  QWidget *m_Parent;

  utility::string_t m_restURL;
  utility::string_t m_HostURL;
};

#endif // DicomWebView_h
