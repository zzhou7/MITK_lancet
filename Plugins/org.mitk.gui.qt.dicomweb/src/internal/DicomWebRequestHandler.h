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

#ifndef DicomWebRequestHandler_h
#define DicomWebRequestHandler_h

#include <QObject>
#include <mitkDICOMweb.h>
#include <mitkIRESTManager.h>
#include <mitkIRESTObserver.h>

/**
 * @brief This class represents the http message handler for the segmentation DicomWeb.
 */
class DicomWebRequestHandler : public QObject, public mitk::IRESTObserver
{
  Q_OBJECT
public:
  struct DicomDTO
  {
    utility::string_t segSeriesUIDA;
    utility::string_t segSeriesUIDB;
    utility::string_t imageSeriesUID;
    utility::string_t studyUID;
    utility::string_t segInstanceUIDA;
    utility::string_t segInstanceUIDB;
    utility::string_t srSeriesUID;
    std::vector<utility::string_t> seriesUIDList;
    std::vector<double> simScoreArray;
    int minSliceStart;
    utility::string_t groundTruth;
  };

  DicomWebRequestHandler();

  /**
   * Creates the request handler for segmentation DicomWeb. The received data is stored into the given download dir.
   *
   * @param downloadDir the directory to which received data is stored.
   */
  DicomWebRequestHandler(std::string downloadDir, utility::string_t pacsURI);

  /**
   * Overrides IRESTObserver::Notify. Here arrive the incoming messages.
   */
  web::http::http_response Notify(const web::uri &uri,
                                  const web::json::value &data,
                                  const web::http::method &method,
                                  const mitk::RESTUtil::ParamMap &headers);

signals:
  void InvokeProgress(int, QString status);
  void InvokeSimilarityGraph(std::vector<double> score, int sliceStart);
  void InvokeUpdateDcmMeta(DicomDTO dto);
  void InvokeLoadData(std::vector<std::string>);
  void InvokeLoadDataSegDicomWeb(std::vector<std::string>);

private:
  DicomDTO ExtractDTO(const web::json::value &data);
  web::http::http_response HandlePut(const web::uri &uri, const web::json::value &data);
  web::http::http_response HandleGet(const web::uri &uri, const web::json::value &data);
  web::http::http_response HandleOptions(const web::uri &uri, const web::json::value &data);
  web::http::http_response HandlePost(const web::uri &uri,
                                      const web::json::value &data,
                                      const mitk::RESTUtil::ParamMap &headers);

  std::string m_DownloadDir;
  mitk::DICOMweb m_DicomWeb;
};

#endif // DicomWebRequestHandler_h
