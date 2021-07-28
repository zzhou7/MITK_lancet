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
#include <itkFileTools.h>

/**
 * @brief This class represents the http message handler for the segmentation DicomWeb.
 */
class DicomWebRequestHandler : public QObject, public mitk::IRESTObserver
{
  Q_OBJECT
public:
  struct DicomDTO
  {
    utility::string_t studyUID;
    std::vector<utility::string_t> seriesUIDList;
  };

  DicomWebRequestHandler();

  /**
   * Creates the request handler for segmentation DicomWeb. The received data is stored into the given download dir.
   *
   * @param downloadDir the directory to which received data is stored.
   */
  DicomWebRequestHandler(std::string downloadDir, utility::string_t pacsURI, bool useSystemProxy=true);

  /**
   * Overrides IRESTObserver::Notify. Here arrive the incoming messages.
   */
  web::http::http_response Notify(const web::uri &uri,
                                  const web::json::value &data,
                                  const web::http::method &method,
                                  const mitk::RESTUtil::ParamMap &headers);
  void UpdateDicomWebUrl(utility::string_t baseURI);
  void UpdateAccessToken(utility::string_t accessToken);
  void UpdateUserCredentials(utility::string_t userName, utility::string_t password);
  void UpdateUseSystemProxy(bool useSystemProxy);
  bool Authenticate(utility::string_t newHost,
                                       utility::string_t username,
                                       utility::string_t password);
  mitk::DICOMweb& DicomWebGet();

signals:
  void InvokeProgress(int, QString status);
  void InvokeUpdateDcmMeta(DicomDTO dto);
  void InvokeLoadData(std::vector<std::string>);

private:
  DicomDTO ExtractDTO(std::map<utility::string_t, utility::string_t>);
  std::vector<pplx::task<std::string>> CreateWADOTasks(DicomDTO dto);
  void RefreshAccess(std::shared_ptr<web::uri> uri);
  void ProcessWADOTasks(std::vector<pplx::task<std::string>> tasks, const web::uri &uri);
  web::http::http_response HandlePut(const web::uri &uri, const web::json::value &data);
  web::http::http_response HandleGet(const web::uri &uri);
  web::http::http_response HandleOptions(const web::uri &uri, const web::json::value &data);
  web::http::http_response HandlePost(const web::uri &uri,
                                      const web::json::value &data,
                                      const mitk::RESTUtil::ParamMap &headers);
  std::vector<std::string> GetPathsToLoad(std::vector<std::string> filePathList);

  utility::string_t m_BaseURI;
  utility::string_t m_UserName;
  utility::string_t m_Password;
  std::string m_DownloadDir;
  mitk::DICOMweb m_DicomWeb;
  bool m_UseSystemProxy;
};

#endif // DicomWebRequestHandler_h