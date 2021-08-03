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

#include <mitkIOUtil.h>
#include <mitkRESTUtil.h>

#include "DicomWebRequestHandler.h"

#include <usGetModuleContext.h>
#include <usModuleContext.h>
#include <usModuleInitialization.h>

#include <Poco/Zip/Decompress.h>
#include <Poco/File.h>
#include <Poco/Path.h>

#include <mitkRestRedirectException.h>

US_INITIALIZE_MODULE

DicomWebRequestHandler::DicomWebRequestHandler() : m_UseSystemProxy{true} {}

DicomWebRequestHandler::DicomWebRequestHandler(std::string downloadDir, utility::string_t pacsURI, bool useSystemProxy)
  : m_DownloadDir{downloadDir}, m_UseSystemProxy{useSystemProxy}
{
  m_DicomWeb = mitk::DICOMweb(pacsURI, m_UseSystemProxy);
}

void DicomWebRequestHandler::UpdateDicomWebUrl(utility::string_t baseURI) {
  m_BaseURI = baseURI;
  utility::string_t retrieveURI = baseURI + U("/dcm4chee-arc/aets/KAAPANA/");
  utility::string_t sendURI = baseURI + U("/dicomweb/");
  m_DicomWeb.UpdateBaseUri(retrieveURI);
  m_DicomWeb.UpdateSendUri(sendURI);
}

void DicomWebRequestHandler::UpdateAccessToken(utility::string_t accessToken) {
  m_DicomWeb.UpdateAccessToken(accessToken);
}

void DicomWebRequestHandler::UpdateUserCredentials(utility::string_t userName, utility::string_t password)
{
  m_UserName = userName;
  m_Password = password;
}

void DicomWebRequestHandler::UpdateUseSystemProxy(bool useSystemProxy) {
  m_UseSystemProxy = useSystemProxy;
  m_DicomWeb.UpdateUseSystemProxy(useSystemProxy);
}

mitk::DICOMweb& DicomWebRequestHandler::DicomWebGet() {
  return m_DicomWeb;
}

bool DicomWebRequestHandler::Authenticate(utility::string_t newHost, utility::string_t username, utility::string_t password)
{
  web::uri authUrl = web::uri(newHost + U("/oauth/login"));
  std::map<utility::string_t, utility::string_t> content = {};
  content[U("client_id")] = U("admin-cli");
  content[U("username")] = username;
  content[U("password")] = password;
  content[U("grant_type")] = U("password");
  pplx::task<web::json::value> resultJSON = m_DicomWeb.m_RESTManager->SendRequest(
    authUrl, mitk::IRESTManager::RequestType::Post, {}, content, m_UseSystemProxy);
  try
  {
    web::json::value result = resultJSON.get();
    auto token = result[U("access_token")].to_string();
    UpdateAccessToken(U("kc-access=") + token.substr(1, token.size() - 2));
    UpdateUserCredentials(username, password);
  }
  catch (const std::exception &e)
  {
    std::string unauthorized = "401";
    if (unauthorized.compare(e.what()))
    {
      MITK_INFO << "Username or password was wrong";
    }
    else
    {
      MITK_INFO << "Authorization went wrong (error: " << e.what() << ")";
    }
    return false;
  }
  return true;
}

web::http::http_response DicomWebRequestHandler::Notify(const web::uri &uri,
                                                      const web::json::value &data,
                                                      const web::http::method &method,
                                                      const mitk::RESTUtil::ParamMap &headers)
{
  MITK_INFO << "Incoming notify";
  if (method == web::http::methods::GET)
  {
    return HandleGet(uri);
  }
  else if (method == web::http::methods::PUT)
  {
    return HandlePut(uri, data);
  }
  else if (method == web::http::methods::POST)
  {
    return HandlePost(uri, data, headers);
  }
  else if (method == web::http::methods::OPTIONS)
  {
    return HandleOptions(uri, data);
  }

  web::http::http_response response(web::http::status_codes::BadGateway);
  response.set_body(U("No one can handle http method from request")); // TODO: include method name
  return response;
}

web::http::http_response DicomWebRequestHandler::HandlePut(const web::uri &uri, const web::json::value &data)
{

  MITK_INFO << "not implemented yet";
  auto errorResponse = web::http::http_response(web::http::status_codes::BadRequest);
  return errorResponse;
}

DicomWebRequestHandler::DicomDTO DicomWebRequestHandler::ExtractDTO(
  std::map<utility::string_t, utility::string_t> httpParams)
{
  // data extraction
  DicomDTO dto;
  dto.studyUID = httpParams.at(U("studyUID"));
  auto seriesUIDList = httpParams.at(U("seriesUIDList"));
  auto seriesUIDListUtf8 = mitk::RESTUtil::convertToUtf8(seriesUIDList);
  std::istringstream f(seriesUIDListUtf8);
  std::string s;
  while (getline(f, s, ','))
  {
    dto.seriesUIDList.push_back(mitk::RESTUtil::convertToTString(s));
  }
  return dto;
}

std::vector<pplx::task<std::string>> DicomWebRequestHandler::CreateWADOTasks(DicomDTO dto)
{
  std::vector<pplx::task<std::string>> tasks;

    for (auto segSeriesUID : dto.seriesUIDList)
    {
      utility::string_t folderPathSeries =
        utility::conversions::to_string_t(mitk::IOUtil::CreateTemporaryDirectory("XXXXXX", m_DownloadDir) + "/");

      try
      {
        auto seriesTask = m_DicomWeb.SendWADO(folderPathSeries, dto.studyUID, segSeriesUID);
        tasks.push_back(seriesTask);
      }
      catch (const mitk::Exception &exception)
      {
        MITK_INFO << exception.what();
        return std::vector<pplx::task<std::string>>();
      }
    }

  return tasks;
}

std::vector<std::string> DicomWebRequestHandler::GetPathsToLoad(std::vector<std::string> filePathList)
{
  std::vector<std::string> filesToLoad;
  for (std::string &filePath : filePathList)
  {
    std::ifstream file(filePath.c_str(), std::ios::binary);
    if (!file.good())
    {
      mitkThrow() << "Cannot open '" + filePath + "' for reading";
    }
    std::string folder = filePath.substr(0, filePath.find(".zip"));
    Poco::Path pocoPath = Poco::Path(folder);
    Poco::Zip::Decompress unzipper(file, Poco::Path(folder));
    unzipper.decompressAllFiles();
    while (pocoPath.getExtension() != "dcm")
    {
      std::vector<std::string> pathes;
      auto path = Poco::File(pocoPath);
      path.list(pathes);
      pocoPath.append(Poco::Path(pathes.front()));
    }
    filesToLoad.push_back(pocoPath.toString());
  }
  return filesToLoad;
}

void DicomWebRequestHandler::RefreshAccess(std::shared_ptr<web::uri> uri)
{
  // if access token is expired, refresh the access token and send same request again
  MITK_INFO << "Refresh access";
  emit InvokeProgress(100, {""});
  bool successfulAuthentification = Authenticate(m_BaseURI, m_UserName, m_Password);
  if (successfulAuthentification)
  {
    HandleGet(*uri);
  }
  else
  {
    emit InvokeProgress(100, {""});
    MITK_ERROR << "Exception when handling GET";
  }
}

void DicomWebRequestHandler::ProcessWADOTasks(std::vector<pplx::task<std::string>> tasks, const web::uri &uri)
{
  auto joinTask = pplx::when_all(begin(tasks), end(tasks));
  auto reauthUri = std::make_shared<web::uri>(web::uri(uri.to_string()));
  joinTask.then([&, reauthUri](pplx::task<std::vector<std::string>> filePathListTask) {
    try
    {
      std::vector<std::string> filesToLoad = GetPathsToLoad(filePathListTask.get());
      emit InvokeLoadData(filesToLoad);
      emit InvokeProgress(40, {""});
    }
    catch (const mitk::RestRedirectException &e)
    {
      RefreshAccess(reauthUri);
    }
    catch (...)
    {
      emit InvokeProgress(100, {""});
      MITK_ERROR << "Exception when handling GET";
    }
  });
  joinTask.wait();
}

web::http::http_response DicomWebRequestHandler::HandleGet(const web::uri &uri)
{
  auto query = web::uri(uri).query();
  auto httpParams = web::uri::split_query(query);

  // IHE Invoke Image Display style
  auto requestType = httpParams.find(U("requestType"));

  auto errorResponse = web::http::http_response(web::http::status_codes::BadRequest);
  auto successResponse = web::http::http_response(web::http::status_codes::OK);

  if (!itksys::SystemTools::FileIsDirectory(m_DownloadDir))
  {
    itk::FileTools::CreateDirectory(m_DownloadDir);
  }

  if (requestType == httpParams.end())
  {
    MITK_INFO << "no requestType parameter was provided";
    return errorResponse;
  }
  else
  {
    DicomDTO dto = ExtractDTO(httpParams);
    emit InvokeProgress(20, {"incoming series request ..."});
    emit InvokeUpdateDcmMeta(dto);

    std::vector<pplx::task<std::string>> tasks = CreateWADOTasks(dto);
    if (tasks.empty())
    {
      MITK_INFO << "WADO request failed";
      return errorResponse;
    }

    emit InvokeProgress(40, {"download series ..."});

    try
    {
      ProcessWADOTasks(tasks, uri);
    }
    catch (const mitk::Exception &exception)
    {
      MITK_INFO << exception.what();
      return errorResponse;
    }
  }
  return successResponse;
}

web::http::http_response DicomWebRequestHandler::HandlePost(const web::uri &uri,
                                                            const web::json::value &data,
                                                            const mitk::RESTUtil::ParamMap &headers)
{
  if (!data.is_null()) // avoid unused warning
    MITK_INFO << "data was not null";

  MITK_INFO <<"not implemented yet";
  auto errorResponse = web::http::http_response(web::http::status_codes::BadRequest);
  return errorResponse;
}

web::http::http_response DicomWebRequestHandler::HandleOptions(const web::uri &uri, const web::json::value &data)
{
  if (uri.to_string() != U("/inject")) // avoid unused warning
    MITK_INFO << "no inject path";

  if (!data.is_null()) // avoid unused warning
    MITK_INFO << "data was not null";

  MITK_INFO << "OPTIONS incoming";

  web::http::http_response response(web::http::status_codes::OK);

  response.headers().add(U("Access-Control-Allow-Methods"), "PUT");
  response.headers().add(U("Access-Control-Allow-Headers"), "Content-Type");
  response.headers().add(U("Access-Control-Allow-Origin"), "http://localhost:8002");

  return response;
}