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

US_INITIALIZE_MODULE

DicomWebRequestHandler::DicomWebRequestHandler() {}

DicomWebRequestHandler::DicomWebRequestHandler(std::string downloadDir, utility::string_t pacsURI)
  : m_DownloadDir{downloadDir}
{
  m_DicomWeb = mitk::DICOMweb(pacsURI);
}

void DicomWebRequestHandler::UpdateDicomWebUrl(utility::string_t pacsURI) {
  m_DicomWeb = mitk::DICOMweb(pacsURI);
}

mitk::DICOMweb DicomWebRequestHandler::DicomWebGet() {
  return m_DicomWeb;
}

DicomWebRequestHandler::DicomDTO DicomWebRequestHandler::ExtractDTO(const web::json::value &data)
{
  DicomDTO dto;
  auto messageTypeKey = data.at(U("messageType"));
  if (messageTypeKey.as_string() == U("downloadData"))
  {
    MITK_INFO << "within extract dto";

    auto imageStudyUIDKey = data.at(U("studyUID"));
    auto srSeriesUIDKey = data.at(U("srSeriesUID"));

    auto groundTruthKey = data.at(U("groundTruth"));

    auto simScoreKey = data.at(U("simScoreArray"));
    auto minSliceStartKey = data.at(U("minSliceStart"));

    dto.srSeriesUID = srSeriesUIDKey.as_string();
    dto.groundTruth = groundTruthKey.as_string();
    dto.studyUID = imageStudyUIDKey.as_string();
    dto.minSliceStart = minSliceStartKey.as_integer();

    std::vector<double> vec;
    web::json::array simArray = simScoreKey.as_array();

    for (web::json::value score : simArray)
    {
      vec.push_back(score.as_double() * 100);
    }

    dto.simScoreArray = vec;
  }
  return dto;
}

web::http::http_response DicomWebRequestHandler::Notify(const web::uri &uri,
                                                      const web::json::value &data,
                                                      const web::http::method &method,
                                                      const mitk::RESTUtil::ParamMap &headers)
{
  //headers.size();

  MITK_INFO << "Incoming notify";
  if (method == web::http::methods::GET)
  {
    return HandleGet(uri, data);
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
  auto query = web::uri(uri).query();
  if (!data.is_null()) // avoid unused warning
    MITK_INFO << "data was not null";
  auto errorResponse = web::http::http_response(web::http::status_codes::BadRequest);
  return errorResponse;
}

web::http::http_response DicomWebRequestHandler::HandleGet(const web::uri &uri, const web::json::value &data)
{
  if (!data.is_null()) // avoid unused warning
    MITK_INFO << "data was not null";

  auto query = web::uri(uri).query();
  auto httpParams = web::uri::split_query(query);

  // IHE Invoke Image Display style
  auto requestType = httpParams.find(U("requestType"));

  auto errorResponse = web::http::http_response(web::http::status_codes::BadRequest);

  if (requestType != httpParams.end())
  {
    if (requestType->second == U("LOAD_SERIES"))
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
      emit InvokeProgress(20, {"incoming series request ..."});
      emit InvokeUpdateDcmMeta(dto);
      // tasks
      std::vector<pplx::task<std::string>> tasks;

      if (dto.seriesUIDList.size() > 0)
      {
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
            return errorResponse;
          }
        }
      }
      emit InvokeProgress(40, {"download series ..."});
      try
      {
        auto joinTask = pplx::when_all(begin(tasks), end(tasks));
        auto filePathList = joinTask.then([&](std::vector<std::string> filePathList) {
          emit InvokeLoadData(filePathList);
          emit InvokeProgress(40, {""});
        });
      }
      catch (const mitk::Exception &exception)
      {
        MITK_INFO << exception.what();
        return errorResponse;
      }
    }
  }
  else
  {
    MITK_INFO << "no requestType parameter was provided";
  }

  return errorResponse;
}

web::http::http_response DicomWebRequestHandler::HandlePost(const web::uri &uri,
                                                            const web::json::value &data,
                                                            const mitk::RESTUtil::ParamMap &headers)
{
  if (!data.is_null()) // avoid unused warning
    MITK_INFO << "data was not null";

  auto query = web::uri(uri).query();

  auto contentType = headers.find(U("Content-Type"));
  if ((contentType->second.find(U("multipart/form-data")) != std::string::npos) ||
        ((contentType->second.find(U("multipart/related")) != std::string::npos) &&
        (contentType->second.find(U("application/dicom")) != std::string::npos)))
  {
  }

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
