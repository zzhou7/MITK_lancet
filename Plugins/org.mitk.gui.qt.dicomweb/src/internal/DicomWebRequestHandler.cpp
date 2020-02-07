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
  m_DicomWeb = mitk::DICOMweb(pacsURI + U("/dcm4chee-arc/aets/DCM4CHEE/"));
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
  headers.size();

  MITK_INFO << "Incoming notify";
  if (method == web::http::methods::GET)
  {
    return HandleGet(uri, data);
  }
  else if (method == web::http::methods::PUT)
  {
    return HandlePut(uri, data);
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
  if (uri.to_string() != U("/inject")) // avoid unused warning
    MITK_INFO << "no inject path";

  emit InvokeProgress(20, {"display graph and query structured report"});

  if (data == web::json::value())
  {
    MITK_INFO << "no data in body";
    web::http::http_response response(web::http::status_codes::BadRequest);
    response.set_body(U("No data in body of request")); // TODO: include method name
    return response;
  }
  MITK_INFO << mitk::RESTUtil::convertToUtf8(data.serialize());
  DicomDTO dto = ExtractDTO(data);
  MITK_INFO << mitk::RESTUtil::convertToUtf8(dto.imageSeriesUID);

  emit InvokeSimilarityGraph(dto.simScoreArray, dto.minSliceStart);
  emit InvokeUpdateDcmMeta(dto);

  mitk::RESTUtil::ParamMap seriesInstancesParams;

  seriesInstancesParams.insert(mitk::RESTUtil::ParamMap::value_type(U("StudyInstanceUID"), dto.studyUID));
  seriesInstancesParams.insert(mitk::RESTUtil::ParamMap::value_type(U("SeriesInstanceUID"), dto.srSeriesUID));
  seriesInstancesParams.insert(
    mitk::RESTUtil::ParamMap::value_type(U("includefield"),
                                         U("0040A375"))); // Current Requested Procedure Evidence Sequence
  try
  {
    MITK_INFO << "send qido request";
    m_DicomWeb.SendQIDO(seriesInstancesParams).then([=](web::json::value jsonResult) {
      auto firstResult = jsonResult[0];
      auto actualListKey = firstResult.at(U("0040A375"))
                             .as_object()
                             .at(U("Value"))
                             .as_array()[0]
                             .as_object()
                             .at(U("00081115"))
                             .as_object()
                             .at(U("Value"))
                             .as_array();

      MITK_INFO << "received qido response";
      utility::string_t segSeriesUIDA = {};
      utility::string_t segSeriesUIDB = {};
      utility::string_t imageSeriesUID = {};

      for (unsigned int index = 0; index < actualListKey.size(); index++)
      {
        auto element = actualListKey[index].as_object();
        // get SOP class UID
        auto innerElement = element.at(U("00081199")).as_object().at(U("Value")).as_array()[0];
        auto sopClassUID = innerElement.at(U("00081150")).as_object().at(U("Value")).as_array()[0].as_string();

        auto seriesUID = element.at(U("0020000E")).as_object().at(U("Value")).as_array()[0].as_string();

        if (sopClassUID == U("1.2.840.10008.5.1.4.1.1.66.4")) // SEG
        {
          if (segSeriesUIDA.length() == 0)
          {
            segSeriesUIDA = seriesUID;
          }
          else
          {
            segSeriesUIDB = seriesUID;
          }
        }
        else if (sopClassUID == U("1.2.840.10008.5.1.4.1.1.2")) // CT
        {
          imageSeriesUID = seriesUID;
        }
      }

      emit InvokeProgress(10, {"load composite context of structured report"});
      MITK_INFO << "image series UID " << mitk::RESTUtil::convertToUtf8(imageSeriesUID);
      MITK_INFO << "seg A series UID " << mitk::RESTUtil::convertToUtf8(segSeriesUIDA);
      MITK_INFO << "seg B series UID " << mitk::RESTUtil::convertToUtf8(segSeriesUIDB);

      MITK_INFO << "Load related dicom series ...";

      std::string folderPathSeries = mitk::IOUtil::CreateTemporaryDirectory("XXXXXX", m_DownloadDir) + "/";

      std::string pathSegA = mitk::IOUtil::CreateTemporaryDirectory("XXXXXX", m_DownloadDir) + "/";
      std::string pathSegB = mitk::IOUtil::CreateTemporaryDirectory("XXXXXX", m_DownloadDir) + "/";

      auto folderPathSegA = utility::conversions::to_string_t(pathSegA);
      auto folderPathSegB = utility::conversions::to_string_t(pathSegB);

      std::vector<pplx::task<std::string>> tasks;

      auto imageSeriesTask =
        m_DicomWeb.SendWADO(utility::conversions::to_string_t(folderPathSeries), dto.studyUID, imageSeriesUID);
      auto segATask = m_DicomWeb.SendWADO(folderPathSegA, dto.studyUID, segSeriesUIDA);
      auto segBTask = m_DicomWeb.SendWADO(folderPathSegB, dto.studyUID, segSeriesUIDB);
      tasks.push_back(imageSeriesTask);
      tasks.push_back(segATask);
      tasks.push_back(segBTask);

      auto joinTask = pplx::when_all(begin(tasks), end(tasks));
      auto filePathList = joinTask.then([&](std::vector<std::string> filePathList) {
        emit InvokeProgress(50, {"load dicom files from disk"});
        emit InvokeLoadDataSegDicomWeb(filePathList);
      });
    });
  }
  catch (mitk::Exception &e)
  {
    MITK_ERROR << e.what();
  }
  web::http::http_response response(web::http::status_codes::InternalError);
  response.set_body(U("Something went wrong while processing the request.")); // TODO: include method name
  return response;
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