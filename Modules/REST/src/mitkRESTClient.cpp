/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include <mitkExceptionMacro.h>
#include <mitkRESTClient.h>
#include <mitkRESTUtil.h>

#include <cpprest/filestream.h>
#include <cpprest/http_client.h>
#include "mitkRestRedirectException.h"

using http_client = web::http::client::http_client;
using http_request = web::http::http_request;
using http_response = web::http::http_response;
using methods = web::http::methods;
using status_codes = web::http::status_codes;
using file_buffer = concurrency::streams::file_buffer<uint8_t>;
using streambuf = concurrency::streams::streambuf<uint8_t>;

mitk::RESTClient::RESTClient()
{
  m_ClientConfig.set_validate_certificates(false);
}

mitk::RESTClient::~RESTClient() {}

bool mitk::RESTClient::CheckResponseContentType(web::http::http_response &response)
{
  auto status = response.status_code();

  if (status_codes::OK != status)
  {
    MITK_WARN << "Status: " << status;
    MITK_WARN << "Response: " << mitk::RESTUtil::convertToUtf8(response.to_string());
    mitkThrow() << std::to_string(status);
  }

  auto requestContentType = response.headers().content_type();
  MITK_DEBUG << "Content Type: " << mitk::RESTUtil::convertToUtf8(requestContentType);
  MITK_DEBUG << "Body: " << mitk::RESTUtil::convertToUtf8(response.to_string());
  if (requestContentType.find(U("json")) != std::wstring::npos)
  {
    MITK_DEBUG << "Caution! The given response content type was '" << mitk::RESTUtil::convertToUtf8(requestContentType)
               << "' but contains 'json'. So we awesome the answer actually contains a JSON message.";
    response.headers().set_content_type(U("application/json"));
    return true;
  }
  return false;
}

pplx::task<web::json::value> mitk::RESTClient::Get(const web::uri &uri,
                                                   const std::map<utility::string_t, utility::string_t> headers,
                                                   const bool useSystemProxy)
{
  if (useSystemProxy)
  {
    auto proxy = web::web_proxy(web::web_proxy::web_proxy_mode::use_auto_discovery);
    m_ClientConfig.set_proxy(proxy);
  }
  auto client = new http_client(uri, m_ClientConfig);
  http_request request;

  for (auto param : headers)
  {
    request.headers().add(param.first, param.second);
  }

  return client->request(request).then([=](pplx::task<web::http::http_response> responseTask) {
    try
    {
      auto response = responseTask.get();

      bool isjson = CheckResponseContentType(response);
      if (!isjson)
      {
        web::json::value dummy;
        return dummy;
      }
      return response.extract_json().get();
    }
    catch (const std::exception &e)
    {
      MITK_INFO << e.what();
      mitkThrow() << "Getting response went wrong: " << e.what();
    }
  });
}

pplx::task<web::json::value> mitk::RESTClient::Get(const web::uri &uri,
                                                   const utility::string_t &filePath,
                                                   const std::map<utility::string_t, utility::string_t> headers,
                                                   const bool useSystemProxy)
{
  if (useSystemProxy)
  {
    auto proxy = web::web_proxy(web::web_proxy::web_proxy_mode::use_auto_discovery);
    m_ClientConfig.set_proxy(proxy);
  }
  auto client = new http_client(uri, m_ClientConfig);
  auto fileBuffer = std::make_shared<concurrency::streams::streambuf<uint8_t>>();
  http_request request;

  for (auto param : headers)
  {
    request.headers().add(param.first, param.second);
  }

    // Open file stream for the specified file path
    return file_buffer::open(filePath, std::ios::out)
        .then([=](streambuf outFile) -> pplx::task<http_response> {
        *fileBuffer = outFile;
        return client->request(request);
        })
        // Write the response body into the file buffer
        .then([=](http_response response) -> pplx::task<size_t> {
        auto status = response.status_code();
        if (status_codes::OK == status)
        {
            return response.body().read_to_end(*fileBuffer);
        }

        else if (status_codes::SeeOther == status)
        {
            auto response_headers = response.headers();
            auto redirect_uri = response_headers[U("Location")];
            mitkThrowException(mitk::RestRedirectException) << mitk::RESTUtil::convertToUtf8(redirect_uri);
        }
        else
        {
            MITK_INFO << status;
            MITK_INFO << mitk::RESTUtil::convertToUtf8(response.to_string());
            mitkThrow() << mitk::RESTUtil::convertToUtf8(response.to_string());
        }
        })
        // Close the file buffer
        .then([=](size_t) { return fileBuffer->close(); })
        // Return empty JSON object
        .then([=]() { return web::json::value(); });
}

pplx::task<web::json::value> mitk::RESTClient::Put(const web::uri &uri,
                                                   const web::json::value *content,
                                                   const bool useSystemProxy)
{
  if (useSystemProxy)
  {
    auto proxy = web::web_proxy(web::web_proxy::web_proxy_mode::use_auto_discovery);
    m_ClientConfig.set_proxy(proxy);
  }

  auto client = new http_client(uri, m_ClientConfig);
  http_request request(methods::PUT);

  if (nullptr != content)
    request.set_body(*content);

  return client->request(request).then([=](pplx::task<http_response> responseTask) {
    try
    {
      auto response = responseTask.get();

      bool isjson = CheckResponseContentType(response);
      if (!isjson)
      {
        web::json::value dummy;
        return dummy;
      }
      return response.extract_json().get();
    }
    catch (std::exception &e)
    {
      MITK_INFO << e.what();
      mitkThrow() << "Getting response went wrong";
    }
  });
}

pplx::task<web::json::value> mitk::RESTClient::Post(const web::uri &uri,
                                                    const std::map<utility::string_t, utility::string_t> content,
                                                    const std::map<utility::string_t, utility::string_t> headers,
                                                    const bool useSystemProxy)
{
  utf8string contentString = "";
  for (auto const& element : content)
  {
    utf8string key = utility::conversions::to_utf8string(element.first);
    utf8string value = utility::conversions::to_utf8string(element.second);
    contentString.append(key + "=" + value + "&");
  }
  MITK_INFO << contentString;
  //auto client = http_client(uri, m_ClientConfig);
  http_request request;
  request.set_body(contentString, utf8string("application/x-www-form-urlencoded; charset=utf-8"));
  request.set_method(methods::POST);
  for (auto param : headers)
  {
    request.headers().add(param.first, param.second);
  }

  return ExecutePost(uri, request, useSystemProxy);
}

pplx::task<web::json::value> mitk::RESTClient::Post(const web::uri &uri,
                                                    const std::vector<unsigned char> *content,
                                                    const std::map<utility::string_t, utility::string_t> headers,
                                                    const bool useSystemProxy)
{
  auto request = InitRequest(headers);
  request.set_method(methods::POST);

  if (nullptr != content)
    request.set_body(*content);

  return ExecutePost(uri, request, useSystemProxy);
}

pplx::task<web::json::value> mitk::RESTClient::Post(const web::uri &uri,
                                                    const web::json::value *content,
                                                    const std::map<utility::string_t, utility::string_t> headers,
                                                    const bool useSystemProxy)
{
  auto request = InitRequest(headers);
  request.set_method(methods::POST);

  if (nullptr != content)
    request.set_body(*content);

  return ExecutePost(uri, request, useSystemProxy);
}

http_request mitk::RESTClient::InitRequest(const std::map<utility::string_t, utility::string_t> headers)
{
  http_request request;

  for (auto param : headers)
  {
    request.headers().add(param.first, param.second);
  }
  return request;
}

pplx::task<web::json::value> mitk::RESTClient::ExecutePost(const web::uri &uri,
                                                           http_request request,
                                                           const bool useSystemProxy)
{
  if (useSystemProxy)
  {
    auto proxy = web::web_proxy(web::web_proxy::web_proxy_mode::use_auto_discovery);
    m_ClientConfig.set_proxy(proxy);
  }

  auto client = http_client(uri, m_ClientConfig);
    return client.request(request).then([=](pplx::task<http_response> responseTask) {
        try
        {
        auto response = responseTask.get();

        bool isjson = CheckResponseContentType(response);
        if (!isjson)
        {
            web::json::value dummy;
            return dummy;
        }
        return response.extract_json().get();
        }
        catch (std::exception &e)
        {
        MITK_INFO << e.what();
        mitkThrow() << e.what();
        }
    });
}
