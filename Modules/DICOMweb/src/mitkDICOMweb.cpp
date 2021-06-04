/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include "mitkDICOMweb.h"

mitk::DICOMweb::DICOMweb() {}

mitk::DICOMweb::DICOMweb(utility::string_t baseURI, bool useSystemProxy) : m_BaseURI(baseURI), m_UseSystemProxy(useSystemProxy)
{
  MITK_INFO << "base uri: " << mitk::RESTUtil::convertToUtf8(m_BaseURI);
  MITK_INFO << "use system proxy: " << m_UseSystemProxy;
  InitializeRESTManager();
}
void mitk::DICOMweb::UpdateUseSystemProxy(bool useSystemProxy)
{
  m_UseSystemProxy = useSystemProxy;
}

utility::string_t mitk::DICOMweb::CreateQIDOUri(mitk::RESTUtil::ParamMap map)
{
  MitkUriBuilder queryBuilder(m_BaseURI + U("rs/instances"));

  for (auto const &element : map)
  {
    queryBuilder.append_query(element.first, element.second);
  }

  return queryBuilder.to_string();
}

utility::string_t mitk::DICOMweb::CreateWADOUri(utility::string_t studyUID,
                                                utility::string_t seriesUID,
                                                utility::string_t instanceUID)
{
  MitkUriBuilder builder(m_BaseURI + U("wado"));
  builder.append_query(U("requestType"), U("WADO"));
  builder.append_query(U("studyUID"), studyUID);
  builder.append_query(U("seriesUID"), seriesUID);
  builder.append_query(U("objectUID"), instanceUID);
  builder.append_query(U("contentType"), U("application/dicom"));

  return builder.to_string();
}

utility::string_t mitk::DICOMweb::CreateSTOWUri(utility::string_t studyUID)
{
  MitkUriBuilder builder(m_BaseURI + U("rs/studies"));
  builder.append_path(studyUID);
  return builder.to_string();
}

pplx::task<void> mitk::DICOMweb::SendSTOW(utility::string_t filePath,
                                          utility::string_t studyUID)
{
  auto uri = CreateSTOWUri(studyUID);

  // this is the working stow-rs request which supports just one dicom file packed into a multipart message
  std::ifstream input(filePath, std::ios::binary);
  if (!input)
  {
    MITK_WARN << "could not read file to POST";
    return pplx::task<void>();
  }

  std::vector<unsigned char> result;
  std::vector<unsigned char> buffer;

  // Stop eating new lines in binary mode!!!
  input.unsetf(std::ios::skipws);

  input.seekg(0, std::ios::end);
  const std::streampos fileSize = input.tellg();
  input.seekg(0, std::ios::beg);

  MITK_INFO << fileSize << " bytes will be sent.";
  buffer.reserve(fileSize); // file size
  std::copy(
    std::istream_iterator<unsigned char>(input), std::istream_iterator<unsigned char>(), std::back_inserter(buffer));

  // in future more than one file should also be supported..
  std::string head = "";
  head += "\r\n--boundary";
  head += "\r\nContent-Type: " + mitk::RESTUtil::convertToUtf8(U("application/dicom")) + "\r\n\r\n";

  std::vector<unsigned char> bodyVector(head.begin(), head.end());

  std::string tail = "";
  tail += "\r\n--boundary--";

  result.insert(result.end(), bodyVector.begin(), bodyVector.end());
  result.insert(result.end(), buffer.begin(), buffer.end());
  result.insert(result.end(), tail.begin(), tail.end());

  mitk::RESTUtil::ParamMap headers;
  headers.insert(mitk::RESTUtil::ParamMap::value_type(
    U("Content-Type"), U("multipart/related; type=\"application/dicom\"; boundary=boundary")));

  try
  {
    return m_RESTManager->SendBinaryRequest(uri, mitk::IRESTManager::RequestType::Post, &result, headers, m_UseSystemProxy)
      .then([=](web::json::value result) {
        MITK_INFO << "after send";
        MITK_INFO << mitk::RESTUtil::convertToUtf8(result.serialize());
        result.is_null();
      });
  }
  catch (std::exception &e)
  {
    MITK_WARN << e.what();
  }

  return pplx::task<void>();
}

pplx::task<void> mitk::DICOMweb::SendWADO(utility::string_t filePath,
                                          utility::string_t studyUID,
                                          utility::string_t seriesUID,
                                          utility::string_t instanceUID,
                                          utility::string_t access_token)
{
  auto uri = CreateWADOUri(studyUID, seriesUID, instanceUID);

  mitk::RESTUtil::ParamMap headers;
  if (access_token != U(""))
  {
    headers.insert(mitk::RESTUtil::ParamMap::value_type(U("Cookie"), access_token));
  }

  // don't want return something
  try
  {
    return m_RESTManager
      ->SendJSONRequest(uri, mitk::IRESTManager::RequestType::Get, nullptr, headers, filePath, m_UseSystemProxy)
      .then([=](web::json::value result) { result.is_null(); });
  }
  catch (const mitk::Exception &e)
  {
    mitkThrow() << e.what();
  }
}

pplx::task<std::string> mitk::DICOMweb::SendWADO(utility::string_t folderPath,
                                                 utility::string_t studyUID,
                                                 utility::string_t seriesUID,
                                                 utility::string_t access_token)
{
  auto uri = m_BaseURI + U("rs/studies/") + studyUID + U("/series/") + seriesUID + U("?accept=application/zip");

  mitk::RESTUtil::ParamMap headers;
  if (access_token != U(""))
  {
    headers.insert(mitk::RESTUtil::ParamMap::value_type(U("Cookie"), access_token));
  }

  auto filePath = folderPath + seriesUID + U(".zip");
  try
  {
    auto task = m_RESTManager -> SendJSONRequest(uri, mitk::IRESTManager::RequestType::Get, nullptr, headers, filePath, m_UseSystemProxy)
      .then([=](web::json::value result) {
        result.is_null();
        auto filePathUtf8 = utility::conversions::to_utf8string(filePath);
        return filePathUtf8;
    });
    return task;
  }
  catch (const mitk::Exception &e)
  {
    mitkThrow() << e.what();
  }
}

pplx::task<web::json::value> mitk::DICOMweb::SendQIDO(mitk::RESTUtil::ParamMap map, utility::string_t access_token)
{
  auto uri = CreateQIDOUri(map);

  mitk::RESTUtil::ParamMap headers;
  if (access_token != U(""))
  {
    headers.insert(mitk::RESTUtil::ParamMap::value_type(U("Cookie"), access_token));
  }
  headers.insert(mitk::RESTUtil::ParamMap::value_type(U("Accept"), U("application/json")));
  return m_RESTManager->SendJSONRequest(
    uri, mitk::IRESTManager::RequestType::Get, nullptr, headers, {}, m_UseSystemProxy);
}

void mitk::DICOMweb::InitializeRESTManager()
{
  auto *context = us::GetModuleContext();
  auto managerRef = context->GetServiceReference<mitk::IRESTManager>();
  if (managerRef)
  {
    auto managerService = context->GetService(managerRef);
    if (managerService)
    {
      m_RESTManager = managerService;
    }
  }
}
