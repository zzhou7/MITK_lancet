#include "mitkRestDataStorageServer.h"

#include "mitkIOUtil.h"
#include "mitkRESTUtil.h"

void mitk::mitkRestDataStorageServer::StartServer()
    {
      utility::string_t path = U("/inject");

      auto pathEnv = std::getenv("SERVER_PATH");
      if (pathEnv)
      {
        path = mitk::RESTUtil::convertToTString(std::string(pathEnv));
      }

      utility::string_t port = U(":4040");
      utility::string_t address = U("http://+");
      if (m_HostURL == U("http://localhost"))
        address = m_HostURL;

      address.append(port);
      address.append(path);
      // Setup listening server
      m_ManagerService->ReceiveRequest(address, m_RequestHandler);
      MITK_INFO << "Listening for requests at: " << utility::conversions::to_utf8string(address);
    }
