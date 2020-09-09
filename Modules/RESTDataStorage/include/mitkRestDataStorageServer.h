#include "mitkRESTUtil.h"
#include "mitkIRESTManager.h"
#include "mitkRestDataStorageRequestHandler.h"

namespace mitk {
class mitkRestDataStorageServer{
public:
  void StartServer();

  mitk::IRESTManager *m_ManagerService;
  mitk::mitkRestDataStorageRequestHandler *m_RequestHandler;

  utility::string_t m_restURL;
  utility::string_t m_HostURL;
};
}
