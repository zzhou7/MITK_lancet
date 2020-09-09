#include ""

namespace mitk {
class  mitkRestDataStorageRequestHandler{
    web::http::http_response HandleGET(const web::uri &uri, const web::json::value &data);
};
}
