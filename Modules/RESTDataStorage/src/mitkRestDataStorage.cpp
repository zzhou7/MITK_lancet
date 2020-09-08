#include "mitkRestDataStorage.h"

#include "mitkDataStorageManager.h"
#include "mitkRESTServer.h"
#include "mitkIRESTManager.h"
#include "mitkDataStorage.h"
#include "mitkUtilMethods.h"
#include <cpprest/http_listener.h>
#include <string>

using string = std::string;
using http_request = web::http::http_request;

string mitk::RESTDataStorage::Get(char scope, char propertynames)
{
  mitk::RESTServer RESTServer(uri);
  RESTServer.HandleRequest();

  mitk::DataStorageManager DataStorageManager();
  return DataStorageManager().GetAllNodeURLs(scope, propertynames);
};

string mitk::RESTDataStorage::Get(char node)
{
  mitk::RESTServer RESTServer(uri);
  RESTServer.HandleRequest();

  mitk::mitkUtilMethods dataNode;
  if (dataNode.Exists(node) == false)
  {
      return("Error");
  }

  mitk::DataStorageManager DataStorageManager();
  return DataStorageManager().GetData(node);
};

string mitk::RESTDataStorage::Get(char node, char propertynames)
{
  mitk::RESTServer RESTServer(uri);
  RESTServer.HandleRequest();


  mitk::mitkUtilMethods dataNode;
  if (dataNode.Exists(node) == false)
  {
      return("Error");
  }

  mitk::DataStorageManager DataStorageManager();
  return DataStorageManager().GetAllChildrenNodeURLs(node, propertynames);
}

string mitk::RESTDataStorage::Get(char node, string context, char scope)
{
  mitk::RESTServer RESTServer(uri);
  RESTServer.HandleRequest();

  mitk::mitkUtilMethods dataNode;
  if (dataNode.Exists(node) == false)
  {
      return("Error");
  }

  mitk::DataStorageManager DataStorageManager();
  return DataStorageManager().GetNodePropertyURLs(node, context, scope);
}

string mitk::RESTDataStorage::Get(string context, char scope, char propertynames)
{
    mitk::RESTServer RESTServer(uri);
    RESTServer.HandleRequest();

    mitk::DataStorageManager DataStorageManager();
    return DataStorageManager().GetPropertynames(context, scope, propertynames);
}

