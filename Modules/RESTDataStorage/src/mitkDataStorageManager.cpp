#include "mitkDataStorageManager.h"
#include "mitkDataNode.h"
#include <string>

using string = std::string;


string mitk::DataStorageManager::GetAllNodeURLs(char scope, char propertynames)
{
    //ToDo: read .json with all Nodes
    //return NodeURLs
}

string mitk::DataStorageManager::GetData(char node)
{
    //ToDo: get data of this node
    //return NodeData
}

string mitk::DataStorageManager::GetAllChildrenNodeURLs(char node, char propertynames)
{
    //ToDo: get children node urls for specified node
    //return ChildrenNodeURL
}

string mitk::DataStorageManager::GetNodePropertyURLs(char node, string context, char scope)
{
    //ToDo: get all properties of specified nodes
    //return NodePropertyURLs
}

string mitk:: DataStorageManager::GetPropertynames(string context, char scope, char propertynames)
{
    mitk::DataNode DataNode();
    DataNode.GetProperty(&propertynames, context, true);
    //ToDo: get value of property
    //return PropertyValue
}
