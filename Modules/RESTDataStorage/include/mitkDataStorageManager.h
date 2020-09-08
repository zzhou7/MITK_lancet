#ifndef DATASTORAGE_H
#define DATASTORAGE_H

#include <string>

using string = std::string;

namespace mitk
{
class DataStorageManager
{
public:
  string GetAllNodeURLs(char scope, char propertynames);
  string GetData(char node);
  string GetAllChildrenNodeURLs(char node, char propertynames);
  string GetNodePropertyURLs(char node, string context, char scope);
  string GetPropertynames(string context, char scope, char propertynames)
};
}

#endif // DATASTORAGE_H
