#ifndef RESTDATASTORAGE_H
#define RESTDATASTORAGE_H

#include <string>

using string = std::string;

namespace mitk
{
class RESTDataStorage
{
public:
  string Get(char scope, char propertynames);
  string Get(char node);
  string Get(char node, char propertynames);
  string Get(char node, string context, char scope);
  string Get(string context, char scope, char property);

  string Post();
  string Post(char node);

  void Put(char node);
  void Put(char property, char context, char scope);

  void Delete(char node);
};
}

#endif // RESTDATASTORAGE_H
