/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include "mitkPythonService.h"
//#include <Python.h>
#include <mitkIOUtil.h>
//#include <QFile>
//#include <QDir>

#ifdef _DEBUG
  #undef _DEBUG
  #include <python.h>
  #define _DEBUG
#else
  #include <python.h>
#endif

//#ifdef _MSC_VER
//#  pragma warning(push)
//#  pragma warning(disable: 5208)
//#endif

//#include <PythonQt.h>

//#ifdef _MSC_VER
//#  pragma warning(pop)
//#endif

#include "PythonPath.h"
#include <vtkPolyData.h>
#include <mitkRenderingManager.h>
#include <mitkImageReadAccessor.h>
#include <mitkImageWriteAccessor.h>
//#include <QFileInfo>
//#include <QCoreApplication>
#include <itksys/SystemTools.hxx>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <mitkExceptionMacro.h>

#ifndef WIN32
#include <dlfcn.h>
#endif

typedef itksys::SystemTools ist;

mitk::PythonService::PythonService()
  : m_ItkWrappingAvailable(true),
    m_OpenCVWrappingAvailable(true),
    m_VtkWrappingAvailable(true),
    m_ErrorOccured(false)
{
  if (!Py_IsInitialized())
  {
    Py_Initialize();
  }
  std::string programPath = mitk::IOUtil::GetProgramPath();
  std::replace(programPath.begin(), programPath.end(), '\\', '/');
  programPath.append("/");
  MITK_INFO << programPath;
  std::string pythonCommand;
  pythonCommand.append("import site, sys\n");
  pythonCommand.append("sys.path.append('')\n");
  pythonCommand.append("sys.path.append('" + programPath + "')\n");
  pythonCommand.append("sys.path.append('" +std::string(EXTERNAL_DIST_PACKAGES) + "')\n");
  pythonCommand.append("\nsite.addsitedir('"+std::string(EXTERNAL_SITE_PACKAGES)+"')");
  if (PyRun_SimpleString(pythonCommand.c_str()) == -1)
  {
    MITK_ERROR << "Something went wrong in setting the path in Python";
  }

  PyObject *main = PyImport_AddModule("__main__");
  m_GlobalDictionary = PyModule_GetDict(main);
  m_LocalDictionary = m_GlobalDictionary;
}

mitk::PythonService::~PythonService()
{
  if (Py_IsInitialized())
  {
    PyGILState_Ensure();
    Py_FinalizeEx();
  }
}

void mitk::PythonService::AddRelativeSearchDirs(std::vector< std::string > dirs)
{
  
}

void mitk::PythonService::AddAbsoluteSearchDirs(std::vector< std::string > dirs)
{
  
}

std::string mitk::PythonService::Execute(const std::string &stdpythonCommand, int commandType, std::string projectPath)
{
  if (!Py_IsInitialized())
  {
    Py_Initialize();
  }
  std::string result = "";
  PyGILState_STATE gState = PyGILState_Ensure();
  if (projectPath!="")
  {
    this->SetProjectPath(projectPath);
    //As the Method SetProjectPath sets free the GIL we need to re-ensure it
    PyGILState_STATE gState = PyGILState_Ensure();
  }
  try
  {
    switch (commandType)
    {
      case IPythonService::SINGLE_LINE_COMMAND:
        commandType = Py_single_input;
        break;
      case IPythonService::MULTI_LINE_COMMAND:
        commandType = Py_file_input;
        break;
      case IPythonService::EVAL_COMMAND:
        commandType = Py_eval_input;
        break;
      default:
        commandType = Py_file_input;
    }
    PyObject* executionResult = PyRun_String(stdpythonCommand.c_str(), commandType, m_GlobalDictionary, m_LocalDictionary);
    if (executionResult)
    {
      PyObject *objectsRepresentation = PyObject_Repr(executionResult);
      const char *resultChar = PyUnicode_AsUTF8(objectsRepresentation);
      result = std::string(resultChar);
      m_ThreadState = PyEval_SaveThread();
    }
    else
    {
      mitkThrow() << "An error occured while running the Python code";
    }
  }
  catch (const mitk::Exception) 
  {
    m_ThreadState = PyEval_SaveThread();
    throw;
  }
  return result;
}

void mitk::PythonService::SetProjectPath(std::string projectPath) 
{
  try
  {
    std::string path = std::string(MITK_ROOT) + projectPath;
    std::string pythonCommand = "import sys\nsys.path.append('" + path + "')\n";
    this->Execute(pythonCommand.c_str());
  }
  catch (const mitk::Exception)
  {
    mitkThrow() << "An error occured setting the project path";
  }
}


void mitk::PythonService::ExecuteScript(const std::string &pythonScript, std::string projectPath)
{
  std::ifstream t(pythonScript.c_str());
  std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
  t.close();
  try
  {
    this->Execute(str.c_str(), MULTI_LINE_COMMAND, projectPath);
  }
  catch (const mitk::Exception)
  {
    throw;
  }
}

std::vector<mitk::PythonVariable> mitk::PythonService::GetVariableStack() const
{
  std::vector<mitk::PythonVariable> list;

  return list;
}

std::string mitk::PythonService::GetVariable(const std::string& name) const
{
  return NULL;
}

bool mitk::PythonService::DoesVariableExist(const std::string& name) const
{
  return NULL;
}

void mitk::PythonService::AddPythonCommandObserver(mitk::PythonCommandObserver *observer)
{
  
}

void mitk::PythonService::RemovePythonCommandObserver(mitk::PythonCommandObserver *observer)
{
 
}

void mitk::PythonService::NotifyObserver(const std::string &command)
{
  
}

bool mitk::PythonService::IsSimpleItkPythonWrappingAvailable()
{
  return NULL;
}

bool mitk::PythonService::CopyToPythonAsSimpleItkImage(mitk::Image *image, const std::string &stdvarName)
{
  return NULL;
}


//mitk::PixelType DeterminePixelType(const std::string& pythonPixeltype, unsigned long nrComponents, int dimensions)
//{
//  return NULL;
//}

mitk::Image::Pointer mitk::PythonService::CopySimpleItkImageFromPython(const std::string &stdvarName)
{
  return NULL;
}

bool mitk::PythonService::CopyToPythonAsCvImage( mitk::Image* image, const std::string& stdvarName )
{
  return NULL;
}


mitk::Image::Pointer mitk::PythonService::CopyCvImageFromPython( const std::string& stdvarName )
{
  return NULL;
}

//ctkAbstractPythonManager *mitk::PythonService::GetPythonManager()
//{
//  return NULL;
//}

mitk::Surface::Pointer mitk::PythonService::CopyVtkPolyDataFromPython( const std::string& stdvarName )
{
  return NULL;
}

bool mitk::PythonService::CopyToPythonAsVtkPolyData( mitk::Surface* surface, const std::string& stdvarName )
{
  return NULL;
}

bool mitk::PythonService::IsOpenCvPythonWrappingAvailable()
{
  return NULL;
}

bool mitk::PythonService::IsVtkPythonWrappingAvailable()
{
  return NULL;
}

bool mitk::PythonService::PythonErrorOccured() const
{
  return NULL;
}

