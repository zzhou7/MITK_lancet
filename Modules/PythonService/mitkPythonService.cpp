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
  PyGILState_STATE gState = PyGILState_Ensure();
  std::string programPath = mitk::IOUtil::GetProgramPath();
  std::replace(programPath.begin(), programPath.end(), '\\', '/');
  programPath.append("/");
  MITK_INFO << programPath;
  std::string pythonCommand;

  pythonCommand.append("import SimpleITK as sitk\n");
  pythonCommand.append("import SimpleITK._SimpleITK as _SimpleITK\n");
  pythonCommand.append("import numpy\n");

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
  m_ThreadState = PyEval_SaveThread();
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
  for (auto dir : dirs)
  {
    try
    {
      std::string path = std::string(MITK_ROOT) + dir;
      std::string pythonCommand = "import sys\nsys.path.append('" + path + "')\n";
      this->Execute(pythonCommand.c_str());
    }
    catch (const mitk::Exception)
    {
      mitkThrow() << "An error occured setting the relative project path";
    }
  }
}

void mitk::PythonService::AddAbsoluteSearchDirs(std::vector< std::string > dirs)
{
  for (auto dir : dirs)
  {
    try
    {
      std::string pythonCommand = "import sys\nsys.path.append('" + dir + "')\n";
      this->Execute(pythonCommand.c_str());
    }
    catch (const mitk::Exception)
    {
      mitkThrow() << "An error occured setting the absolute project path";
    }
  }
}

std::string mitk::PythonService::Execute(const std::string &stdpythonCommand, int commandType)
{
  if (!Py_IsInitialized())
  {
    Py_Initialize();
  }
  std::string result = "";

  PyGILState_STATE gState = PyGILState_Ensure();
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
      m_ErrorOccured = false;
    }
    else
    {
      m_ErrorOccured = true;
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


void mitk::PythonService::ExecuteScript(const std::string &pythonScript)
{
  std::ifstream t(pythonScript.c_str());
  std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
  t.close();
  try
  {
    this->Execute(str.c_str(), MULTI_LINE_COMMAND);
  }
  catch (const mitk::Exception)
  {
    throw;
  }
}

std::vector<mitk::PythonVariable> mitk::PythonService::GetVariableStack()
{
  std::vector<mitk::PythonVariable> list;
  PyGILState_STATE gState = PyGILState_Ensure();
  try
  {
    PyObject *dict = PyImport_GetModuleDict();
    PyObject *object = PyDict_GetItemString(dict, "__main__");
    if (!object)
    {
      mitkThrow() << "An error occured getting the Dictionary";
    }
    PyObject *dirMain = PyObject_Dir(object);
    PyObject *tempObject = nullptr;

    if (dirMain)
    {
      std::string name, attrValue, attrType;

      for (int i = 0; i < PyList_Size(dirMain); i++)
      {
        tempObject = PyList_GetItem(dirMain, i);
        if (!tempObject)
        {
          mitkThrow() << "An error occured getting an item from the dictionary";
        }
        PyObject *objectsRepresentation = PyObject_Repr(tempObject);
        const char *objectChar = PyUnicode_AsUTF8(objectsRepresentation);
        std::string name = std::string(objectChar);
        name = name.substr(1, name.size() - 2);
        tempObject = PyObject_GetAttrString(object, name.c_str());
        if (!tempObject)
        {
          mitkThrow() << "Could not get the attribute to determine type";
        }
        attrType = tempObject->ob_type->tp_name;

        PyObject *valueStringRepresentation = PyObject_Repr(tempObject);
        const char *valueChar = PyUnicode_AsUTF8(valueStringRepresentation);
        std::string attrValue = std::string(valueChar);

        mitk::PythonVariable var;
        var.m_Name = name;
        var.m_Value = attrValue;
        var.m_Type = attrType;
        list.push_back(var);
      }
    }
    m_ThreadState = PyEval_SaveThread();
  }
  catch (const mitk::Exception)
  {
    m_ThreadState = PyEval_SaveThread();
    throw;
  }
  return list;
}

std::string mitk::PythonService::GetVariable(const std::string& name)
{
  std::vector<mitk::PythonVariable> allVars;
  try
  {
    allVars = this->GetVariableStack();
  }
  catch (const mitk::Exception)
  {
    mitkThrow() << "Error getting the variable stack";
  }
  for (unsigned int i = 0; i < allVars.size(); i++)
  {
    if (allVars.at(i).m_Name == name)
      return allVars.at(i).m_Value;
  }

  return "";
}

bool mitk::PythonService::DoesVariableExist(const std::string& name)
{
  bool varExists = false;
  std::vector<mitk::PythonVariable> allVars;
  try
  {
    allVars = this->GetVariableStack();
  }
  catch (const mitk::Exception)
  {
    mitkThrow() << "Error getting the variable stack";
  }
 
  for (unsigned int i = 0; i < allVars.size(); i++)
  {
    if (allVars.at(i).m_Name == name)
    {
      varExists = true;
      break;
    }
  }

  return varExists;
}

void mitk::PythonService::AddPythonCommandObserver(mitk::PythonCommandObserver *observer)
{
  //only add observer if it isn't already in list of observers
  if (!(std::find(m_Observer.begin(), m_Observer.end(), observer) != m_Observer.end()))
  {
    m_Observer.push_back(observer);
  }
}

void mitk::PythonService::RemovePythonCommandObserver(mitk::PythonCommandObserver *observer)
{
  for (std::vector<mitk::PythonCommandObserver *>::iterator iter = m_Observer.begin(); iter != m_Observer.end(); ++iter)
  {
    if (*iter == observer)
    {
      m_Observer.erase(iter);
      break;
    }
  }
}

void mitk::PythonService::NotifyObserver(const std::string &command)
{
  for (int i = 0; i < m_Observer.size(); ++i)
  {
    m_Observer.at(i)->CommandExecuted(command);
  }
}


int mitk::PythonService::GetNumberOfObserver() 
{
  return m_Observer.size();
}

bool mitk::PythonService::IsSimpleItkPythonWrappingAvailable()
{
  return NULL;
}

bool mitk::PythonService::CopyToPythonAsSimpleItkImage(mitk::Image *image, const std::string &stdvarName)
{
  PyGILState_STATE gState = PyGILState_Ensure();
  //QString varName = QString::fromStdString(stdvarName);
  std::string varName = stdvarName;
  std::string command;
  unsigned int *imgDim = image->GetDimensions();
  int npy_nd = 1;

  // access python module
  PyObject *pyMod = PyImport_AddModule("__main__");
  // global dictionary
  PyObject *pyDict = PyModule_GetDict(pyMod);
  const mitk::Vector3D spacing = image->GetGeometry()->GetSpacing();
  const mitk::Point3D origin = image->GetGeometry()->GetOrigin();
  mitk::PixelType pixelType = image->GetPixelType();
  itk::ImageIOBase::IOPixelType ioPixelType = image->GetPixelType().GetPixelType();
  PyObject *npyArray = nullptr;
  mitk::ImageReadAccessor racc(image);
  void *array = const_cast<void *>(racc.GetData());

  mitk::Vector3D xDirection;
  mitk::Vector3D yDirection;
  mitk::Vector3D zDirection;
  const vnl_matrix_fixed<ScalarType, 3, 3> &transform =
    image->GetGeometry()->GetIndexToWorldTransform()->GetMatrix().GetVnlMatrix();

  mitk::Vector3D s = image->GetGeometry()->GetSpacing();

  // ToDo: Check if this is a collumn or row vector from the matrix.
  // right now it works but not sure for rotated geometries
  mitk::FillVector3D(xDirection, transform[0][0] / s[0], transform[0][1] / s[1], transform[0][2] / s[2]);
  mitk::FillVector3D(yDirection, transform[1][0] / s[0], transform[1][1] / s[1], transform[1][2] / s[2]);
  mitk::FillVector3D(zDirection, transform[2][0] / s[0], transform[2][1] / s[1], transform[2][2] / s[2]);

  // save the total number of elements here (since the numpy array is one dimensional)
  npy_intp *npy_dims = new npy_intp[1];
  npy_dims[0] = imgDim[0];

  /**
   * Build a string in the format [1024,1028,1]
   * to describe the dimensionality. This is needed for simple itk
   * to know the dimensions of the image
   */
  std::string dimensionString;
  dimensionString.append("[");
  dimensionString.append(std::to_string(imgDim[0]));
  for (unsigned i = 1; i < 3; ++i)
  // always three because otherwise the 3d-geometry gets destroyed
  // (relevant for backtransformation of simple itk image to mitk.
  {
    dimensionString.append(",");
    dimensionString.append(std::to_string(imgDim[i]));
    npy_dims[0] *= imgDim[i];
  }
  dimensionString.append("]");

  // the next line is necessary for vectorimages
  npy_dims[0] *= pixelType.GetNumberOfComponents();

  // default pixeltype: unsigned short
  NPY_TYPES npy_type = NPY_USHORT;
  std::string sitk_type = "sitkUInt8";
  if (ioPixelType == itk::ImageIOBase::SCALAR)
  {
    if (pixelType.GetComponentType() == itk::ImageIOBase::DOUBLE)
    {
      npy_type = NPY_DOUBLE;
      sitk_type = "sitkFloat64";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::FLOAT)
    {
      npy_type = NPY_FLOAT;
      sitk_type = "sitkFloat32";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::SHORT)
    {
      npy_type = NPY_SHORT;
      sitk_type = "sitkInt16";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::CHAR)
    {
      npy_type = NPY_BYTE;
      sitk_type = "sitkInt8";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::INT)
    {
      npy_type = NPY_INT;
      sitk_type = "sitkInt32";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::LONG)
    {
      npy_type = NPY_LONG;
      sitk_type = "sitkInt64";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::UCHAR)
    {
      npy_type = NPY_UBYTE;
      sitk_type = "sitkUInt8";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::UINT)
    {
      npy_type = NPY_UINT;
      sitk_type = "sitkUInt32";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::ULONG)
    {
      npy_type = NPY_LONG;
      sitk_type = "sitkUInt64";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::USHORT)
    {
      npy_type = NPY_USHORT;
      sitk_type = "sitkUInt16";
    }
  }
  else if (ioPixelType == itk::ImageIOBase::VECTOR || ioPixelType == itk::ImageIOBase::RGB ||
           ioPixelType == itk::ImageIOBase::RGBA)
  {
    if (pixelType.GetComponentType() == itk::ImageIOBase::DOUBLE)
    {
      npy_type = NPY_DOUBLE;
      sitk_type = "sitkVectorFloat64";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::FLOAT)
    {
      npy_type = NPY_FLOAT;
      sitk_type = "sitkVectorFloat32";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::SHORT)
    {
      npy_type = NPY_SHORT;
      sitk_type = "sitkVectorInt16";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::CHAR)
    {
      npy_type = NPY_BYTE;
      sitk_type = "sitkVectorInt8";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::INT)
    {
      npy_type = NPY_INT;
      sitk_type = "sitkVectorInt32";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::LONG)
    {
      npy_type = NPY_LONG;
      sitk_type = "sitkVectorInt64";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::UCHAR)
    {
      npy_type = NPY_UBYTE;
      sitk_type = "sitkVectorUInt8";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::UINT)
    {
      npy_type = NPY_UINT;
      sitk_type = "sitkVectorUInt32";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::ULONG)
    {
      npy_type = NPY_LONG;
      sitk_type = "sitkVectorUInt64";
    }
    else if (pixelType.GetComponentType() == itk::ImageIOBase::USHORT)
    {
      npy_type = NPY_USHORT;
      sitk_type = "sitkVectorUInt16";
    }
  }
  else
  {
    MITK_WARN << "not a recognized pixeltype";
    return false;
  }

  // creating numpy array
  import_array1(true);
  npyArray = PyArray_SimpleNewFromData(npy_nd, npy_dims, npy_type, array);

  // add temp array it to the python dictionary to access it in python code
  std::string statusString = varName + "_numpy_array";
  const int status =
    PyDict_SetItemString(pyDict, statusString.c_str(), npyArray);

  // sanity check
  if (status != 0)
    return false;

  std::string imageCommand = varName + " = sitk.Image(" + dimensionString + ",sitk." + sitk_type.c_str() + "," +
                             std::to_string(pixelType.GetNumberOfComponents()) + ")\n";
  command.append(imageCommand);
  std::string spacingCommand = varName + ".SetSpacing([" + std::to_string(spacing[0]) + "," +
                               std::to_string(spacing[1]) + "," + std::to_string(spacing[2]) + "])\n";
  command.append(spacingCommand);
  std::string originCommand = varName + ".SetOrigin([" + std::to_string(origin[0]) + "," + std::to_string(origin[1]) +
                              "," + std::to_string(origin[2]) + "])\n";
  command.append(originCommand);
  std::string directionCommand = varName + ".SetDirection([" + std::to_string(xDirection[0]) + "," +
                                 std::to_string(xDirection[1]) + "," + std::to_string(xDirection[2]) + "," +
                                 std::to_string(yDirection[0]) + "," + std::to_string(yDirection[1]) + "," +
                                 std::to_string(yDirection[2]) + "," + std::to_string(zDirection[0]) + "," +
                                 std::to_string(zDirection[1]) + "," + std::to_string(zDirection[2]) + "])\n";
  command.append(directionCommand);
  // directly access the cpp api from the lib
  std::string imageFromArray = "_SimpleITK._SetImageFromArray(" + varName + "_numpy_array," + varName + ")\n";
  command.append(imageFromArray);
  std::string deleteArray = "del " + varName + "_numpy_array";
  command.append(deleteArray);

  MITK_DEBUG("PythonService") << "Issuing python command " << command;
  m_ThreadState = PyEval_SaveThread();

  this->Execute(command);
  return true;
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
  return m_ErrorOccured;
}

