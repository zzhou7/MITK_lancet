/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#include <mitkCommon.h>
#include <usModuleContext.h>
#include <usServiceReference.h>
#include <usGetModuleContext.h>
#include <mitkIPythonService.h>
#include <mitkTestingMacros.h>
#include <mitkTestFixture.h>
#include <mitkStandardFileLocations.h>




class mitkPythonTestSuite : public mitk::TestFixture
{
  CPPUNIT_TEST_SUITE(mitkPythonTestSuite);
  
  MITK_TEST(TestEvaluateOperationWithResult);
  MITK_TEST(TestExecuteStatement);
  MITK_TEST(TestSettingVariable);
  MITK_TEST(TestSettingVariableAndUseIt);
  MITK_TEST(TestRunningScript);
  MITK_TEST(TestRunningScriptCallOtherScript);
  MITK_TEST(TestRunningScriptCallOtherScriptInSubfolder);
  CPPUNIT_TEST_SUITE_END();

public:



  void TestEvaluateOperationWithResult()
  {
    std::string result = "";
    us::ModuleContext *context = us::GetModuleContext();
    std::string filter = "(Name=PythonService)";
    auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

    if (!m_PythonServiceRefs.empty())
    {
      mitk::IPythonService *m_PythonService = dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));
      result = m_PythonService->Execute("5+5", mitk::IPythonService::EVAL_COMMAND);
      try
      {
        std::string result = m_PythonService->Execute("5+5", mitk::IPythonService::EVAL_COMMAND);
        CPPUNIT_ASSERT_MESSAGE("Testing if running python code 5+5 results in 10", result == "10");
      }
      catch (const mitk::Exception &e)
      {
        MITK_ERROR << e.GetDescription();
        CPPUNIT_FAIL("Error in Python Execution");
      }
    }
    else
    {
      CPPUNIT_FAIL("No Service Reference found");
    }
  }

    void TestExecuteStatement()
  {
    us::ModuleContext *context = us::GetModuleContext();
    std::string filter = "(Name=PythonService)";
    auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

    if (!m_PythonServiceRefs.empty())
    {
      mitk::IPythonService *m_PythonService =
        dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

      try
      {
        std::string result = m_PythonService->Execute("print('Hello')");
        //std::string result = "None";
        CPPUNIT_ASSERT_MESSAGE("Testing if executing a statement works", result == "None");
      }
      catch (const mitk::Exception &e)
      {
        MITK_ERROR << e.GetDescription();
        CPPUNIT_FAIL("Error in Python Execution");
      }
    }
    else
    {
      CPPUNIT_FAIL("No Service Reference found");
    }
  }

  void TestSettingVariable() 
  {
    us::ModuleContext *context = us::GetModuleContext();
    std::string filter = "(Name=PythonService)";
    auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

    if (!m_PythonServiceRefs.empty())
    {
      mitk::IPythonService *m_PythonService =
        dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

      try
      {
        std::string result = m_PythonService->Execute("number = 5");
        CPPUNIT_ASSERT_MESSAGE("Testing if initializing a variable works", result == "None");
      }
      catch (const mitk::Exception &e)
      {
        MITK_ERROR << e.GetDescription();
        CPPUNIT_FAIL("Error in Python Execution");
      }
    }
    else
    {
      CPPUNIT_FAIL("No Service Reference found");
    }
  }

  void TestSettingVariableAndUseIt() 
  {
    std::string result = "";
    us::ModuleContext *context = us::GetModuleContext();
    std::string filter = "(Name=PythonService)";
    auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

    if (!m_PythonServiceRefs.empty())
    {
      mitk::IPythonService *m_PythonService =
        dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

      try
      {
        result = m_PythonService->Execute("number = 5");
        result = m_PythonService->Execute("number+5", mitk::IPythonService::EVAL_COMMAND);
        CPPUNIT_ASSERT_MESSAGE("Testing if initializing a variable and using it works", result == "10");
      }
      catch (const mitk::Exception &e)
      {
        MITK_ERROR << e.GetDescription();
        CPPUNIT_FAIL("Error in Python Execution");
      }
    }
    else
    {
      CPPUNIT_FAIL("No Service Reference found");
    }
  }

  void TestRunningScript() 
  {
    us::ModuleContext *context = us::GetModuleContext();
    std::string filter = "(Name=PythonService)";
    auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

    if (!m_PythonServiceRefs.empty())
    {
      mitk::IPythonService *m_PythonService =
        dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

      try
      {
        std::string pythonFileName = "hello.py";
        std::string fileName = mitk::StandardFileLocations::GetInstance()->FindFile(
          pythonFileName.c_str(), "Modules/Python/test/hello_world_project");
        m_PythonService->ExecuteScript(fileName);
      }
      catch(const mitk::Exception &e)
      {
        MITK_ERROR << e.GetDescription();
        CPPUNIT_FAIL("Error in Python Execution for Script");
        return;
      }
    }
    else
    {
      CPPUNIT_FAIL("No Service Reference found");
    }
  }

    void TestRunningScriptCallOtherScript()
  {
    us::ModuleContext *context = us::GetModuleContext();
    std::string filter = "(Name=PythonService)";
    auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

    if (!m_PythonServiceRefs.empty())
    {
      mitk::IPythonService *m_PythonService =
        dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

      try
      {
        std::string pythonFileName = "call_hello.py";
        std::string fileName =
          mitk::StandardFileLocations::GetInstance()->FindFile(pythonFileName.c_str(), "Modules/Python/test/hello_world_project");
        m_PythonService->ExecuteScript(fileName);
      }
      catch (const mitk::Exception &e)
      {
        MITK_ERROR << e.GetDescription();
        CPPUNIT_FAIL("Error in Python Execution for Script");
        return;
      }
    }
    else
    {
      CPPUNIT_FAIL("No Service Reference found");
    }
  }

    void TestRunningScriptCallOtherScriptInSubfolder() 
    {
      us::ModuleContext *context = us::GetModuleContext();
      std::string filter = "(Name=PythonService)";
      auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

      if (!m_PythonServiceRefs.empty())
      {
        mitk::IPythonService *m_PythonService =
          dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

        try
        {
          std::string pythonFileName = "call_hello_in_subfolder.py";
          std::string fileName = mitk::StandardFileLocations::GetInstance()->FindFile(
            pythonFileName.c_str(), "Modules/Python/test/hello_world_project");
          m_PythonService->ExecuteScript(fileName);
        }
        catch (const mitk::Exception &e)
        {
          MITK_ERROR << e.GetDescription();
          CPPUNIT_FAIL("Error in Python Execution for Script");
          return;
        }
      }
      else
      {
        CPPUNIT_FAIL("No Service Reference found");
      }
   }
};

MITK_TEST_SUITE_REGISTRATION(mitkPython)
