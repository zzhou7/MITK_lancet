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

//#include <mitkQtPythonService.h>

class mitkQtPythonTestSuite : public mitk::TestFixture
{
  CPPUNIT_TEST_SUITE(mitkQtPythonTestSuite);
  MITK_TEST(TestEvaluateOperationWithResult);
  MITK_TEST(TestExecuteStatement);
  MITK_TEST(TestSettingVariable);
  MITK_TEST(TestSettingVariableAndUseIt);
  MITK_TEST(TestRunningScript);
  MITK_TEST(TestGetVariableStack);
  MITK_TEST(TestDoesVariableExist_True);
  MITK_TEST(TestDoesVariableExist_False);
  CPPUNIT_TEST_SUITE_END();

public:

  void setUp() 
  { 
    mitk::IPythonService::ForceLoadModule();
  }

 void TestEvaluateOperationWithResult()
  {
    std::string result = "";
    us::ModuleContext *context = us::GetModuleContext();
    std::string filter = "(Name=QtPythonService)";
    auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

    if (!m_PythonServiceRefs.empty())
    {
      mitk::IPythonService *m_PythonService =
        dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));
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
    std::string filter = "(Name=QtPythonService)";
    auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

    if (!m_PythonServiceRefs.empty())
    {
      mitk::IPythonService *m_PythonService =
        dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

      try
      {
        std::string result = m_PythonService->Execute("print('Hello')");
        // std::string result = "None";
        CPPUNIT_ASSERT_MESSAGE("Testing if executing a statement works", result == "");
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
    std::string filter = "(Name=QtPythonService)";
    auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

    if (!m_PythonServiceRefs.empty())
    {
      mitk::IPythonService *m_PythonService =
        dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

      try
      {
        std::string result = m_PythonService->Execute("number = 5");
        CPPUNIT_ASSERT_MESSAGE("Testing if initializing a variable works", result == "");
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
    std::string filter = "(Name=QtPythonService)";
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
    std::string filter = "(Name=QtPythonService)";
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

   void TestGetVariableStack() 
   {
     us::ModuleContext *context = us::GetModuleContext();
     std::string filter = "(Name=QtPythonService)";
     auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

     if (!m_PythonServiceRefs.empty())
     {
       mitk::IPythonService *m_PythonService =
         dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

        try
       {
         auto list = m_PythonService->GetVariableStack();
         if (!(list.size() > 0))
         {
           CPPUNIT_FAIL("Failed to get variable stack");
         }
         //for (mitk::PythonVariable var : list)
         //{
         //  MITK_INFO << var.m_Name << ", " << var.m_Value << ", " << var.m_Type;
         //}
       }
       catch (const mitk::Exception &e)
       {
         MITK_ERROR << e.GetDescription();
         CPPUNIT_FAIL("Error in getting variable stack");
         return;
       }       
     }
     else
     {
       CPPUNIT_FAIL("No Service Reference found");
     }
   }

   void TestDoesVariableExist_True() 
   {
     us::ModuleContext *context = us::GetModuleContext();
     std::string filter = "(Name=QtPythonService)";
     auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

     if (!m_PythonServiceRefs.empty())
     {
       mitk::IPythonService *m_PythonService =
         dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

       m_PythonService->Execute("existingVariable ='This variable exists'");

       bool variableExists = m_PythonService->DoesVariableExist("existingVariable");
       CPPUNIT_ASSERT_MESSAGE("Testing if an existing variable is recognized", variableExists == true);
     }
     else
     {
       CPPUNIT_FAIL("No Service Reference found");
     }
   }

   void TestDoesVariableExist_False()
   {
     us::ModuleContext *context = us::GetModuleContext();
     std::string filter = "(Name=QtPythonService)";
     auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

     if (!m_PythonServiceRefs.empty())
     {
       mitk::IPythonService *m_PythonService =
         dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));

       bool variableExists = m_PythonService->DoesVariableExist("nonExistingVariable");
       CPPUNIT_ASSERT_MESSAGE("Testing if an not existing variable is not recognized", variableExists == false);
     }
     else
     {
       CPPUNIT_FAIL("No Service Reference found");
     }
   }
};

MITK_TEST_SUITE_REGISTRATION(mitkQtPython)
