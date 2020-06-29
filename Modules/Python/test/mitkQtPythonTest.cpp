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

class mitkQtPythonTestSuite : public mitk::TestFixture
{
  CPPUNIT_TEST_SUITE(mitkQtPythonTestSuite);
  MITK_TEST(TestPython);
  CPPUNIT_TEST_SUITE_END();

public:

  void TestPython()
  {
    std::string result = "";
    us::ModuleContext *context = us::GetModuleContext();
    std::string filter = "(Name=QtPythonService)";
    auto m_PythonServiceRefs = context->GetServiceReferences<mitk::IPythonService>(filter);

    if (!m_PythonServiceRefs.empty())
    {
      mitk::IPythonService *m_PythonService = dynamic_cast<mitk::IPythonService *>(context->GetService<mitk::IPythonService>(m_PythonServiceRefs.front()));
      mitk::IPythonService::ForceLoadModule();

      result = m_PythonService->Execute("5+5", mitk::IPythonService::EVAL_COMMAND);
    }

    MITK_TEST_CONDITION(result == "10", "Testing if running python code 5+5 results in 10");
  }
};

MITK_TEST_SUITE_REGISTRATION(mitkQtPython)
