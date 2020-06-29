/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#ifndef mitkQtPythonActivator_h
#define mitkQtPythonActivator_h

// Microservices
#include <usModuleActivator.h>
#include "usModuleContext.h"
#include "mitkQtPythonService.h"
#include <usServiceRegistration.h>

namespace mitk
{
    ///
    /// installs the PythonService
    /// runs all initial commands (setting env paths etc)
    ///
    class QtPythonActivator : public us::ModuleActivator
    {
    public:

        void Load(us::ModuleContext* context) override
        {
          MITK_DEBUG << "QtPythonActivator::Load";
          // Registering QtPythonService as MicroService
          //m_PythonService = itk::SmartPointer<mitk::QtPythonService>(new QtPythonService());

          //us::ServiceProperties _PythonServiceProps;
          //_PythonServiceProps["Name"] = std::string("QtPythonService");
          //_PythonServiceProps["service.ranking"] = int(1);

          //m_PythonServiceRegistration = context->RegisterService<mitk::IPythonService>(m_PythonService.GetPointer(), _PythonServiceProps);
        }

        void Unload(us::ModuleContext*) override
        {
          //MITK_DEBUG("QtPythonActivator") << "QtPythonActivator::Unload";
          //MITK_DEBUG("QtPythonActivator") << "m_PythonService GetReferenceCount " << m_PythonService->GetReferenceCount();
          //m_PythonServiceRegistration.Unregister();
          //m_PythonService->Delete();
          //MITK_DEBUG("QtPythonActivator") << "m_PythonService GetReferenceCount " << m_PythonService->GetReferenceCount();
        }

        ~QtPythonActivator() override
        {
        }

    private:
        itk::SmartPointer<mitk::QtPythonService> m_PythonService;
        us::ServiceRegistration<QtPythonService> m_PythonServiceRegistration;
    };
}

US_EXPORT_MODULE_ACTIVATOR(mitk::QtPythonActivator)
#endif
