/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef RESTREDIRECTEXCEPTION_H_INCLUDED
#define RESTREDIRECTEXCEPTION_H_INCLUDED

#include <MitkRESTExports.h>
#include <mitkException.h>
#include "mitkExceptionMacro.h"

namespace mitk {
  /**Documentation
  * \brief An object of this class represents an exception to redirect a rest request.
  *
  */  class MITKREST_EXPORT RestRedirectException : public mitk::Exception
  {
  public:
    mitkExceptionClassMacro(RestRedirectException,mitk::Exception);
  };
} // namespace mitk
#endif
