/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef __mitkLabel_H_
#define __mitkLabel_H_

#include "MitkMultilabelExports.h"
#include <mitkColorProperty.h>
#include <mitkPropertyList.h>
#include <mitkVector.h>

namespace mitk
{
  //##
  //##Documentation
  //## @brief A data structure describing a label.
  //## @ingroup Data
  //##
  class MITKMULTILABEL_EXPORT Label : public itk::Object, public IPropertyProvider
  {
  public:
    mitkClassMacroItkParent(Label, itk::Object);
    itkFactorylessNewMacro(Self);

    typedef unsigned short PixelType;

    /// The maximum value a label can get: Since the value is of type unsigned short MAX_LABEL_VALUE = 65535
    static const PixelType MAX_LABEL_VALUE;

    itkSetMacro(Locked, bool);
    itkGetConstMacro(Locked, bool);
    itkBooleanMacro(Locked);

    itkSetMacro(Visible, bool);
    itkGetConstMacro(Visible, bool);
    itkBooleanMacro(Visible);

    itkSetMacro(Opacity, float);
    itkGetConstMacro(Opacity, float);

    void SetName(const std::string &name);
    const std::string& GetName() const;

    itkSetMacro(Color, mitk::Color);
    itkGetConstReferenceMacro(Color, mitk::Color);

    /** Returns the center of mass for a given time step.
     *@param t Timestep for which the information is requested.
     *@pre t must be a valid time step.*/
    mitk::Point3D GetCenterOfMassCoordinates(TimeStepType t = 0) const;

    /** Returns the number of pixels of the label for a given time step.
     *@param t Timestep for which the information is requested.
     *@pre t must be a valid time step.*/
    itk::SizeValueType GetNumberOfPixels(TimeStepType t = 0) const;

    PixelType GetValue() const;

    BaseProperty::ConstPointer GetConstProperty(const std::string& propertyKey,
      const std::string& contextName = "",
      bool fallBackOnDefaultContext = true) const override;

    std::vector<std::string> GetPropertyKeys(const std::string& contextName = "",
      bool includeDefaultContext = false) const override;

    std::vector<std::string> GetPropertyContextNames() const override;

  protected:
    mitkCloneMacro(Self);

    Label();
    ~Label() override;
    Label(const Label& other);

    using CenterVectorType = std::vector<mitk::Point3D>;
    using SizeVectorType = std::vector<itk::SizeValueType>;

    //The following setters are only available via friend access (MultiLabelImage),
    //as changes are managed by the friend class to ensure a valid state of
    //the MultiLabelImage instance that owns this label.
    friend class MultiLabelImage;
    void SetCenterOfMassCoordinates(const CenterVectorType& centers);
    void SetNumberOfVoxels(const SizeVectorType& numbers);
    void SetValue(PixelType pixelValue);

    void PrintSelf(std::ostream &os, itk::Indent indent) const override;

  private:
    bool m_Locked = false;
    bool m_Visible = true;
    float m_Opacity = 1.0;
    std::string m_Name;
    mitk::Color m_Color;
    PixelType m_Value;
    CenterVectorType m_Centers;
    SizeVectorType m_NumberOfVoxels;
  };

  /**
  * @brief Equal A function comparing two labels for beeing equal in data
  *
  * @ingroup MITKTestingAPI
  *
  * Following aspects are tested for equality:
  *  - Lebel equality via Equal-PropetyList
  *
  * @param rightHandSide An image to be compared
  * @param leftHandSide An image to be compared
  * @param eps Tolarence for comparison. You can use mitk::eps in most cases.
  * @param verbose Flag indicating if the user wants detailed console output or not.
  * @return true, if all subsequent comparisons are true, false otherwise
  */
  MITKMULTILABEL_EXPORT bool Equal(const mitk::Label &leftHandSide,
                                   const mitk::Label &rightHandSide,
                                   ScalarType eps,
                                   bool verbose);

} // namespace mitk

#endif // __mitkLabel_H_
