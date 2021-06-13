/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef __mitkLabelProxyImage_H_
#define __mitkLabelProxyImage_H_

#include <mitkImage.h>
#include <mitkLabel.h>
#include <mitkWeakPointer.h>
#include <MitkMultilabelExports.h>

namespace mitk
{
  class MultiLabelImage;

  class MITKMULTILABEL_EXPORT InvalidLabelException : public Exception
  {
  public:
    mitkExceptionClassMacro(InvalidLabelException, Exception);
  };

  class MITKMULTILABEL_EXPORT InvalidMultiLabelImageException : public Exception
  {
  public:
    mitkExceptionClassMacro(InvalidMultiLabelImageException, Exception);
  };

  /** @brief LabelProxyImage is a dedicated image class that allows the image like read access
   * to a certain label content of a MultLabelImage.
   * The key idea of this class is to allow the interaction (read only) with a label of a
   * MultiLabelInstance as its label content would be stored in a normal (classical) segmentation
   * image; just containing the label and background. In difference to a normal image, the LabelProxyImage
   * generates the image data on demand; thus as long nobody has to access image data itself, no memory
   * is consumed to represent the label content. Other features like acessing the properties, time geometry
   * and alike are directly available (e.g. for the use with node predicates).
   * A LabelProxyInstalled is normaly owned by a MultiLableImage instance (parent), therefore you need an instance
   * for New(). The parent is the source of the label conent and information, further the parent takes care
   * to indicate if the LabelProxyImage is modified and its current label content is outdated (updates will also be
   * handled lazy).
  */
  //Implementation remarks:
  // - reimplementation should avoid handcrafted iteration (as it is currently done), itk filter
  // should be used to make use of threading.
  // - We have to check all virtual methods of superclasses. Most of them have to be looped through,
  // to allow the lazy behavior in many ocassions.
  // - It is not cleare yet, if the instance also rely "contain" the image content or is just a facade
  //   to an internally aggregated normal image. ... Should be checked.
  // - An other idea would also to combine it internally with a CompressedImageContainer to reduce the
  // - foot print of the label representation.
  class MITKMULTILABEL_EXPORT LabelProxyImage : public Image
  {
  public:
    using LabelValueType = mitk::Label::PixelType;

    mitkClassMacro(LabelProxyImage, SlicedData);
    mitkNewMacro2Param(Self, const MultiLabelImage*, LabelValueType);

    /** Returns the label information associated with the proxy image.*
     * @pre Label is (still) valid in the parent -> InvalidLabelException
     * @pre Parent is valid -> InvalidMultiLabelImageException*/
    const mitk::Label *GetLabel(LabelValueType value) const;
    /** Returns the label information associated with the proxy image.*
     * @pre Label is (still) valid in the parent -> InvalidLabelException
     * @pre Parent is valid -> InvalidMultiLabelImageException*/
    mitk::Label* GetLabel(LabelValueType value);

    /** Returns the background label information of the parent.*
     * @pre Parent is valid -> InvalidMultiLabelImageException*/
    const mitk::Label *GetBackgroundLabel() const;

    /**Gets the modified time for the whole instance. Equals at least the newest MTime of all its components.*/
    unsigned long GetMTime() const override;

  protected:

    LabelProxyImage(const MultiLabelImage* parent, LabelValueType label);
    LabelProxyImage(const LabelProxyImage &other) = delete;
    ~LabelProxyImage() override;

    friend class MultiLabelImage;

    //moved to protexted scope because it should not be used directly.
    using mitk::Image::Initialize;

    mitk::WeakPointer<MultiLabelImage> m_Parent;
    LabelValueType m_LabelValue;
  };

  /**
  * @brief Equal A function comparing two label set images for beeing equal in meta- and imagedata
  *
  * @ingroup MITKTestingAPI
  *
  * Following aspects are tested for equality:
  *  - MultiLabelImage members
  *  - working image data
  *  - layer image data
  *  - labels in label set
  *
  * @param rightHandSide An image to be compared
  * @param leftHandSide An image to be compared
  * @param eps Tolerance for comparison. You can use mitk::eps in most cases.
  * @param verbose Flag indicating if the user wants detailed console output or not.
  * @return true, if all subsequent comparisons are true, false otherwise
  */
  MITKMULTILABEL_EXPORT bool Equal(const mitk::MultiLabelImage &leftHandSide,
                                   const mitk::MultiLabelImage &rightHandSide,
                                   ScalarType eps,
                                   bool verbose);

} // namespace mitk

#endif // __mitkLabelProxyImage_H_
