/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef __mitkMultiLabelImage_H_
#define __mitkMultiLabelImage_H_

#include <mitkLabelProxyImage.h>
#include <mitkLabel.h>
#include <MitkMultilabelExports.h>

namespace mitk
{

  /** @brief MultiLabelImage class for handling labels and layers that are semantically linked
   (generated on the same input image(s)).
   @remark Refactored version. Breaking changes are:
   - a label is unique for all layers of a MultiLabelImage. So a label can only be associated
     with one layer.
   - reimplementation should avoid handcrafted iteration (as it is currently done), itk filter
     should be used to make use of threading.
  */
  class MITKMULTILABEL_EXPORT MultiLabelImage : public SlicedData
  {
  public:
    mitkClassMacro(MultiLabelImage, SlicedData);
    itkNewMacro(Self);

    using LabelValueType = mitk::Label::PixelType;
    using LayerIndexType = std::size_t;
    using ConstLabelVectorType = std::vector<Label::ConstPointer>;
    using LabelVectorType = std::vector<Label::Pointer>;

    /**
     * @brief Initialize an mitk::MultiLabelImage using the information of an mitk::Image
     * (e.g. dimensionality, time steps, geonetry...).
     * The content of the image can also be used to init the content of the label set image.
     * This is controlled by the parameter useImageContent (see details below).
     * @param image the image which is used for initializing the mitk::MultiLabelImage
     * @param useImageContent If useImageContent equals false, the image content will be ignored.
     * One empty layer will be generated per time step. If useImageContent equals true, the image
     * content of each time step will be used. For all distinct pixel values of the parameter image
     * new labels will be created. If the number of distinct pixel values exceeds
     * mitk::Label::MAX_LABEL_VALUE an exception will be thrown. If the pixel value of the image
     * can be directly mapped into the the values of LabelValueType, the value will also be
     * used for the label.
     * @pre image must point to a valid instance.
     * @pre If image content is used, image does not have more distinct pixel values, then the range of
     * Label::PixelType.
     */
    void Initialize(const mitk::Image *image, bool useImageContent = false);

    /**
     * @brief Initialize an mitk::MultiLabelImage with one empty layer, using the information
     * of an mitk::ImageDescriptor.
     * @param inDesc Pointer to an ImageDescriptor  instance which is used for initializing the mitk::MultiLabelImage
     * @pre inDesc must point to a valid instance.
     */
    void Initialize(const mitk::ImageDescriptor* inDesc);

    /**
     * @brief Initialize an mitk::MultiLabelImage with one empty layer, using the information
     * of an mitk::Geomtry.
     * @param geometry Pointer to an geometry instance which is used for initializing the mitk::MultiLabelImage
     * @pre geometry must point to a valid instance.
     */
    void Initialize(const mitk::BaseGeometry* geometry);

    /**
     * @brief Initialize an mitk::MultiLabelImage with one empty layer, using the information
     * of an mitk::TimeGeomtry.
     * @param geometry Pointer to an time geometry instance which is used for initializing the mitk::MultiLabelImage
     * @pre geometry must point to a valid instance.
     */
    void Initialize(const mitk::TimeGeometry* geometry);

    bool IsInitialized() const override;

    enum class MergeStyle
    {
      Replace, //The old label content of a lable value will be replaced by its new label content.
               //Therefore pixels that are labeled might become unlabeled again.
               //(This means that a lock of the value is also ignored).
      Merge //The union of old and new label content will be generated.
    };

    enum class OverwriteStyle
    {
      RegardLocks, //Locked labels on the same layer will not be overwritten.
      IgnoreLocks //Label locks on the same layer will be ignored.
    };

    /**
     * @brief Merges the content of a source label into the target label.
     *
     * @param targetLabelValue The value of the label that should be the merged label
     * If the label does not exist, it will be generated.
     * @param sourceLabelValue The value of the label that should be merged into the specified one
     * @param mergeStyle Controls the way the content of source label should be merged into
     * the target label and its layer.
     * @param overwriteStyle Controls if the merge operation should regard the label locks in the
     * layer of the target label.
     * @param removeSourceLabel If set to true the source label will be removed after the merge.
     * It results in the same state as calling RemoveLabels with the sourceLabelValue after the merge.
     * @param silentOp If set to true, no events will be triggered by the operation.
     */
    void MergeLabel(LabelValueType targetLabelValue, LabelValueType sourceLabelValue, MergeStyle mergeStyle = MergeStyle::Merge,
                    OverwriteStyle overwriteStyle = OverwriteStyle::RegardLocks, bool removeSourceLabel = false, bool silentOp = false);

    /**
     * @brief Merges the content of a source labels into the target label.
     *
     * @param targetLabelValue The value of the label that should be the merged label
     * If the label does not exist, it will be generated.
     * @param sourceLabelValues The values of the labels that should be merged into the target label
     * with merge style "Merge".
     * @param overwriteStyle Controls if/how the merge operation should regard the label locks in the
     * layer of the target label.
     * @param removeSourceLabel If set to true the source labels will be removed after the merge.
     * It results in the same state as calling RemoveLabels with the sourceLabelValues after the merge.
     * @param silentOp If set to true, no events will be triggered by the operation.
     */
    void MergeLabels(LabelValueType targetLabelValue, const std::vector <LabelValueType>& sourceLabelValues,
      OverwriteStyle overwriteStyle = OverwriteStyle::RegardLocks, bool removeSourceLabel = false, bool silentOp = false);

    /**
     * @brief Moves a label from one layer to another layer.
     *
     * @param labelValue The value of the label that should be moved.
     * If the label does not exist, nothing happens.
     * @param targetLayer The layer the label should be moved to. If the layer does not exist,
     * an exception will be raised.
     * @param overwriteStyle Controls if the move operation should regard the label locks in the
     * target layer.
     * @param silentOp If set to true, no events will be triggered by the operation.
     * @pre target layer must exist.
     */
    void MoveLabelToLayer(LabelValueType labelValue, LayerIndexType targetLayer,
      OverwriteStyle overwriteStyle = OverwriteStyle::RegardLocks, bool silentOp = false);

    /**
     * @brief Changes the value of the label.
     *
     * @param oldValue The value of the label that should be changed.
     * If the label does not exist, nothing happens.
     * @param newValue The new value of the label. If the value is already used an exception will
     * be thrown.
     * @param silentOp If set to true, no events will be triggered by the operation.
     * @pre newValue must not exist.
     */
    void ChangeLabelValue(LabelValueType oldValue, ChangeLabelValue newValue, bool silentOp = false);

    /**
     * @brief Updates all target labels by the content of the source image.
     *
     * If multiple target labels are specified the update operations will be done
     * the same order.
     * @param targetLabelValues Vector of all values that should be updated. If the value does not
     * exist in the sourceImage it will be ignored. If it is not a valid label in the MultiLabelImage
     * instance an exception will be raised.
     * @param sourceImage Pointer to the image that contains the update content that should be transfered.
     * @param mergeStyle Controls the way the content of source label should be merged into
     * the target label and its layer.
     * @param overwriteStyle Controls if the merge operation should regard the label locks in the
     * layer of the target label.
     * @param aPlane If specified the update will only performed on the given plane and not for the whole
     * volume.
     * @param ignoreTimeGeometry If set to false (default), the time geometry will be used to identify,
     * the timesteps of the source image that should be used to update the respective time step of the
     * MultiLabelImage. If set to true, the time geometry will be ignored. Static source image (Time step count ==1)
     * will be used to update all time steps of the MultiLabelImage. For dynamic source images it is assumed
     * that the time steps of both images correspond.
     * @param silentOp If set to true, no events will be triggered by the operation.
     * @pre targetLabelValues may not contain labels that are not defined.
     * @pre sourceImage must point to a valid instance.
     * @pre If ignoreTimeGeometry is true and sourceImage->GetTimeSteps()>1, then sourceImage->GetTimeSteps() must
     * equal this->GetTimeSteps().
     */
    void UpdateLabels(const std::vector <LabelValueType>& targetLabelValues, const Image* sourceImage, MergeStyle mergeStyle = MergeStyle::Replace,
      OverwriteStyle overwriteStyle = OverwriteStyle::RegardLocks, const mitk::PlaneGeometry* aPlane = nullptr,
      bool ignoreTimeGeometry = false, bool silentOp = false);

    /**
     * @brief Removes labels from the mitk::LabelSet of given layer.
     * Calls mitk::MultiLabelImage::EraseLabels() which also removes the labels from within the image.
     * If a label value does not exist, it will be ignored.
     * @param labelValues a list of labels to be removed
     */
    void RemoveLabels(const std::vector<LabelValueType> &labelValues);

    /**
     * @brief Removes a complete layer with all its labels.
     * @remark with removing a layer all layers with greater index will be reindexed to
     * close the gap. So externaly stored layer indeces may become invalid.
     * @param layer index of the layer that should be removed. If the layer does not exist, an
     * exception will be raised.
     * @pre layer index must be valid.
     * @pre Instance may not be empty after the operation; thus the last layer cannot be removed.
     */
    void RemoveLayer(LayerIndexType layer);

    /**
     * @brief Erases the label from its respective layer. After the operation, the label is still
     * defined, but no pixel is associated with this label anymore.
     * @param value the value of the label which will be remove from the image. If this label does not
     * exists, nothing happens and this instance will not be modified.
     */
    void EraseLabelContent(LabelValueType value);
    /** Overloaded version that removes the label content just in the specified time step.*
     * @param t Index of the time step for which the label content should be erased.
     * @pre t must be a valid TimeStep.*
     */
    void EraseLabelContent(LabelValueType value, TimeStepType t);

    /**
     * @brief Similar to mitk::MultiLabelImage::EraseLabelContent() this funtion erase a list of labels
     * from the image content.
     * @param labelValues the list of labels that should be remove. If a lable value does not exist,
     * it will be ignored.
     */
    void EraseLabelContents(const std::vector<LabelValueType>& labelValues);
    /** Overloaded version that removes the label contents just in the specified time step.*
     * @param t Index of the time step for which the label content should be erased.
     * @pre t must be a valid TimeStep.*
     */
    void EraseLabelContents(const std::vector<LabelValueType>& labelValues, TimeStepType t);

    /**
      * \brief  Returns true if the value exists in the MultiLabelImage instance*/
    bool ExistLabel(LabelValueType value) const;

    /**
     * @brief Checks if a label exists in a certain layer
     * @param value the label value
     * @param layer the layer in which should be searched for the label
     * @return true if the label exists otherwise false
     */
    bool ExistLabel(LabelValueType value, LayerIndexType layer) const;

    /**
      * \brief  Returns true if the layer exists in the MultiLabelImage instance.*/
    bool ExistLayer(LayerIndexType layer) const;

    /**
     * @brief Returns the mitk::Label with the given value.
     * @param value the pixel value of the label
     * @return the mitk::Label if available otherwise nullptr
     */
    const mitk::Label *GetLabel(LabelValueType value) const;
    mitk::Label* GetLabel(LabelValueType value);

    /** Returns a vector with all labels currently defined in the MultiLabelImage
    instance.*/
    const ConstLabelVectorType* GetLabels() const;
    const LabelVectorType* GetLabels();

    /**
     * @brief Retruns a vector of all labels located on the specified layer.
     * @param layer the layer for which the vector of labels should be retrieved.
     * @return the respective vector of labels or nullptr if the layer does not exist.
     */
    const ConstLabelVectorType* GetLabelsInLayer(LayerIndexType layer) const;
    const LabelVectorType* GetLabelsInLayer(LayerIndexType layer);

    /** returns the number of layers currently existing in the MultiLabelImage instance.*/
    LayerIndexType GetNumberOfLayers() const;

    /**
     * @brief Get the number of all existing mitk::Labels for a given layer
     * @param layer the layer ID for which the number of labels should be retrieved
     * @return the number of all existing mitk::Labels for the given layer
     */
    LabelValueType GetNumberOfLabels(LayerIndexType layer = 0) const;

    /**
     * @brief Returns the number of all labels summed up across all layers
     * @return the overall number of labels across all layers
     */
    LabelValueType GetTotalNumberOfLabels() const;

    /** Returns a proxy image that allows to access the content of one specified label as image.
     * This proxy image can be used like a normal image. The main differences to a normal image
     * are 1) the MultiLabelImage always keeps ownership and also handles updates in order to
     * keep the proxy up to date; 2) the proxy is lazy, thus the image data is only generated if
     * realy needed. For more details see LabelProxyImage.
     * If the label value does not exist, nullptr will be returned.*
     * @param value The label value that should be represented in the label image.*/
    const mitk::LabelProxyImage* GetLabelImage(LabelValueType value) const;

    /** Returns the image that represents the specified layer. If the layer does not exist,
     * a null ptr will be returned.*/
    const mitk::Image* GetLayerImage(LayerIndexType layer) const;

    /**Adds a new layer.
    *@param layerContent Pointer to an image instance that should be used as content for the new
    * layer (equal to the behavior of Initialize(const mitk::Image*)). The layerContent image will be copied. If layerContent is nullptr, an empty layer
    * will be generated.
    *@pre layerContent must have the same time geometry than the instance of MultiLabelImage.
    *@pre layerContent must not contain pixel values that are already defined on other layers.
    * */
    LayerIndexType AddLayer(const mitk::Image* layerContent = nullptr);

    /** Adds a new label to the MultiLabelImage instance.
     *@param value Value of the new label.
     *@param layer Layer the new label should be on.
     *@pre The new label does not already exist.-> InvalidLabelException*
     *@pre The layer does exist. -> InvalidMultiLabelImageException*/
    Label* AddLabel(LabelValueType value, LayerIndexType layer = 0);


    /**
     * @brief Removes the active layer and the respective mitk::LabelSet and image information.
     *        The new active layer is the one below, if exists
     */
    void RemoveLayer();

    /**
     * @brief Gets the mitk::Label which is used as exterior label
     * @return the exterior mitk::Label
     */
    mitk::Label *GetExteriorLabel();
    const mitk::Label *GetExteriorLabel() const;

    /**Returns the current lookup table that represents the label settings of the MultiLableImage instance.
    */
    itkGetConstObjectMacro(LookupTable, mitk::LookupTable);

    ////////////////////////////////////////////////////////////////////
    //Modification time queries.

    /**Gets the modified time for the whole instance. Equals at least the newest MTime of all its components.*/
    unsigned long GetMTime() const override;

    /**Returns the modified time of a layer. Results in the same like this->GetLayerImage(layer)->GetMTime().
     *@param layer Index of the layer.
     *@pre layer index must be valid. -> InvalidMultiLabelImageException
     */
    unsigned long GetLayerMTime(LayerIndexType layer = 0) const;

    /**Returns the modified time of the label voxel content.
     * The label content is modified if its assigend voxels are changed
     * or its layer.
     * Results in the same like this->GetLabelImage(label)->GetMTime().
     * In difference to GetLabelMTime value changes of the respective label
     * instance (e.g. name, visibility, opacity, color, label value) will be
     * ignored. Only the the pixel content is relevant.
     *@param label Value of the label.
     *@pre label index must be valid. -> InvalidLabelException
     */
    unsigned long GetLabelContentMTime(LabelValueType label) const;

    /**Returns the modified time of the label voxel content.
     * The label content is modified if its assigend voxels are changed
     * or its layer.
     * Results in the same like this->GetLabel(label)->GetMTime().
     * In difference to GetLabelContentMTime value changes of the respective label
     * instance (e.g. name, visibility, opacity, color, label value) are taken
     * into account. Thus any changes of a respective label are relevant.
     *@param label Value of the label.
     *@pre label index must be valid. -> InvalidLabelException
     */
    unsigned long GetLabelMTime(LabelValueType label) const;


    ////////////////////////////////////////////////////////////////////
    //Message slots that allow to react to changes in an instance

    using LabelEventType = Message1<LabelValueType>;
    using LayerEventType = Message1<LayerIndexType>;

    /**
    * \brief LabelAdded is emitted whenever a new label has been added.
    *
    * Observers should register to this event by calling this->AddLabelAddedListener(myObject,
    * MyObject::MyMethod).
    * After registering, myObject->MyMethod() will be called every time a new label has been added to the LabelSet.
    * Observers should unregister by calling this->RemoveLabelAddedListener(myObject, MyObject::MyMethod).
    * @remark the usage of the message object is thread safe.
    */
    mitkNewMessage1Macro(LabelAdded, LabelValueType);

    /**
    * \brief LabelModified is emitted whenever a label has been modified.
    * 
    * A label is modified if either its pixel content was changed, its layer or the label instance
    * information.
    * If you just want to get notfied at the end of an MultiLabelImage instance manipulation in the
    * case that at least one label was modified (e.g. to avoid getting an signal for each label
    * individually), use LabelsChanged instead.
    * Observers should register to this event by calling this->AddLabelModifiedListener(myObject,
    * MyObject::MyMethod).
    * After registering, myObject->MyMethod() will be called every time a new label has been added to the LabelSet.
    * Observers should unregister by calling this->RemoveLabelModifiedListener(myObject, MyObject::MyMethod).
    * @remark the usage of the message object is thread safe.
    */
    mitkNewMessage1Macro(LabelModified, LabelValueType);

    /**
    * \brief LabelRemoved is emitted whenever a label has been removed.
    *
    * Observers should register to this event by calling this->AddLabelRemovedListener(myObject,
    * MyObject::MyMethod).
    * After registering, myObject->MyMethod() will be called every time a new label has been added to the LabelSet.
    * Observers should unregister by calling this->RemoveLabelRemovedListener(myObject, MyObject::MyMethod).
    * @remark the usage of the message object is thread safe.
    */
    mitkNewMessage1Macro(LabelRemoved, LabelValueType);

    /**
    * \brief LabelsChanged is emitted when labels are changed (added, removed, modified).
    *
    * In difference to the other label events LabelsChanged is send only *one time* after the modification of the
    * MultiLableImage instance is finished. So e.g. even if 4 labels are changed by a merge operation, this event will
    * only be sent one time (compared to LabelRemoved or LabelModified). 
    * Observers should register to this event by calling myLabelSet->AddLabelsChangedListener(myObject,
    * MyObject::MyMethod).
    * After registering, myObject->MyMethod() will be called every time a new label has been removed from the LabelSet.
    * Observers should unregister by calling myLabelSet->RemoveLabelsChangedListener(myObject,
    * MyObject::MyMethod).
    *
    * member variable is not needed to be locked in multi-threaded scenarios since the LabelSetEvent is a typedef for
    * a Message object which is thread safe
    */
    mitkNewMessage1Macro(LabelsChanged, LabelValueType);

  protected:
    mitkCloneMacro(Self);

    MultiLabelImage();
    MultiLabelImage(const MultiLabelImage &other);
    ~MultiLabelImage() override;

    /**
    * Removes the complete content (all labels and additional layers).
    * After clearance only one empty layer is left.
    */
    virtual void ClearData() override;

    /** Return non const pointer to a layer image.*/
    mitk::Image* GetLayerImage(LayerIndexType layer);

    /** \brief
    */
    void SetLookupTable(LookupTable* lut);

    /** \brief
    */
    void UpdateLookupTable(PixelType pixelValue);

    /** Methods updates the geometric properties (e.g. center of mass)
     * of all labels in a layer.*/
    //Implementation remark: Use itk::LabelGeometryImageFilter instead of handcrafted calculation!
    void UpdateLabelGeometrieProperties(LayerIndexType layer);

    Label::Pointer m_ExteriorLabel;

    using LabelMapType = std::map<LabelValueType, Label::Pointer>;
    LabelMapType m_LabelMap;

    using LabelImageMapType = std::map<LabelValueType, LabelProxyImage::Pointer>;
    LabelImageMapType m_LabelImageMap;

    using LayerImageVectorType = std::vector<Image::Pointer>;
    LayerImageVectorType m_LayerImages;

    /**This type is internally used to track which label is currently
    * associated which with layer.*/
    using LayerToLabelMapType = std::map<LayerIndexType, LabelVectorType>;
    LayerToLabelMapType m_LayerToLabelMap;
    using LabelToLayerMapType = std::map<LabelValueType, LayerIndexType>;
    LabelToLayerMapType m_LabelToLayerMap;

    LookupTable::Pointer m_LookupTable;
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

#endif // __mitkMultiLabelImage_H_
