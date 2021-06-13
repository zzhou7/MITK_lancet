/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef __mitkMultiLabelSegmentation_H_
#define __mitkMultiLabelSegmentation_H_

#include <mitkLabelProxyImage.h>
#include <mitkLabel.h>
#include <MitkMultilabelExports.h>

namespace mitk
{

  /** @brief MultiLabelSegmentation class represents multiple labels/segmentions that are semantically linked.
   *
   * The sementical linkage of labels are expressed in several ways by this class:
   * - First, it is assumed that all labels represented by this class were generated on the same input image(s)
   * and therefor represent the same anatomical/topological representation. Within this representation the labels
   * annotated/represent different elements (e.g. different organs or different tissue classes (e.g. healthy,
   * pathologic, ...)).
   * - Second, labels can be grouped in spatial groups. All labels in a spatial group are spatially mutual exclusiv.
   * Thus two labels in a spatial group cannot overlap. If the content of a label is updated the MultiLabelSegmentation
   * instance will ensure that the spatial group semantics are regarded (for more details see MergeStyle and OverwriteStyle
   * below). The criterion for label overlap depends on the SegmentationStyle and the SegmenationFractionalType (see below)
   *
   * Overlap criterion:
   * - SegmentationStyle = BINARY: Labels of a spatial group overlap, if the mark the same spatial/voxel as foreground.
   * - SegmentationStyle = FRACTIONAL and SegmentationFractionType = OCCUPANCY: Labels of a spatial group overlap, if their
   * fractional sum for the same spatial/voxel is greater then 1.0 (so more then 100% of voxel volume would be covered by them).
   * - SegmentationStyle = FRACTIONAL and SegmentationFractionType = PROPABILITY: Labels of a spatial group overlap, if their
   * fractional sum for the same spatial/voxel is greater then 1.0 (so all labels in the group have in total a properbility higher
   * then 100%, which cannot be).
   * @remark The SegmentationStyle and SegmentationFractionType can only be defined/changed at creation time. To convert a
   * MultiLabelSegmentation instance to another style or type a conversion function/policy has to be used that generates a
   * new MultiLabelSegmentation instance with the converted content.
   * @remark This calls abstract away the internal representation of the labels. The first implementation pragmatically focuses
   * on pixel based representation (and the class will always provide a possibility to query LabelImages); but it is also not
   * set how the memory layout looks like (e.g. real images, or concepts like itk::LabelMap). But in later iterations of this
   * class labels might also be representation by non rasterized structures like meshes or alike.
   * @remark The design of the class aims for a.o. the compatibality with DICOM
   * Segmentation to ensure easy serialization from and into it. Divergence can be found where
   * it is needed for an easier/more functional representation in the application.
   * @remark The current implementation does only support SegmentationStyle BINARY. This should be extended in the future.
   * Nevertheless the design was already made with the other options in mind to ensure future additions without breaking
   * changes.
   @remark Refactored version. Breaking changes are:
   - Layers are now more abstract defined as spatial groups.
   - a label is unique for all spatial groups (formerly known as layer) of a MultiLabelSegmentation. So a label can
     only be associated with one spatial group.

   //Implementation remark:
   - reimplementation should avoid handcrafted iteration (as it is currently done), itk filter
     should be used to make use of threading.
   - The first version supports SegmentationStyle BINARY. Other options are postponed and will be implemented later on.

  */
  class MITKMULTILABEL_EXPORT MultiLabelSegmentation : public BaseData
  {
  public:
    mitkClassMacro(MultiLabelSegmentation, BaseData);
    itkNewMacro(Self);

    using LabelValueType = mitk::Label::PixelType;
    using SpatialGroupIndexType = std::size_t;
    using ConstLabelVectorType = std::vector<Label::ConstPointer>;
    using LabelVectorType = std::vector<Label::Pointer>;

    /**
     * @brief Initialize an mitk::MultiLabelSegmentation using the information of an mitk::Image
     * (e.g. dimensionality, time steps, geonetry...).
     * The content of the image can also be used to init the content of the label set image.
     * This is controlled by the parameter useImageContent (see details below).
     * @param templateImage the image which is used for initializing the MultiLabelSegmentation
     * @param useImageContent If useImageContent equals false, the image content will be ignored.
     * If useImageContent equals true, the image content of each time step will be used. Independent
     * from the SegmentationStyle of the MultiLabelSegmentation instance is BINARY, this function
     * creates for all distinct pixel values of templateImage new labels. If the SegmentationStyle is
     * FRACTIONAL it will always use the fractional value 1.0. If the number of distinct pixel values
     * exceeds mitk::Label::MAX_LABEL_VALUE an exception will be thrown.
     * If the pixel value of the image can be directly mapped into the the values of LabelValueType,
     * the value will also be used for the label.
     * @param sameSpatialGroup. If useImageContent == true, this parameter controlls if the created labels
     * should be in one spatial group or if all have a own.
     * @pre image must point to a valid instance.
     * @pre If image content is used, image does not have more distinct pixel values, then the range of
     * Label::PixelType.
     */
    void Initialize(const mitk::Image *templateImage, bool useImageContent = false, bool sameSpatialGroup = true);

    /**
     * @brief Initialize an mitk::MultiLabelSegmentation using the information
     * of an mitk::ImageDescriptor.
     * @param inDesc Pointer to an ImageDescriptor  instance which is used for initializing the mitk::MultiLabelSegmentation
     * @pre inDesc must point to a valid instance.
     */
    void Initialize(const mitk::ImageDescriptor* inDesc);

    /**
     * @brief Initialize an mitk::MultiLabelSegmentation using the information
     * of an mitk::Geomtry.
     * @param geometry Pointer to an geometry instance which is used for initializing the mitk::MultiLabelSegmentation
     * @pre geometry must point to a valid instance.
     */
    void Initialize(const mitk::BaseGeometry* geometry);

    /**
     * @brief Initialize an mitk::MultiLabelSegmentation using the information
     * of an mitk::TimeGeomtry.
     * @param geometry Pointer to an time geometry instance which is used for initializing the mitk::MultiLabelSegmentation
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
      RegardLocks, //Locked labels in the same spatial group will not be overwritten/changed.
      IgnoreLocks //Label locks in the same spatial group will be ignored, so these labels might be changed.
    };

    /** Enum that specifies the encoding of the label content for certain voxel location.
     * This encodes the DICOM attribute "Segmentation Type" (0062,0001)*/
    enum class SegmentationStyle
    {
      BINARY, //Background is encoded by 0, the label content is indicated by 1.
      FRACTIONAL //Encodes the association with a label by a value between 0 and
                 //1. How the value is interpreted is defined by SegmentationFractionalType.
    };

    /** If SegmentationStyle is "FRACTIONAL", this enum encodes how the fractional
     * value should be interpreted.
     * MITK, in difference to the DICOM standard, always encodes the fraction as float
     * value between 0 .. 1.0. The when loading and storing it into DICOM Segmentation
     * It has to be converted accordingly.
     * This encodes the DICOM attribute "Segmentation Type" (0062,0010)*/
    enum class SegmentationFractionalType
    {
      PROBABILITY, //Defines the probability, as value between 0 and 1.0, that the segmented
                   //property occupies the spatial area defined by the voxel.
      OCCUPANCY //Defines the proportion of the pixel volume occupied by the segmented
                //property as a value between 0 and 1.0.
    };

    enum class OverlapState
    {
      NO = 0, //no lables overlap
      UNDEFINED = 1 //Some labels might overlap
      YES = 2 //Some labeles overlap
    };

    /** Returns the SegmentationStyle of the instance.
     * The style is defined when
     * created the instance. Changing the style always means to convert the content
     * into a new instance.
     * This encodes the DICOM attribute "Segmentation Type" (0062,0001).
     * The values can also be recivied as a property GetProperty() via the property
     * name "DICOM.0062.0001"*/
    virtual SegmentationStyle GetSegmentationStyle() const;

    /** Returns the SegmentationFractionalType of the instance.
     * Is only relevant if SegmentationStyle is "FRACTIONAL".
     * The style is defined when created the instance.
     * This encodes the DICOM attribute "Segmentation Type" (0062,0010).
     * The values can also be recivied as a property GetProperty() via the property
     * name "DICOM.0062.0010"*/
    virtual SegmentationFractionalType GetSegmentationFractionalType() const;

    /** Returns the current OverlapState of the instance.
     * This encodes the DICOM attribute "Segments Overlap" (0062,0013).
     * The values can also be recivied as a property GetProperty() via the property
     * name "DICOM.0062.0013"*/
    virtual OverlapState GetOverlapState() const;

    /**
     * @brief Merges the content of a source label into the target label.
     *
     * @param targetLabelValue The value of the label that should be the merged label
     * If the label does not exist, it will be generated.
     * @param sourceLabelValue The value of the label that should be merged into the specified one
     * @param mergeStyle Controls the way the content of source label should be merged into
     * the target label and its spatial group.
     * @param overwriteStyle Controls if the merge operation should regard the label locks in the
     * spatial group of the target label.
     * @param removeSourceLabel If set to true the source label will be removed after the merge.
     * It results in the same state as calling RemoveLabels with the sourceLabelValue after the merge.
     * @param silentOp If set to true, no events will be triggered by the operation.
     * @remark If a spatial group is empty after the operation, it might be removed. The removal of an spatial group
     * might invalidate any other spatial group index (due to new sorting/ordering).
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
     * spatial group of the target label.
     * @param removeSourceLabel If set to true the source labels will be removed after the merge.
     * It results in the same state as calling RemoveLabels with the sourceLabelValues after the merge.
     * @param silentOp If set to true, no events will be triggered by the operation.
     * @remark If a spatial group is empty after the operation, it might be removed. The removal of an spatial group
     * might invalidate any other spatial group index (due to new sorting/ordering).
     */
    void MergeLabels(LabelValueType targetLabelValue, const std::vector <LabelValueType>& sourceLabelValues,
      OverwriteStyle overwriteStyle = OverwriteStyle::RegardLocks, bool removeSourceLabel = false, bool silentOp = false);

    /**
     * @brief Moves a label from one spatial group to another spatial group.
     *
     * @param labelValue The value of the label that should be moved.
     * If the label does not exist, nothing happens.
     * @param targetGroup The spatial group the label should be moved to. If the spatial group does not exist,
     * an exception will be raised.
     * @param overwriteStyle Controls if the move operation should regard the label locks in the
     * target spatial group.
     * @param silentOp If set to true, no events will be triggered by the operation.
     * @pre target spatial group must exist.
     * @post the label is removed from its original spatial group and is moved to the target group.
     * @remark If a spatial group is empty after the operation, it might be removed. The removal of an spatial group
     * might invalidate any other spatial group index (due to new sorting/ordering).
     */
    void MoveLabelToLayer(LabelValueType labelValue, SpatialGroupIndexType targetGroup,
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
     * exist in the sourceImage it will be ignored. If it is not a valid label in the MultiLabelSegmentation
     * instance an exception will be raised.
     * @param sourceImage Pointer to the image that contains the update content that should be transfered.
     * @param mergeStyle Controls the way the content of source label should be merged into
     * the target label and its spatial group.
     * @param overwriteStyle Controls if the merge operation should regard the label locks in the
     * spatial group of the target label.
     * @param aPlane If specified the update will only performed on the given plane and not for the whole
     * volume.
     * @param ignoreTimeGeometry If set to false (default), the time geometry will be used to identify,
     * the timesteps of the source image that should be used to update the respective time step of the
     * MultiLabelSegmentation. If set to true, the time geometry will be ignored. Static source image (Time step count ==1)
     * will be used to update all time steps of the MultiLabelSegmentation. For dynamic source images it is assumed
     * that the time steps of both images correspond.
     * @param silentOp If set to true, no events will be triggered by the operation.
     * @pre targetLabelValues may not contain labels that are not defined.
     * @pre sourceImage must point to a valid instance.
     * @pre If ignoreTimeGeometry is true and sourceImage->GetTimeSteps()>1, then sourceImage->GetTimeSteps() must
     * equal this->GetTimeSteps().
     */
    void UpdateLabelsByImage(const std::vector <LabelValueType>& targetLabelValues, const Image* sourceImage, MergeStyle mergeStyle = MergeStyle::Replace,
      OverwriteStyle overwriteStyle = OverwriteStyle::RegardLocks, const mitk::PlaneGeometry* aPlane = nullptr,
      bool ignoreTimeGeometry = false, bool silentOp = false);

    /**
     * @brief Removes labels from the mitk::MultiLabelSegmentation.
     * Calls mitk::MultiLabelSegmentation::EraseLabels() which also removes the labels from within the image.
     * If a label value does not exist, it will be ignored.
     * @param labelValues a list of labels to be removed
     */
    void RemoveLabels(const std::vector<LabelValueType> &labelValues);

    /**
     * @brief Removes a complete spatial group with all its labels.
     * @remark with removing a spatial group all spatial groups with greater index will be reindexed to
     * close the gap. So externaly stored spatial group indeces may become invalid.
     * @param spatial group index of the spatial group that should be removed. If the spatial group does not exist, an
     * exception will be raised.
     * @pre spatial group index must be valid.
     */
    void RemoveSpatialGroup(SpatialGroupIndexType spatielGroup);

    /**
     * @brief Erases the label content. After the operation, the label is still
     * defined (including its spatial group assiciation), but no pixel /spatial area is associated with
     * this label anymore.
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
     * @brief Similar to mitk::MultiLabelSegmentation::EraseLabelContent() this funtion erase a list of labels
     * contents.
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
      * \brief  Returns true if the value exists in the MultiLabelSegmentation instance*/
    bool ExistLabel(LabelValueType value) const;

    /**
     * @brief Checks if a label belongs in a certain spatial group
     * @param value the label value
     * @param groupIndex Indexp of the spacial group which should be checked for the label
     * @return true if the label exists otherwise false
     */
    bool ExistLabel(LabelValueType value, SpatialGroupIndexType groupIndex) const;

    /**
      * \brief  Returns true if the spatial group exists in the MultiLabelSegmentation instance.*/
    bool ExistSpatialGroup(SpatialGroupIndexType index) const;

    /**
     * @brief Returns the mitk::Label with the given value.
     * @param value the pixel value of the label
     * @return the mitk::Label if available otherwise nullptr
     */
    const mitk::Label *GetLabel(LabelValueType value) const;
    mitk::Label* GetLabel(LabelValueType value);

    /** Returns a vector with all labels currently defined in the MultiLabelSegmentation
    instance.*/
    const ConstLabelVectorType* GetLabels() const;
    const LabelVectorType* GetLabels();

    /**
     * @brief Retruns a vector of all labels located on the specified spatial group.
     * @param index the index of the spatial group for which the vector of labels should be retrieved.
     * If an invalid index is passed an exception will be raised.
     * @return the respective vector of labels.
     * @pre spatial group index must exist.
     */
    const ConstLabelVectorType GetLabelsInLayer(SpatialGroupIndexType index) const;
    const LabelVectorType GetLabelsInLayer(SpatialGroupIndexType index);

    /** returns the number of spatial groups currently existing in the MultiLabelSegmentation instance.*/
    SpatialGroupIndexType GetNumberOfLayers() const;

    /**
     * @brief Get the number of all existing mitk::Labels for a given spatial group
     * @param index of the spatial group for which the number of labels should be retrieved
     * @return the number of all existing mitk::Labels in the given spatial group
     */
    LabelValueType GetNumberOfLabelsInSpatialGroup(SpatialGroupIndexType index = 0) const;

    /**
     * @brief Returns the number of all labels.
     * @return the overall number of labels
     */
    LabelValueType GetNumberOfLabels() const;

    /** Returns a proxy image that allows to access the content of one specified label as image.
     * This proxy image can be used like a normal image. The main differences to a normal image
     * are 1) the MultiLabelSegmentation always keeps ownership and also handles updates in order to
     * keep the proxy up to date; 2) the proxy is lazy, thus the image data is only generated if
     * realy needed. For more details see LabelProxyImage.
     * If the label value does not exist, nullptr will be returned.*
     * @param value The label value that should be represented in the label image.*/
    const LabelProxyImage* GetLabelImage(LabelValueType value) const;

    /**Adds a new spatial group including labels and content defined by an image.
     *@param groupContent Pointer to an image instance that should be used as content for the new
     * spatial group (equal to the behavior of Initialize(const mitk::Image*)).
     * The groupContent image will not be referenced internally, but copied if needed. Thus later
     * changes in groupContent will not affect the MultiLabelSegmentation instance.
     *@param silentOp If set to true, no events will be triggered by the operation.
     *@pre groupContent must have the same time geometry than the instance of MultiLabelSegmentation.
     *@pre groupContent must not contain pixel values that are already defined. -> InvalidLabelException*/
    SpatialGroupIndexType AddSpatialGroupByImage(const mitk::Image* groupContent, bool silentOp = false);

    /**Adds a new empty spatial group to the MultiLabelSegmentation and returns the index of the new
     * spatial group.*/
    SpatialGroupIndexType AddSpatialGroup();

    /** Adds a new label to the MultiLabelSegmentation instance.
     *@param value Value of the new label.
     *@param groupIndex index of the spatial group the new label should be in.
     *@param silentOp If set to true, no events will be triggered by the operation.
     *@pre The new label does not already exist.-> InvalidLabelException*
     *@pre The spatial group does exist. -> InvalidMultiLabelSegmentationException*/
    Label* AddLabel(LabelValueType value, SpatialGroupIndexType groupIndex = 0, bool silentOp = false);

    /**
     * @brief Gets the mitk::Label which is used as background label
     * @return the exterior mitk::Label
     */
    mitk::Label *GetBackgroundLabel();
    const mitk::Label *GetBackgroundLabel() const;

    /**Returns the current lookup table that represents the label settings of the MultiLableImage instance.
    */
    itkGetConstObjectMacro(LookupTable, mitk::LookupTable);

    ////////////////////////////////////////////////////////////////////
    //Modification time queries.

    /**Gets the modified time for the whole instance. Equals at least the newest MTime of all its components.*/
    unsigned long GetMTime() const override;

    /**Returns the modified time of a spatial group.
     *@param index Index of the spatial group.
     *@pre spatial group index must be valid. -> InvalidMultiLabelSegmentationException
     */
    unsigned long GetSpatialGroupMTime(SpatialGroupIndexType index = 0) const;

    /**Returns the modified time of the label voxel content.
     * The label content is modified if its assigend voxels are changed
     * or its spatial group.
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
     * or its spatial group.
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

    /**
    * \brief LabelAdded is emitted whenever a new label has been added.
    *
    * Observers should register to this event by calling this->AddLabelAddedListener(myObject,
    * MyObject::MyMethod).
    * After registering, myObject->MyMethod() will be called every time a new label has been added to the MultiLabelSegmentation.
    * Observers should unregister by calling this->RemoveLabelAddedListener(myObject, MyObject::MyMethod).
    * @remark the usage of the message object is thread safe.
    */
    mitkNewMessage1Macro(LabelAdded, LabelValueType);

    /**
    * \brief LabelModified is emitted whenever a label has been modified.
    * 
    * A label is modified if either its pixel content was changed, its spatial group or the label instance
    * information.
    * If you just want to get notfied at the end of an MultiLabelSegmentation instance manipulation in the
    * case that at least one label was modified (e.g. to avoid getting an signal for each label
    * individually), use LabelsChanged instead.
    * Observers should register to this event by calling this->AddLabelModifiedListener(myObject,
    * MyObject::MyMethod).
    * After registering, myObject->MyMethod() will be called every time a new label has been added to the MultiLabelSegmentation.
    * Observers should unregister by calling this->RemoveLabelModifiedListener(myObject, MyObject::MyMethod).
    * @remark the usage of the message object is thread safe.
    */
    mitkNewMessage1Macro(LabelModified, LabelValueType);

    /**
    * \brief LabelRemoved is emitted whenever a label has been removed.
    *
    * Observers should register to this event by calling this->AddLabelRemovedListener(myObject,
    * MyObject::MyMethod).
    * After registering, myObject->MyMethod() will be called every time a new label has been added to the MultiLabelSegmentation.
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
    * Observers should register to this event by calling myMultiLabelSegmentation->AddLabelsChangedListener(myObject,
    * MyObject::MyMethod).
    * After registering, myObject->MyMethod() will be called every time a new label has been removed from the MultiLabelSegmentation.
    * Observers should unregister by calling myMultiLabelSegmentation->RemoveLabelsChangedListener(myObject,
    * MyObject::MyMethod).
    *
    * @remark the usage of the message object is thread safe.
    */
    mitkNewMessage1Macro(LabelsChanged, LabelValueType);

  protected:
    mitkCloneMacro(Self);

    MultiLabelSegmentation();
    MultiLabelSegmentation(const MultiLabelSegmentation &other);
    ~MultiLabelSegmentation() override;

    /**
    * Removes the complete content (all labels and additional spatial groups).
    * After clearance only one empty layer is left.
    */
    virtual void ClearData() override;

    /** Return non const pointer to a layer image.*/
    mitk::Image* GetLayerImage(SpatialGroupIndexType layer);
    /** Returns the image that represents the specified layer. If the layer does not exist,
     * a null ptr will be returned.*/
    const mitk::Image* GetLayerImage(SpatialGroupIndexType layer) const;

    /** \brief
    */
    void SetLookupTable(LookupTable* lut);

    /** \brief
    */
    void UpdateLookupTable(PixelType pixelValue);

    /** Methods updates the geometric properties (e.g. center of mass)
     * of all labels in a layer.*/
    //Implementation remark: Use itk::LabelGeometryImageFilter instead of handcrafted calculation!
    void UpdateLabelGeometrieProperties(SpatialGroupIndexType layer);

    Label::Pointer m_BackgroundLabel;

    using LabelMapType = std::map<LabelValueType, Label::Pointer>;
    LabelMapType m_LabelMap;

    using LabelImageMapType = std::map<LabelValueType, LabelProxyImage::Pointer>;
    LabelImageMapType m_LabelImageMap;

    using LayerImageVectorType = std::vector<Image::Pointer>;
    LayerImageVectorType m_LayerImages;

    /**This type is internally used to track which label is currently
    * associated which with layer.*/
    using LayerToLabelMapType = std::map<SpatialGroupIndexType, LabelVectorType>;
    LayerToLabelMapType m_LayerToLabelMap;
    using LabelToLayerMapType = std::map<LabelValueType, SpatialGroupIndexType>;
    LabelToLayerMapType m_LabelToLayerMap;

    LookupTable::Pointer m_LookupTable;
  };

  /**
  * @brief Equal A function comparing two label set images for beeing equal in meta- and imagedata
  *
  * @ingroup MITKTestingAPI
  *
  * Following aspects are tested for equality:
  *  - MultiLabelSegmentation members
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
  MITKMULTILABEL_EXPORT bool Equal(const mitk::MultiLabelSegmentation &leftHandSide,
                                   const mitk::MultiLabelSegmentation &rightHandSide,
                                   ScalarType eps,
                                   bool verbose);

} // namespace mitk

#endif // __mitkMultiLabelSegmentation_H_
