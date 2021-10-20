/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef mitkClosedContourTool_h_Included
#define mitkClosedContourTool_h_Included

#include "mitkCommon.h"
#include <mitkFeedbackContourTool.h>
#include <MitkSegmentationExports.h>
#include <mitkContourModelLiveWireInteractor.h>

namespace mitk
{
  class Image;
  class StateMachineAction;
  class InteractionEvent;

  class MITKSEGMENTATION_EXPORT ClosedContourTool : public FeedbackContourTool
  {
  public:
    mitkClassMacro(ClosedContourTool, FeedbackContourTool);
    void ConfirmSegmentation();
    void ClearSegmentation();

  protected:
    ClosedContourTool(bool snapToContour);
    ~ClosedContourTool() override;

    
    void Activated() override;
    void Deactivated() override;

    void ConnectActionsAndFunctions() override;

    void UpdateLiveWireContour();
    void OnTimePointChanged() override;

    mitk::ContourModel::Pointer m_LiveWireContour;
    mitk::ImageLiveWireContourModelFilter::Pointer m_LiveWireFilter;
    mitk::ImageLiveWireContourModelFilter::Pointer m_LiveWireFilterClosure;

  private:
    bool m_SnapToContour;
    /// \brief Initialize tool.
    void OnInitLiveWire(StateMachineAction *, InteractionEvent *interactionEvent);

    /// \brief Add a control point and finish current segment.
    void OnAddPoint(StateMachineAction *, InteractionEvent *interactionEvent);

    /// \brief Draw a contour according to the mouse movement when mouse button is pressed and mouse is moved.
    void OnDrawing(StateMachineAction *, InteractionEvent *interactionEvent);

    /// \brief Actual LiveWire computation.
    void OnMouseMoved(StateMachineAction *, InteractionEvent *interactionEvent);

    /// \brief Check double click on first control point to finish the LiveWire tool.
    bool OnCheckPoint(const InteractionEvent *interactionEvent);

    /// \brief Finish LiveWire tool.
    void OnFinish(StateMachineAction *, InteractionEvent *interactionEvent);

    /// \brief Close the contour.
    void OnLastSegmentDelete(StateMachineAction *, InteractionEvent *interactionEvent);

    /// \brief Don't use dynamic cost map for LiveWire calculation.
    void OnMouseMoveNoDynamicCosts(StateMachineAction *, InteractionEvent *interactionEvent);

    /// \brief Finish contour interaction.
    void FinishTool();

    void EnableContourLiveWireInteraction(bool on);

    bool IsPositionEventInsideImageRegion(InteractionPositionEvent *positionEvent, BaseData *data);

    void ReleaseInteractors();

    void ReleaseHelperObjects();

    void RemoveHelperObjects();

    template <typename TPixel, unsigned int VImageDimension>
    void FindHighestGradientMagnitudeByITK(itk::Image<TPixel, VImageDimension> *inputImage,
                                           itk::Index<3> &index,
                                           itk::Index<3> &returnIndex);

    ContourModel::Pointer CreateNewContour() const;

    mitk::ContourModel::Pointer m_Contour;
    mitk::DataNode::Pointer m_ContourNode;

    
    mitk::DataNode::Pointer m_LiveWireContourNode;

    mitk::ContourModel::Pointer m_ClosureContour;
    mitk::DataNode::Pointer m_ClosureContourNode;

    mitk::ContourModel::Pointer m_EditingContour;
    mitk::DataNode::Pointer m_EditingContourNode;
    mitk::ContourModelLiveWireInteractor::Pointer m_ContourInteractor;

    /** Slice of the reference data the tool is currently actively working on to
    define contours.*/
    mitk::Image::Pointer m_ReferenceDataSlice;

    bool m_CreateAndUseDynamicCosts;

    std::vector<std::pair<mitk::DataNode::Pointer, mitk::PlaneGeometry::Pointer>> m_WorkingContours;
    std::vector<std::pair<mitk::DataNode::Pointer, mitk::PlaneGeometry::Pointer>> m_EditingContours;
    std::vector<mitk::ContourModelLiveWireInteractor::Pointer> m_LiveWireInteractors;

    PlaneGeometry::ConstPointer m_PlaneGeometry;
  };

} // namespace

#endif
