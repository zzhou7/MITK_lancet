/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "FittingToolBox.h"

// Qt
#include <QMessageBox>

// mitk image
#include "mitkNodePredicateAnd.h"
#include "mitkNodePredicateDataType.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateOr.h"
#include "mitkNodePredicateProperty.h"
#include "mitkPointSet.h"
#include "mitkSurface.h"
#include <mitkImage.h>

#include <leastsquaresfit.h>
#include <vtkArrowSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPlaneSource.h>
#include <vtkSphereSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <boost/graph/adjacency_iterator.hpp>

const std::string FittingToolBox::VIEW_ID = "org.mitk.views.fittingtoolbox";

void FittingToolBox::SetFocus()
{
  m_Controls.widget_PointSetSelector->setFocus();
}

void FittingToolBox::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);

  //PointSet NodeSelector
  InitPointSetSelector(m_Controls.widget_PointSetSelector);

  connect(m_Controls.widget_PointSetSelector, &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
      this, &FittingToolBox::OnNodeSelectionChanged);

  connect(m_Controls.pushButton_Plane, &QPushButton::clicked, this, &FittingToolBox::OnPushButtonFitPlaneClicked);
  connect(m_Controls.pushButton_Rectangle, &QPushButton::clicked, this, &FittingToolBox::OnPushButtonFitRectangleClicked);

  connect(m_Controls.pushButton_Sphere, &QPushButton::clicked, this, &FittingToolBox::OnPushButtonFitSphereClicked);
}

//void FittingToolBox::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
//                                                const QList<mitk::DataNode::Pointer> &nodes)
//{
  // iterate all selected objects, adjust warning visibility
  /*foreach (mitk::DataNode::Pointer node, nodes)
  {
    if (node.IsNotNull() && dynamic_cast<mitk::Image *>(node->GetData()))
    {
      m_Controls.labelWarning->setVisible(false);
      m_Controls.buttonPerformImageProcessing->setEnabled(true);
      return;
    }
  }

  m_Controls.labelWarning->setVisible(true);
  m_Controls.buttonPerformImageProcessing->setEnabled(false);*/
//}

void FittingToolBox::DoImageProcessing()
{
  QList<mitk::DataNode::Pointer> nodes = this->GetDataManagerSelection();
  if (nodes.empty())
    return;

  mitk::DataNode *node = nodes.front();

  if (!node)
  {
    // Nothing selected. Inform the user and return
    QMessageBox::information(nullptr, "Template", "Please load and select an image before starting image processing.");
    return;
  }

  // here we have a valid mitk::DataNode

  // a node itself is not very useful, we need its data item (the image)
  mitk::BaseData *data = node->GetData();
  if (data)
  {
    // test if this data item is an image or not (could also be a surface or something totally different)
    mitk::Image *image = dynamic_cast<mitk::Image *>(data);
    if (image)
    {
      std::stringstream message;
      std::string name;
      message << "Performing image processing for image ";
      if (node->GetName(name))
      {
        // a property called "name" was found for this DataNode
        message << "'" << name << "'";
      }
      message << ".";
      MITK_INFO << message.str();

      // actually do something here...
    }
  }
}

void FittingToolBox::InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget)
{
    widget->SetDataStorage(GetDataStorage());
    widget->SetNodePredicate(mitk::NodePredicateAnd::New(
        mitk::TNodePredicateDataType<mitk::PointSet>::New(),
        mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(
            mitk::NodePredicateProperty::New("helper object"),
            mitk::NodePredicateProperty::New("hidden object")))));

    widget->SetSelectionIsOptional(true);
    widget->SetAutoSelectNewNodes(true);
    widget->SetEmptyInfo(QString("Please select a point set"));
    widget->SetPopUpTitel(QString("Select point set"));
}

void FittingToolBox::OnNodeSelectionChanged(QmitkSingleNodeSelectionWidget::NodeList)
{
    m_PointSet = m_Controls.widget_PointSetSelector->GetSelectedNode();
}

void FittingToolBox::OnPushButtonFitPlaneClicked()
{
  if (m_PointSet==nullptr)
  {
      MITK_ERROR << "point set null";
    return;
  }
  auto Pset = dynamic_cast<mitk::PointSet*>(m_PointSet->GetData());
  std::vector<double> inp_Pset;
  std::array<double, 3> res_center, res_normal;
  for (int i = 0; i < Pset->GetSize();i++)
  {
    for(int j = 0;j<3;j++)
    {
        inp_Pset.push_back(Pset->GetPoint(i)[j]);
    }
  }
  auto res = lancetAlgorithm::fit_plane(inp_Pset, res_center, res_normal);
  if (res!=true)
  {
      MITK_ERROR << "fit_plane failed";
    return;
  }

  std::array<double, 3> endp;
  for (int i =0;i<3;i++)
  {
      endp[i] = res_center[i] + res_normal[i];
  }

  showArrow(res_center.data(), endp.data(), 5, 0, m_PointSet->GetName() + "_normal");
}

void FittingToolBox::OnPushButtonFitRectangleClicked()
{
    if (m_PointSet == nullptr)
    {
        MITK_ERROR << "point set null";
        return;
    }
    auto Pset = dynamic_cast<mitk::PointSet*>(m_PointSet->GetData());
    std::vector<double> inp_Pset;
    std::array<double, 3> res_center, res_normal,res_xaxis,res_yaxis;
    double res_wid, res_len;
    for (int i = 0; i < Pset->GetSize(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            inp_Pset.push_back(Pset->GetPoint(i)[j]);
        }
    }
    auto res = lancetAlgorithm::fit_rectangle(inp_Pset, res_center, res_normal,res_xaxis,res_yaxis,res_len,res_wid);
    if (res != true)
    {
        MITK_ERROR << "fit_rectangle failed";
        return;
    }
    std::array<double, 3> endp;
    for (int i = 0; i < 3; i++)
    {
        endp[i] = res_center[i] + res_normal[i];
    }
    showArrow(res_center.data(), endp.data(), 10, 0, m_PointSet->GetName() + "_normal");
    showRectangle(res_center, res_normal, res_xaxis, res_yaxis, res_len, res_wid, 1, m_PointSet->GetName() + "_rect");
}

void FittingToolBox::OnPushButtonFitSphereClicked()
{
    if (m_PointSet == nullptr)
    {
        MITK_ERROR << "point set null";
        return;
    }
    auto Pset = dynamic_cast<mitk::PointSet*>(m_PointSet->GetData());
    std::vector<double> inp_Pset;
    std::array<double, 3> res_center;
    double res_r;
    for (int i = 0; i < Pset->GetSize(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            inp_Pset.push_back(Pset->GetPoint(i)[j]);
        }
    }
    auto res = lancetAlgorithm::fit_sphere(inp_Pset, res_center,res_r);
    if (res != true)
    {
        MITK_ERROR << "fit_sphere failed";
        return;
    }
    showSphere(res_center, res_r, 2, m_PointSet->GetName() + "_sphere");
}

void FittingToolBox::showArrow(double p_start[], double p_end[], int factor, int color, std::string name ,bool overwrite )
{
    vtkSmartPointer<vtkArrowSource> normalArrow = vtkArrowSource::New();

    // Compute a basis
    double normalizedX[3];
    double normalizedY[3];
    double normalizedZ[3];

    // The X axis is a vector from start to end
    vtkMath::Subtract(p_end, p_start, normalizedX);
    double length = vtkMath::Norm(normalizedX);
    length *= factor;
    vtkMath::Normalize(normalizedX);

    // The Z axis is an arbitrary vector cross X
    double arbitrary[3];
    arbitrary[0] = vtkMath::Random(-10, 10);
    arbitrary[1] = vtkMath::Random(-10, 10);
    arbitrary[2] = vtkMath::Random(-10, 10);
    vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
    vtkMath::Normalize(normalizedZ);

    // The Y axis is Z cross X
    vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
    vtkSmartPointer<vtkMatrix4x4> matrix =
        vtkSmartPointer<vtkMatrix4x4>::New();

    // Create the direction cosine matrix
    matrix->Identity();
    for (unsigned int i = 0; i < 3; i++)
    {
        matrix->SetElement(i, 0, normalizedX[i]);
        matrix->SetElement(i, 1, normalizedY[i]);
        matrix->SetElement(i, 2, normalizedZ[i]);
    }

    // Apply the transforms
    vtkSmartPointer<vtkTransform> transform =
        vtkSmartPointer<vtkTransform>::New();
    transform->Translate(p_start);
    transform->Concatenate(matrix);
    transform->Scale(length, length, length);

    // Transform the polydata
    vtkSmartPointer<vtkTransformPolyDataFilter> transformPD =
        vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformPD->SetTransform(transform);
    transformPD->SetInputConnection(normalArrow->GetOutputPort());
    transformPD->Update();
    //transformPD->GetOutput();

    normalArrow->Update();
    vtkPolyData* arrowData = transformPD->GetOutput();

    mitk::Surface::Pointer arrowSurface = mitk::Surface::New();
    arrowSurface->SetVtkPolyData(arrowData);

    mitk::DataNode::Pointer arrowNode = mitk::DataNode::New();
    //arrowNode->SetName("arrow");
    arrowNode->SetData(arrowSurface);
    //sphereNode->SetProperty("Surface", mitk::BoolProperty::New(true));
    arrowNode->SetName(name);
    arrowNode->SetVisibility(true);

    switch (color)
    {
    case 0: arrowNode->SetColor(1, 0, 0);
        break;
    case 1: arrowNode->SetColor(0, 1, 0);
        break;
    case 2: arrowNode->SetColor(0, 0, 1);
        break;

    default: arrowNode->SetColor(0.5, 1, 1);
    }

    if (overwrite == true)
    {
      auto node = GetDataStorage()->GetNamedNode(name);
      if (node)
      {
          GetDataStorage()->Remove(node);
      }
    }
    
    GetDataStorage()->Add(arrowNode);
}

void FittingToolBox::showRectangle(std::array<double, 3> center,
    std::array<double, 3> normal,
    std::array<double, 3> x_axis,
    std::array<double, 3> y_axis,
    double length,
    double width,
    int color,
    std::string name,
    bool overwrite)
{
    vtkSmartPointer<vtkPlaneSource> planeSource = vtkPlaneSource::New();
    planeSource->SetOrigin(-length/2, -width/2, 0);
    planeSource->SetPoint1(length/2, -width / 2, 0);
    planeSource->SetPoint2(-length / 2, width/2, 0);
    //planeSource->SetCenter(center.data());
    //planeSource->SetNormal(normal.data());
    planeSource->Update();

    vtkMatrix4x4* matrix = vtkMatrix4x4::New();
    matrix->Identity();
    for (int i =0 ;i<3;i++)
    {
        matrix->SetElement(i, 0, x_axis[i]);
        matrix->SetElement(i, 1, y_axis[i]);
        matrix->SetElement(i, 2, normal[i]);
        matrix->SetElement(i, 3, center[i]);
    }
    vtkSmartPointer<vtkTransform> transform = vtkTransform::New();
    transform->SetMatrix(matrix);
    transform->Update();
    

    vtkSmartPointer<vtkTransformPolyDataFilter> transFilter = vtkTransformPolyDataFilter::New();
    transFilter->SetInputData(planeSource->GetOutput());
    transFilter->SetTransform(transform);
    transFilter->Update();

    mitk::Surface::Pointer planeSurface = mitk::Surface::New();
    planeSurface->SetVtkPolyData(transFilter->GetOutput());

    mitk::DataNode::Pointer planeNode = mitk::DataNode::New();
    //arrowNode->SetName("arrow");
    planeNode->SetData(planeSurface);
    //sphereNode->SetProperty("Surface", mitk::BoolProperty::New(true));
    planeNode->SetName(name);
    planeNode->SetVisibility(true);

    switch (color)
    {
    case 0: planeNode->SetColor(1, 0, 0);
        break;
    case 1: planeNode->SetColor(0, 1, 0);
        break;
    case 2: planeNode->SetColor(0, 0, 1);
        break;

    default: planeNode->SetColor(0.5, 1, 1);
    }

    if (overwrite == true)
    {
        auto node = GetDataStorage()->GetNamedNode(name);
        if (node)
        {
            GetDataStorage()->Remove(node);
        }
    }

    GetDataStorage()->Add(planeNode);
}

void FittingToolBox::showSphere(std::array<double, 3> center, double r, int color, std::string name, bool overwrite)
{
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSphereSource::New();
    sphereSource->SetCenter(center.data());
    sphereSource->SetRadius(r);
    sphereSource->SetPhiResolution(30);
    sphereSource->SetThetaResolution(30);
    sphereSource->Update();

    mitk::Surface::Pointer surface = mitk::Surface::New();
    surface->SetVtkPolyData(sphereSource->GetOutput());

    mitk::DataNode::Pointer node = mitk::DataNode::New();
    node->SetData(surface);
    node->SetName(name);
    node->SetVisibility(true);

    switch (color)
    {
    case 0: node->SetColor(1, 0, 0);
        break;
    case 1: node->SetColor(0, 1, 0);
        break;
    case 2: node->SetColor(0, 0, 1);
        break;

    default: node->SetColor(0.5, 1, 1);
    }

    if (overwrite == true)
    {
        auto tmp = GetDataStorage()->GetNamedNode(name);
        if (tmp)
        {
            GetDataStorage()->Remove(tmp);
        }
    }

    GetDataStorage()->Add(node);
}
