/*===================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center,
Division of Medical and Biological Informatics.
All rights reserved.

This software is distributed WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See LICENSE.txt or http://www.mitk.org for details.

===================================================================*/

// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "DicomWebView.h"

#include <Poco/File.h>
#include <itkFileTools.h>

// Qt
#include <QMessageBox>
#include <QProgressBar>

// mitk 
#include <QmitkNewSegmentationDialog.h>
#include <mitkExtractSliceFilter.h>
#include <mitkIOUtil.h>
#include <mitkImagePixelWriteAccessor.h>
#include <mitkOverwriteSliceImageFilter.h>
#include <mitkSegTool2D.h>
#include <mitkSegmentationInterpolationController.h>
#include <mitkToolManagerProvider.h>
#include <mitkVtkImageOverwrite.h>

#include <usGetModuleContext.h>
#include <usModule.h>
#include <usModuleRegistry.h>
#include <usServiceTracker.h>

#include <mitkDICOMweb.h>

#include <mitkDICOMDCMTKTagScanner.h>
#include <mitkDICOMSegIOMimeTypes.h>

const std::string DicomWebView::VIEW_ID = "org.mitk.views.dicomwebview";

void DicomWebView::SetFocus() {}

void DicomWebView::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);
  m_Parent = parent;

  qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
  qRegisterMetaType<std::vector<double>>("std::vector<double>");
  qRegisterMetaType<DicomWebRequestHandler::DicomDTO>("DicomDTO");

  m_Controls.cleanDicomBtn->setVisible(true);
  m_Controls.individualWidget_2->setVisible(false);

  m_Controls.sliderWidget->setMinimum(1);
  m_Controls.sliderWidget->setMaximum(100);
  m_Controls.sliderWidget->setTickInterval(1);
  m_Controls.sliderWidget->setSingleStep(1);
  m_Controls.radioA->setChecked(true);

  connect(m_Controls.buttonUpload, &QPushButton::clicked, this, &DicomWebView::UploadNewSegmentation);
  connect(m_Controls.buttonNewSeg, &QPushButton::clicked, this, &DicomWebView::CreateNewSegmentationC);
  connect(m_Controls.cleanDicomBtn, &QPushButton::clicked, this, &DicomWebView::CleanDicomFolder);
  connect(m_Controls.restartConnection, &QPushButton::clicked, this, &DicomWebView::OnRestartConnection);
  connect(m_Controls.testConnection, &QPushButton::clicked, this, &DicomWebView::TestConnection);
  connect(m_Controls.checkIndiv, &QCheckBox::stateChanged, this, &DicomWebView::OnIndividualCheckChange);
  connect(
    m_Controls.sliderWidget, &ctkSliderWidget::valueChanged, this, &DicomWebView::OnSliderWidgetChanged);

  m_DownloadBaseDir = mitk::IOUtil::GetTempPath() + "segrework";
  MITK_INFO << "using download base dir: " << m_DownloadBaseDir;
  m_UploadBaseDir = mitk::IOUtil::GetTempPath() + "uploadSeg";

  if (!itksys::SystemTools::FileIsDirectory(m_DownloadBaseDir))
  {
    itk::FileTools::CreateDirectory(m_DownloadBaseDir);
  }

  if (!itksys::SystemTools::FileIsDirectory(m_UploadBaseDir))
  {
    itk::FileTools::CreateDirectory(m_UploadBaseDir);
  }

  m_HostURL = U("http://localhost");
  auto host = m_HostURL;
  auto envHostURL = std::getenv("HOST_URL");
  if (envHostURL)
  {
    MITK_INFO << "host url " << std::string(envHostURL);
    m_HostURL = mitk::RESTUtil::convertToTString(std::string(envHostURL));
    host = m_HostURL;
  }
  else
  {
    host = host.append(U(":8000"));
  }

  m_restURL = host.append(U("/rest-srs"));
  MITK_INFO << "rest url: " << mitk::RESTUtil::convertToUtf8(m_restURL);
  utility::string_t pacsURL = U("http://193.174.48.78:8090");
  auto envPacsURL = std::getenv("PACS_URL");

  if (envPacsURL)
  {
    pacsURL = mitk::RESTUtil::convertToTString(std::string(envPacsURL));
  }

  m_RequestHandler = new DicomWebRequestHandler(m_DownloadBaseDir, pacsURL);

  connect(this, &DicomWebView::InvokeProgress, this, &DicomWebView::AddProgress);
  connect(m_RequestHandler, &DicomWebRequestHandler::InvokeProgress, this, &DicomWebView::AddProgress);
  connect(
    m_RequestHandler, &DicomWebRequestHandler::InvokeSimilarityGraph, this, &DicomWebView::SetSimilarityGraph);
  connect(
    m_RequestHandler, &DicomWebRequestHandler::InvokeUpdateDcmMeta, this, &DicomWebView::InitializeDcmMeta);
  connect(m_RequestHandler, &DicomWebRequestHandler::InvokeLoadData, this, &DicomWebView::LoadData);
  connect(
    m_RequestHandler, &DicomWebRequestHandler::InvokeLoadDataSegDicomWeb, this, &DicomWebView::LoadDataSegDicomWeb);

  // Get the micro service
  us::ModuleContext *context = us::ModuleRegistry::GetModule(1)->GetModuleContext();
  auto managerRef = context->GetServiceReference<mitk::IRESTManager>();
  if (managerRef)
  {
    auto managerService = context->GetService(managerRef);
    if (managerService)
    {
      m_ManagerService = managerService;
    }
  }

  RestartConnection(pacsURL);
}

void DicomWebView::OnSliderWidgetChanged(double value)
{
  std::map<double, double>::iterator it;
  unsigned int count = 0;
  for (it = m_ScoreMap.begin(); it != m_ScoreMap.end(); it++)
  {
    if (it->second < value)
    {
      count++;
    }
  }
  QString labelsToDelete = "slices to delete: " + QString::number(count);
  m_Controls.slicesToDeleteLabel->setText(labelsToDelete);

  std::map<double, double> thresholdMap;

  for (it = m_ScoreMap.begin(); it != m_ScoreMap.end(); it++)
  {
    thresholdMap.insert(std::map<double, double>::value_type(it->first, value));
  }

  m_Controls.chartWidget->RemoveData(m_thresholdLabel);
  m_Controls.chartWidget->AddData2D(thresholdMap, m_thresholdLabel);
  m_Controls.chartWidget->SetChartType(m_thresholdLabel, QmitkChartWidget::ChartType::line);
  m_Controls.chartWidget->Show();
}

void DicomWebView::AddProgress(int progress, QString status)
{
  auto futureValue = m_Controls.progressBar->value() + progress;
  if (futureValue >= 100)
  {
    m_Controls.progressBar->setValue(0);
    m_Controls.progressBar->setFormat("");
  }
  else
  {
    m_Controls.progressBar->setFormat(status.append(" %p%"));
    m_Controls.progressBar->setValue(futureValue);
  }
}

void DicomWebView::InitializeDcmMeta(DicomWebRequestHandler::DicomDTO dto)
{
  m_CurrentStudyUID = mitk::RESTUtil::convertToUtf8(dto.studyUID);
  m_SRUID = mitk::RESTUtil::convertToUtf8(dto.srSeriesUID);
  m_GroundTruth = mitk::RESTUtil::convertToUtf8(dto.groundTruth);
}

void DicomWebView::Activated()
{
  StartServer();
  MITK_INFO << "activated";
}

void DicomWebView::Deactivated()
{
  MITK_INFO << "deactivated";
  m_ManagerService->HandleDeleteObserver(m_RequestHandler);
}

void DicomWebView::Visible()
{
  MITK_INFO << "visible";
}

void DicomWebView::Hidden()
{
  MITK_INFO << "hidden";
}

pplx::task<bool> DicomWebView::TestConnection()
{
  mitk::RESTUtil::ParamMap seriesInstancesParams;

  seriesInstancesParams.insert(mitk::RESTUtil::ParamMap::value_type(U("limit"), U("1")));
  m_Controls.connectionStatus->setText(QString("Testing connection ..."));

  return m_DicomWeb.SendQIDO(seriesInstancesParams).then([=](pplx::task<web::json::value> resultTask) {
    try
    {
      auto result = resultTask.get();
      if (!result.is_null())
      {
        m_Controls.connectionStatus->setText(QString("Connection works!"));
        return true;
      }
      else
      {
        m_Controls.connectionStatus->setText(QString("Trouble with connection. Not valid!"));
        return false;
      }
    }
    catch (mitk::Exception &e)
    {
      MITK_WARN << e.what();
      m_Controls.connectionStatus->setText(QString("No connection possible."));
      return false;
    }
  });
}

void DicomWebView::OnRestartConnection()
{
  RestartConnection(mitk::RESTUtil::convertToTString(m_Controls.dcm4cheeHostValue->text().toStdString()));
}

void DicomWebView::RestartConnection(utility::string_t newHost)
{
  utility::string_t host;
  if (newHost.empty())
  {
    MITK_INFO << "Host was empty";
    m_Controls.connectionStatus->setText(QString("Host must not be empty!"));
    return;
  }

  utility::string_t url = newHost + U("/dcm4chee-arc/aets/DCM4CHEE/");

  MITK_INFO << "Restarting connection to " << mitk::RESTUtil::convertToUtf8(url) << " ...";
  m_Controls.connectionStatus->setText(QString("Restarting connection..."));
  m_Controls.dcm4cheeURL->setText({(utility::conversions::to_utf8string(url).c_str())});

  m_DicomWeb = mitk::DICOMweb(url);

  if (!TestConnection().get())
  {
    MITK_INFO << "Restart did not work..";
    m_Controls.connectionStatus->setText(QString("No PACS server available under given host!"));
  }
  else
  {
    MITK_INFO << "requests to pacs are sent to: " << mitk::RESTUtil::convertToUtf8(url);
  }
}

void DicomWebView::OnIndividualCheckChange(int state)
{
  if (state == Qt::Unchecked)
  {
    m_Controls.individualWidget_2->setVisible(false);
  }
  else if (state == Qt::Checked)
  {
    m_Controls.individualWidget_2->setVisible(true);
  }
}

std::string DicomWebView::GetAlgorithmOfSegByPath(std::string path)
{
  auto scanner = mitk::DICOMDCMTKTagScanner::New();

  mitk::DICOMTagPath algorithmName;
  algorithmName.AddAnySelection(0x0062, 0x0002).AddElement(0x0062, 0x0009);

  mitk::StringList files;
  files.push_back(path);
  scanner->SetInputFiles(files);
  scanner->AddTagPath(algorithmName);

  scanner->Scan();

  mitk::DICOMDatasetAccessingImageFrameList frames = scanner->GetFrameInfoList();
  auto findings = frames.front()->GetTagValueAsString(algorithmName);
  if (findings.size() != 0)
    MITK_INFO << findings.front().value;
  return findings.front().value;
}

mitk::DataStorage::SetOfObjects::Pointer DicomWebView::LoadData(std::vector<std::string> filePathList)
{
  MITK_INFO << "Pushing data to data storage ...";
  auto ds = GetDataStorage();
  auto dataNodes = mitk::IOUtil::Load(filePathList, *ds);
  // reinit view
  mitk::RenderingManager::GetInstance()->InitializeViewsByBoundingObjects(ds);
  return dataNodes;
}

void DicomWebView::LoadDataSegDicomWeb(std::vector<std::string> filePathList)
{
  auto dataNodes = LoadData(filePathList);

  // find data nodes
  m_Image = dataNodes->at(0);
  m_Image->SetName("image data");
  m_SegA = dataNodes->at(1);
  m_SegB = dataNodes->at(2);

  auto algorithmNameA = GetAlgorithmOfSegByPath(filePathList[1]);
  auto algorithmNameB = GetAlgorithmOfSegByPath(filePathList[2]);
  m_SegA->SetName(algorithmNameA);
  m_SegB->SetName(algorithmNameB);
  m_Controls.labelSegAValue->setText(algorithmNameA.c_str());
  m_Controls.labelSegBValue->setText(algorithmNameB.c_str());
  m_Controls.labelGroundTruthValue->setText(m_GroundTruth.c_str());
  emit InvokeProgress(20, {""});
}

void DicomWebView::SetSimilarityGraph(std::vector<double> simScoreArray, int sliceMinStart)
{
  std::string label = "similarity graph";
  m_thresholdLabel = "threshold";

  double sliceIndex = sliceMinStart;
  for (double score : simScoreArray)
  {
    m_ScoreMap.insert(std::map<double, double>::value_type(sliceIndex, score));
    sliceIndex++;
  }

  std::map<double, double> thresholdMap;

  m_Controls.chartWidget->AddData2D(m_ScoreMap, label);
  m_Controls.chartWidget->AddData2D(thresholdMap, m_thresholdLabel);
  m_Controls.chartWidget->SetChartType(label, QmitkChartWidget::ChartType::line);
  m_Controls.chartWidget->SetChartType(m_thresholdLabel, QmitkChartWidget::ChartType::line);
  m_Controls.chartWidget->SetXAxisLabel("slice number");
  m_Controls.chartWidget->SetYAxisLabel("similarity in percent");
  m_Controls.chartWidget->SetTitle("Similartiy Score for Segmentation Comparison");
  m_Controls.chartWidget->Show();
}

void DicomWebView::StartServer()
{
  utility::string_t path = U("/inject");

  auto pathEnv = std::getenv("SERVER_PATH");
  if (pathEnv)
  {
    path = mitk::RESTUtil::convertToTString(std::string(pathEnv));
  }

  utility::string_t port = U(":4040");
  utility::string_t address = U("http://+");
  if (m_HostURL == U("http://localhost"))
    address = m_HostURL;

  address.append(port);
  address.append(path);
  // Setup listening server
  m_ManagerService->ReceiveRequest(address, m_RequestHandler);
  MITK_INFO << "Listening for requests at: " << utility::conversions::to_utf8string(address);
}

void DicomWebView::UploadNewSegmentation()
{
  AddProgress(10, {"save SEG to temp folder"});
  std::string folderPathSeg = mitk::IOUtil::CreateTemporaryDirectory("XXXXXX", m_UploadBaseDir) + "/";

  const std::string savePath = folderPathSeg + m_SegC->GetName() + ".dcm";
  const std::string mimeType = mitk::MitkDICOMSEGIOMimeTypes::DICOMSEG_MIMETYPE_NAME();
  mitk::IOUtil::Save(m_SegC->GetData(), mimeType, savePath);

  // get Series Instance UID from new SEG
  auto scanner = mitk::DICOMDCMTKTagScanner::New();
  mitk::DICOMTagPath seriesUID(0x0020, 0x000E);

  mitk::StringList files;
  files.push_back(savePath);
  scanner->SetInputFiles(files);
  scanner->AddTagPath(seriesUID);

  scanner->Scan();

  mitk::DICOMDatasetAccessingImageFrameList frames = scanner->GetFrameInfoList();
  auto findings = frames.front()->GetTagValueAsString(seriesUID);
  auto segSeriesUID = findings.front().value;

  AddProgress(20, {"push SEG to PACS"});
  auto filePath = utility::conversions::to_string_t(savePath);
  try
  {
    m_DicomWeb.SendSTOW(filePath, mitk::RESTUtil::convertToTString(m_CurrentStudyUID)).then([=] {
      emit InvokeProgress(50, {"persist reworked SEG to evaluation database"});

      mitk::DICOMweb::MitkUriBuilder queryBuilder(m_restURL + U("/tasks/evaluations/"));
      queryBuilder.append_query(U("srUID"), utility::conversions::to_string_t(m_SRUID));

      m_ManagerService->SendRequest(queryBuilder.to_uri(), mitk::IRESTManager::RequestType::Get)
        .then([=](web::json::value result) {
          MITK_INFO << "after GET";
          MITK_INFO << utility::conversions::to_utf8string(result.serialize());
          auto updatedContent = result.as_array()[0];
          updatedContent[U("reworkedSegmentationUID")] =
            web::json::value::string(utility::conversions::to_string_t(segSeriesUID));

          auto id = updatedContent.at(U("id")).as_integer();
          MITK_INFO << id;
          auto idParam = std::to_string(id).append("/");

          mitk::DICOMweb::MitkUriBuilder queryBuilder(m_restURL + U("/tasks/evaluations"));
          queryBuilder.append_path(utility::conversions::to_string_t(idParam));

          m_ManagerService->SendJSONRequest(queryBuilder.to_uri(), mitk::IRESTManager::RequestType::Put, &updatedContent)
            .then([=](web::json::value result) {
              MITK_INFO << utility::conversions::to_utf8string(result.serialize());
              if (result[U("reworkedSegmentationUID")].as_string() == utility::conversions::to_string_t(segSeriesUID))
              {
                MITK_INFO << "successfully stored";
                emit InvokeProgress(30, {"successfully stored"});
              }
            });
        });
    });
  }
  catch (const std::exception &exception)
  {
    std::cout << exception.what() << std::endl;
  }
}

std::vector<unsigned int> DicomWebView::CreateSegmentation(mitk::Image::Pointer baseSegmentation,
                                                                     double threshold)
{
  MITK_INFO << "handle individual segmentation creation";
  std::map<double, double>::iterator it;

  std::vector<unsigned int> sliceIndices;

  unsigned int count = 0;
  for (it = m_ScoreMap.begin(); it != m_ScoreMap.end(); it++)
  {
    if (it->second < threshold)
    {
      auto index = it->first;
      try
      {
        mitk::ImagePixelWriteAccessor<unsigned short, 3> imageAccessor(baseSegmentation);
        for (unsigned int x = 0; x < baseSegmentation->GetDimension(0); x++)
        {
          for (unsigned int y = 0; y < baseSegmentation->GetDimension(1); y++)
          {
            imageAccessor.SetPixelByIndex({{x, y, int(index)}}, 0);
          }
        }
      }
      catch (mitk::Exception &e)
      {
        MITK_ERROR << e.what();
      }

      count++;
      sliceIndices.push_back(index);
      MITK_INFO << "slice " << it->first << " removed ";
    }
  }
  MITK_INFO << "slices deleted " << count;
  return sliceIndices;
}

void DicomWebView::CreateNewSegmentationC()
{
  mitk::ToolManager *toolManager = mitk::ToolManagerProvider::GetInstance()->GetToolManager();
  toolManager->InitializeTools();
  toolManager->SetReferenceData(m_Image);

  mitk::Image::Pointer baseImage;
  if (m_Controls.radioA->isChecked())
  {
    baseImage = dynamic_cast<mitk::Image *>(m_SegA->GetData())->Clone();
  }
  else if (m_Controls.radioB->isChecked())
  {
    baseImage = dynamic_cast<mitk::Image *>(m_SegB->GetData())->Clone();
  }

  if (m_Controls.checkIndiv->isChecked())
  {
    auto sliceIndices = CreateSegmentation(baseImage, m_Controls.sliderWidget->value());
  }

  QmitkNewSegmentationDialog *dialog = new QmitkNewSegmentationDialog(m_Parent);

  int dialogReturnValue = dialog->exec();
  if (dialogReturnValue == QDialog::Rejected)
  {
    // user clicked cancel or pressed Esc or something similar
    return;
  }

  // ask the user about an organ type and name, add this information to the image's (!) propertylist
  // create a new image of the same dimensions and smallest possible pixel type
  mitk::Tool *firstTool = toolManager->GetToolById(0);
  if (firstTool)
  {
    try
    {
      std::string newNodeName = dialog->GetSegmentationName().toStdString();
      if (newNodeName.empty())
      {
        newNodeName = "no_name";
      }

      mitk::DataNode::Pointer newSegmentation =
        firstTool->CreateSegmentationNode(baseImage, newNodeName, dialog->GetColor());
      // initialize showVolume to false to prevent recalculating the volume while working on the segmentation
      newSegmentation->SetProperty("showVolume", mitk::BoolProperty::New(false));
      if (!newSegmentation)
      {
        return; // could be aborted by user
      }

      if (mitk::ToolManagerProvider::GetInstance()->GetToolManager()->GetWorkingData(0))
      {
        mitk::ToolManagerProvider::GetInstance()->GetToolManager()->GetWorkingData(0)->SetSelected(false);
      }
      newSegmentation->SetSelected(true);
      this->GetDataStorage()->Add(
        newSegmentation,
        toolManager->GetReferenceData(0)); // add as a child, because the segmentation "derives" from the original

      m_SegC = newSegmentation;

      auto referencedImages = m_Image->GetData()->GetProperty("files");
      m_SegC->GetData()->SetProperty("referenceFiles", referencedImages);
    }
    catch (std::bad_alloc)
    {
      QMessageBox::warning(
        nullptr, tr("Create new segmentation"), tr("Could not allocate memory for new segmentation"));
    }
  }
  else
  {
    MITK_INFO << "no tools...";
  }
}

void DicomWebView::CleanDicomFolder()
{
  if (m_SegA || m_SegB || m_SegC)
  {
    QMessageBox::warning(nullptr,
                         tr("Clean dicom folder"),
                         tr("Please remove the data in data storage before cleaning the download folder"));
    return;
  }

  auto downloadDir = Poco::File(m_DownloadBaseDir);
  downloadDir.remove(true);
}
