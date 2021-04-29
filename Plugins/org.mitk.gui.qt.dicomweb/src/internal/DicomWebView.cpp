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
#include <Poco/File.h>
#include <itkFileTools.h>

// Qt
#include <QMessageBox>
#include <QProgressBar>

// mitk 
#include "DicomWebView.h"


#include "mitkLabelSetImage.h"
#include <mitkWorkbenchUtil.h>
#include "mitkIOUtil.h"
#include "mitkNodePredicateProperty.h"
#include "mitkImage.h"
#include "mitkNodePredicateDataType.h"
#include <mitkDICOMDCMTKTagScanner.h>
#include <mitkIOMimeTypes.h>
//#include <mitkDICOMSegIOMimeTypes.h>

const std::string DicomWebView::VIEW_ID = "org.mitk.views.dicomwebview";

void DicomWebView::SetFocus() {}

void DicomWebView::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);
  m_Parent = parent;

  SetPredicates();

  


  qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
  qRegisterMetaType<std::vector<double>>("std::vector<double>");
  qRegisterMetaType<DicomWebRequestHandler::DicomDTO>("DicomDTO");

  m_Controls.cleanDicomBtn->setVisible(true);



  connect(m_Controls.buttonUpload, &QPushButton::clicked, this, &DicomWebView::UploadNewSegmentation);
  connect(m_Controls.cleanDicomBtn, &QPushButton::clicked, this, &DicomWebView::CleanDicomFolder);
  connect(m_Controls.restartConnection, &QPushButton::clicked, this, &DicomWebView::OnRestartConnection);
  connect(m_Controls.patImageSelector,
          SIGNAL(CurrentSelectionChanged(QList<mitk::DataNode::Pointer>)),
          this,
          SLOT(OnPatientSelectionChanged(QList<mitk::DataNode::Pointer>)));
  connect(m_Controls.segImageSelector,
          SIGNAL(CurrentSelectionChanged(QList<mitk::DataNode::Pointer>)),
          this,
          SLOT(OnSegmentationSelectionChanged(QList<mitk::DataNode::Pointer>)));

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
  utility::string_t pacsURL = U("");// e.g. http://10.128.129.166:8080
  auto envPacsURL = std::getenv("PACS_URL");

  if (envPacsURL)
  {
    pacsURL = mitk::RESTUtil::convertToTString(std::string(envPacsURL));
  }

  m_RequestHandler = new DicomWebRequestHandler(m_DownloadBaseDir, pacsURL);

  connect(this, &DicomWebView::InvokeProgress, this, &DicomWebView::AddProgress);
  connect(m_RequestHandler, &DicomWebRequestHandler::InvokeProgress, this, &DicomWebView::AddProgress);
  connect(
    m_RequestHandler, &DicomWebRequestHandler::InvokeUpdateDcmMeta, this, &DicomWebView::InitializeDcmMeta);
  connect(m_RequestHandler, &DicomWebRequestHandler::InvokeLoadData, this, &DicomWebView::LoadData);

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

  // Should be done last, if everything else is configured because it triggers the autoselection of data.
  m_Controls.patImageSelector->SetAutoSelectNewNodes(true);
  m_Controls.segImageSelector->SetAutoSelectNewNodes(true);
  //RestartConnection(pacsURL);
}

void DicomWebView::SetPredicates() {
  mitk::TNodePredicateDataType<mitk::Image>::Pointer isImage = mitk::TNodePredicateDataType<mitk::Image>::New();
  mitk::NodePredicateDataType::Pointer isDwi = mitk::NodePredicateDataType::New("DiffusionImage");
  mitk::NodePredicateDataType::Pointer isDti = mitk::NodePredicateDataType::New("TensorImage");
  mitk::NodePredicateDataType::Pointer isOdf = mitk::NodePredicateDataType::New("OdfImage");
  auto isSegment = mitk::NodePredicateDataType::New("Segment");

  mitk::NodePredicateOr::Pointer validImages = mitk::NodePredicateOr::New();
  validImages->AddPredicate(mitk::NodePredicateAnd::New(isImage, mitk::NodePredicateNot::New(isSegment)));
  validImages->AddPredicate(isDwi);
  validImages->AddPredicate(isDti);
  validImages->AddPredicate(isOdf);

  m_IsNotAHelperObject =
    mitk::NodePredicateNot::New(mitk::NodePredicateProperty::New("helper object", mitk::BoolProperty::New(true)));

  m_IsOfTypeImagePredicate = mitk::NodePredicateAnd::New(validImages, m_IsNotAHelperObject);

  mitk::NodePredicateProperty::Pointer isBinaryPredicate =
    mitk::NodePredicateProperty::New("binary", mitk::BoolProperty::New(true));
  mitk::NodePredicateNot::Pointer isNotBinaryPredicate = mitk::NodePredicateNot::New(isBinaryPredicate);

  mitk::NodePredicateAnd::Pointer isABinaryImagePredicate =
    mitk::NodePredicateAnd::New(m_IsOfTypeImagePredicate, isBinaryPredicate);
  mitk::NodePredicateAnd::Pointer isNotABinaryImagePredicate =
    mitk::NodePredicateAnd::New(m_IsOfTypeImagePredicate, isNotBinaryPredicate);

  m_IsASegmentationImagePredicate =
    mitk::NodePredicateOr::New(isABinaryImagePredicate, mitk::TNodePredicateDataType<mitk::LabelSetImage>::New());
  m_IsAPatientImagePredicate = mitk::NodePredicateAnd::New(
    isNotABinaryImagePredicate, mitk::NodePredicateNot::New(mitk::TNodePredicateDataType<mitk::LabelSetImage>::New()));

  m_Controls.patImageSelector->SetDataStorage(GetDataStorage());
  m_Controls.patImageSelector->SetNodePredicate(m_IsAPatientImagePredicate);
  m_Controls.patImageSelector->SetSelectionIsOptional(false);
  m_Controls.patImageSelector->SetInvalidInfo("Select an image.");
  m_Controls.patImageSelector->SetPopUpTitel("Select an image.");
  m_Controls.patImageSelector->SetPopUpHint(
    "Select an image that should be used and could be uploaded."); // TODO: set Image to uplaod

  m_Controls.segImageSelector->SetDataStorage(GetDataStorage());
  m_Controls.segImageSelector->SetNodePredicate(m_IsASegmentationImagePredicate);
  m_Controls.segImageSelector->SetSelectionIsOptional(false);
  m_Controls.segImageSelector->SetInvalidInfo("Select a segmentation.");
  m_Controls.segImageSelector->SetPopUpTitel("Select a segmentation.");
  m_Controls.segImageSelector->SetPopUpHint("Select a segmentation that should be uploaded.");
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

  return m_RequestHandler->DicomWebGet().SendQIDO(seriesInstancesParams, m_Controls.useSystemProxy->isChecked()).then([=](pplx::task<web::json::value> resultTask) {
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

  utility::string_t url = newHost + U("/dcm4chee-arc/aets/KAAPANA/");

  MITK_INFO << "Restarting connection to " << mitk::RESTUtil::convertToUtf8(url) << " ...";
  m_Controls.connectionStatus->setText(QString("Restarting connection..."));
  m_Controls.dcm4cheeURL->setText({(utility::conversions::to_utf8string(url).c_str())});

  m_RequestHandler->UpdateDicomWebUrl(url);

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


mitk::DataStorage::SetOfObjects::Pointer DicomWebView::LoadData(std::vector<std::string> filePathList)
{
  MITK_INFO << "Pushing data to data storage ...";
  auto ds = GetDataStorage();
  auto dataNodes = mitk::IOUtil::Load(filePathList, *ds);
  // reinit view
  mitk::RenderingManager::GetInstance()->InitializeViewsByBoundingObjects(ds);
  return dataNodes;
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
  auto segNote = m_Controls.segImageSelector->GetSelectedNode();
  if (!segNote)
    return;
  AddProgress(10, {"save SEG to temp folder"});
  std::string folderPathSeg = mitk::IOUtil::CreateTemporaryDirectory("XXXXXX", m_UploadBaseDir) + "/";

  const std::string savePath = folderPathSeg + segNote->GetName() + ".dcm";
  const std::string mimeType = mitk::IOMimeTypes::DEFAULT_BASE_NAME() + ".image.dicom.seg";
  mitk::IOUtil::Save(segNote->GetData(), mimeType, savePath);

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
    m_RequestHandler->DicomWebGet().SendSTOW(filePath, mitk::RESTUtil::convertToTString(m_CurrentStudyUID), m_Controls.useSystemProxy->isChecked()).then([=] {
      emit InvokeProgress(50, {"persist reworked SEG to evaluation database"});

      MITK_INFO << "successfully stored";
      emit InvokeProgress(30, {"successfully stored"});

      // TODO update content on server?? needed?
     // mitk::DICOMweb::MitkUriBuilder queryBuilder(m_restURL + U("/tasks/evaluations/"));
      //TODO update content on server?? needed?
      //queryBuilder.append_query(U("srUID"), utility::conversions::to_string_t(m_SRUID));
          

      //m_ManagerService->SendRequest(queryBuilder.to_uri(), mitk::IRESTManager::RequestType::Get)
      //  .then([=](web::json::value result) {
      //    MITK_INFO << "after GET";
      //    MITK_INFO << utility::conversions::to_utf8string(result.serialize());
      //    auto updatedContent = result.as_array()[0];
      //    updatedContent[U("reworkedSegmentationUID")] =
      //      web::json::value::string(utility::conversions::to_string_t(segSeriesUID));

      //    auto id = updatedContent.at(U("id")).as_integer();
      //    MITK_INFO << id;
      //    auto idParam = std::to_string(id).append("/");

      //    mitk::DICOMweb::MitkUriBuilder queryBuilder(m_restURL + U("/tasks/evaluations"));
      //    queryBuilder.append_path(utility::conversions::to_string_t(idParam));

      //    m_ManagerService->SendJSONRequest(queryBuilder.to_uri(), mitk::IRESTManager::RequestType::Put, &updatedContent)
      //      .then([=](web::json::value result) {
      //        MITK_INFO << utility::conversions::to_utf8string(result.serialize());
      //        if (result[U("reworkedSegmentationUID")].as_string() == utility::conversions::to_string_t(segSeriesUID))
      //        {
      //          MITK_INFO << "successfully stored";
      //          emit InvokeProgress(30, {"successfully stored"});
      //        }
      //      });
      //  });
    });
  }
  catch (const std::exception &exception)
  {
    std::cout << exception.what() << std::endl;
  }
  catch(...)
  {
    MITK_ERROR << "Error in sending segmentation";
  }
}


void DicomWebView::OnPatientSelectionChanged(QList<mitk::DataNode::Pointer> nodes)
{
  if (!nodes.empty())
  {
    auto node = nodes.first();
    m_Controls.segImageSelector->SetNodePredicate(m_IsASegmentationImagePredicate);

    mitk::DataNode *segNode = m_Controls.segImageSelector->GetSelectedNode();
    if (segNode)
    {
      // Doing this we can assure that the segmenation is always visible if the segmentation and the patient image are
      // loaded separately
      int layer(10);
      node->GetIntProperty("layer", layer);
      layer++;
      segNode->SetProperty("layer", mitk::IntProperty::New(layer));
      m_Controls.buttonUpload->setEnabled(true);
    }
  }
  else
  {
    m_Controls.segImageSelector->SetNodePredicate(m_IsASegmentationImagePredicate);
    m_Controls.buttonUpload->setEnabled(false);
  }
}

void DicomWebView::OnSegmentationSelectionChanged(QList<mitk::DataNode::Pointer> nodes)
{
  m_Controls.buttonUpload->setEnabled(false);
  if (nodes.empty())
    return;
  

  auto refNode = m_Controls.patImageSelector->GetSelectedNode();
  auto segNode = nodes.front();

  if (!refNode)
    return;

  if (segNode)
  {
    // Doing this we can assure that the segmenation is always visible if the segmentation and the patient image are
    // loaded separately
    int layer(10);
    refNode->GetIntProperty("layer", layer);
    layer++;
    segNode->SetProperty("layer", mitk::IntProperty::New(layer));
    m_Controls.buttonUpload->setEnabled(true);
  }
}

void DicomWebView::CleanDicomFolder()
{
  auto downloadDir = Poco::File(m_DownloadBaseDir);
  downloadDir.remove(true);
}
