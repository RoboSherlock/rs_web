#include <rs_kbreasoning/RSControledAnalysisEngine.h>

void RSControledAnalysisEngine::init(const std::string &file)
{
  uima::ErrorInfo errorInfo;

  size_t pos = file.rfind('/');
  outInfo("Creating analysis engine: " FG_BLUE << (pos == file.npos ? file : file.substr(pos)));

  engine = uima::Framework::createAnalysisEngine(file.c_str(), errorInfo);

  if(errorInfo.getErrorId() != UIMA_ERR_NONE)
  {
    outError("createAnalysisEngine failed." << errorInfo.asString());
    throw uima::Exception(errorInfo);
  }
  // RSPipelineManager rspm(engine);
  rspm = new RSPipelineManager(engine);
  std::vector<icu::UnicodeString> &non_const_nodes = rspm->getFlowConstraintNodes();

  outInfo("*** Fetch the FlowConstraint nodes. Size is: "  << non_const_nodes.size());
  for(int i = 0; i < non_const_nodes.size(); i++)
  {
    std::string tempString;
    non_const_nodes.at(i).toUTF8String(tempString);
    outInfo(tempString);
  }

  rspm->aengine->getNbrOfAnnotators();
  outInfo("*** Number of Annotators in AnnotatorManager: " << rspm->aengine->getNbrOfAnnotators());

  // After all annotators have been initialized, pick the default pipeline
  std::vector<std::string> lowLvlPipeline;
  lowLvlPipeline.push_back("CollectionReader");
  lowLvlPipeline.push_back("ImagePreprocessor");
//  lowLvlPipeline.push_back("RegionFilter");
//  lowLvlPipeline.push_back("NormalEstimator");
//  lowLvlPipeline.push_back("PlaneAnnotator");
//  lowLvlPipeline.push_back("PointCloudClusterExtractor");
//  lowLvlPipeline.push_back("Cluster3DGeometryAnnotator");
//  lowLvlPipeline.push_back("ClusterTFLocationAnnotator");
//  lowLvlPipeline.push_back("SacModelAnnotator");
//  lowLvlPipeline.push_back("PrimitiveShapeAnnotator");
//  lowLvlPipeline.push_back("KBResultAdvertiser");

  // removed color histogram for tests
  rspm->setDefaultPipelineOrdering(lowLvlPipeline);

  // Get a new CAS
  outInfo("Creating a new CAS");
  cas = engine->newCAS();

  if(cas == NULL)
  {
    outError("Creating new CAS failed.");
    engine->destroy();
    delete engine;
    engine = NULL;
    throw uima::Exception(uima::ErrorMessage(UIMA_ERR_ENGINE_NO_CAS), UIMA_ERR_ENGINE_NO_CAS, uima::ErrorInfo::unrecoverable);
  }

  outInfo("initialization done: " << name << std::endl
          << std::endl << FG_YELLOW << "********************************************************************************" << std::endl);
  currentAEName = file;
}

void RSControledAnalysisEngine::process()
{
  designator_integration_msgs::DesignatorResponse d;
  process(d);
}

void RSControledAnalysisEngine::process(
  designator_integration_msgs::DesignatorResponse &designator_response,
  RSQuery *q)
{

  outInfo("executing analisys engine: " << name);
  cas->reset();
  try
  {
    UnicodeString ustrInputText;
    ustrInputText.fromUTF8(name);
    cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
    if(q != NULL)
    {
      rs::SceneCas sceneCas(*cas);
      rs::Query query = rs::create<rs::Query>(*cas);
      if(q->timestamp != 0)
      {
        query.timestamp.set(q->timestamp);
      }
      if(q->location != "")
      {
        query.location.set(q->location);
      }
      if(q->objToInspect != "")
      {
        query.inspect.set(q->objToInspect);
      }
      if(q->ingredient !="")
      {
        query.ingredient.set(q->ingredient);
      }
      outInfo("setting in CAS: ts:" << q->timestamp << " location: " << q->location);
      sceneCas.set("QUERY", query);
    }
    outInfo("processing CAS");
    uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);

    for(int i = 0; casIter.hasNext(); ++i)
    {
      uima::CAS &outCas = casIter.next();

      // release CAS
      outInfo("release CAS " << i);
      engine->getAnnotatorContext().releaseCAS(outCas);
    }
  }
  catch(const rs::Exception &e)
  {
    outError("Exception: " << std::endl << e.what());
  }
  catch(const uima::Exception &e)
  {
    outError("Exception: " << std::endl << e);
  }
  catch(const std::exception &e)
  {
    outError("Exception: " << std::endl << e.what());
  }
  catch(...)
  {
    outError("Unknown exception!");
  }
  // Make a designator from the result
  rs::DesignatorWrapper dw;
  dw.setMode(rs::DesignatorWrapper::CLUSTER);
  dw.setCAS(cas);

  designator_response = dw.getDesignatorResponseMsg();
  outInfo("processing finished");
}

// Call process() and decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(bool reset_pipeline_after_process)
{
  designator_integration_msgs::DesignatorResponse d;
  process(reset_pipeline_after_process, d);
}

// Call process() and decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process( bool reset_pipeline_after_process, designator_integration_msgs::DesignatorResponse &designator_response)
{
  process_mutex->lock();
  outInfo(FG_CYAN << "process(bool,desig) - LOCK OBTAINED");
  process(designator_response, 0);
  if(reset_pipeline_after_process)
  {
    resetPipelineOrdering();  // reset pipeline to default
  }
  process_mutex->unlock();
  outInfo(FG_CYAN << "process(bool,desig) - LOCK RELEASED");
}

// Define a pipeline that should be executed,
// process(reset_pipeline_after_process) everything and
// decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(std::vector<std::string> annotators,
                                        bool reset_pipeline_after_process,
                                        designator_integration_msgs::DesignatorResponse &designator_response,
                                        RSQuery *query)
{
  process_mutex->lock();
  outInfo(FG_CYAN << "process(std::vector, bool) - LOCK OBTAINED");
  setNextPipeline(annotators);
  applyNextPipeline();

  // Process the modified pipeline
  process(designator_response, query);
  if(reset_pipeline_after_process)
  {
    resetPipelineOrdering();  // reset pipeline to default
  }
  process_mutex->unlock();
  outInfo(FG_CYAN << "process(std::vector, bool) - LOCK RELEASED");
}

// Define a pipeline that should be executed,
// process(reset_pipeline_after_process) everything and
// decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(std::vector<std::string> annotators, bool reset_pipeline_after_process)
{
  designator_integration_msgs::DesignatorResponse d;
  process(annotators, reset_pipeline_after_process, d);
}

void RSControledAnalysisEngine::drawResulstOnImage(const std::vector<bool> &filter,
                        const std::vector<designator_integration::Designator> &resultDesignators)
{
  rs::SceneCas sceneCas(*cas);
  rs::Scene scene = sceneCas.getScene();

  cv::Mat rgb;
  sensor_msgs::CameraInfo cam_info;
  sceneCas.get(VIEW_COLOR_IMAGE, rgb);
  sceneCas.get(VIEW_CAMERA_INFO, cam_info);

  std::vector<rs::Cluster> clusters;
  scene.identifiables.filter(clusters);

  //very hacky for now
  for(int i = 0; i < clusters.size(); ++i)
  {
    rs::Cluster &cluster = clusters[i];
    std::vector<rs::ClusterPart> parts;
    cluster.annotations.filter(parts);
    if(parts.empty())
    {
      continue;
    }
    for(int pIdx = 0; pIdx < parts.size(); ++pIdx)
    {
      rs::ClusterPart &part = parts[pIdx];
      pcl::PointIndices indices;
      rs::conversion::from(part.indices(), indices);
      for(int iIdx = 0; iIdx < indices.indices.size(); ++iIdx)
      {
        int idx = indices.indices[iIdx];
        rgb.at<cv::Vec3b>(cv::Point(idx % 640, idx / 640)) =rs::common::cvVec3bcolorsVec[pIdx%rs::common::cvVec3bcolorsVec.size()];
      }
    }
  }

  for(int i = 0; i < filter.size(); ++i)
  {

    if(!filter[i])
    {
      continue;
    }

    designator_integration::Designator desig = resultDesignators[i];
    designator_integration::KeyValuePair *clusterId = desig.childForKey("CLUSTERID");
    if(clusterId != NULL)
    {
      int idx = atoi(clusterId->stringValue().c_str());
      outInfo("Draw cluster: " << idx);
      rs::ImageROI roi = clusters[idx].rois();
      cv::Rect cvRoi;
      rs::conversion::from(roi.roi(), cvRoi);
      cv::rectangle(rgb, cvRoi, rs::common::cvScalarColorsVec[idx % rs::common::cvScalarColorsVec.size()], 1.5);
      std::stringstream clusterName;
      clusterName << "cID_" << idx;
      cv::putText(rgb, clusterName.str(), cv::Point(cvRoi.x + 10, cvRoi.y - 10), cv::FONT_HERSHEY_COMPLEX, 0.7, rs::common::cvScalarColorsVec[idx % rs::common::cvScalarColorsVec.size()]);
    }
    designator_integration::KeyValuePair *handleKvp = desig.childForKey("type");
    if(handleKvp != NULL)
    {
      //color the pixels of the handle
      std::vector<rs::HandleAnnotation> handles;
      scene.annotations.filter(handles);
      for(int i = 0; i < handles.size(); ++i)
      {
        pcl::PointIndices indices;
        rs::conversion::from(handles[i].indices(), indices);
        outInfo("Number of inliers in handle " << i << ": " << indices.indices.size());
        for(int j = 0; j < indices.indices.size(); ++j)
        {
          int idx = indices.indices[j];
          cv::Vec3b new_color;
          new_color[0] = 0;
          new_color[0] = 0;
          new_color[0] = 255;
          rgb.at<cv::Vec3b>(cv::Point(idx % 640, idx / 640)) = new_color;
        }
      }
    }
  }
  cv_bridge::CvImage outImgMsgs;
  outImgMsgs.header = cam_info.header;
  outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
  outImgMsgs.image = rgb;

  std::vector<uchar> imageData;
  std::vector<int> params ={CV_IMWRITE_JPEG_QUALITY,90,0};
  cv::imencode(".jpg",rgb,imageData,params);

  std::string encoded = rs::common::base64_encode(&imageData[0], imageData.size());


  std_msgs::String strMsg;
  strMsg.data = "data:image/jpg;base64,"+encoded;
  base64ImgPub.publish(strMsg);
  image_pub_.publish(outImgMsgs.toImageMsg());
}

