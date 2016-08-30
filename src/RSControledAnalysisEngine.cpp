#include <rs_kbreasoning/RSControledAnalysisEngine.h>

void RSControledAnalysisEngine::init(const std::string &AEFile, const std::vector<std::string> &lowLvlPipeline)
{
  uima::ErrorInfo errorInfo;

  size_t pos = AEFile.rfind('/');
  outInfo("Creating analysis engine: " FG_BLUE << (pos == AEFile.npos ? AEFile : AEFile.substr(pos)));

  engine = uima::Framework::createAnalysisEngine(AEFile.c_str(), errorInfo);

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

  rspm->setDefaultPipelineOrdering(lowLvlPipeline);
  rspm->setPipelineOrdering(lowLvlPipeline);
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
  currentAEName = AEFile;
}

void RSControledAnalysisEngine::setCWAssumption(const std::vector<std::string> &cwObjs)
{
  cwObjects_.assign(cwObjs.begin(), cwObjs.end());
}

void RSControledAnalysisEngine::process()
{
  std::vector<designator_integration::Designator> d;
  process(d);
}

void RSControledAnalysisEngine::process(
  std::vector<designator_integration::Designator> &designatorResponse,
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
      query.asJson.set(q->asJson);
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
      if(q->ingredient != "")
      {
        query.ingredient.set(q->ingredient);
      }
      outInfo("setting in CAS: ts:" << q->timestamp << " location: " << q->location);
      sceneCas.set("QUERY", query);
    }
    if(!cwObjects_.empty())
    {
      outInfo("setting list of objects for closed world assumption in the CAS");
      rs::SceneCas sceneCas(*cas);
      rs::CWAssumption cwAssump = rs::create<rs::CWAssumption>(*cas);
      cwAssump.cwObjects(cwObjects_);
      sceneCas.set("CWA", cwAssump);
    }
    outInfo("processing CAS");
    try
    {
      uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);
      for(int i = 0; casIter.hasNext(); ++i)
      {
        uima::CAS &outCas = casIter.next();

        // release CAS
        outInfo("release CAS " << i);
        engine->getAnnotatorContext().releaseCAS(outCas);
      }
    }
    catch(const rs::FrameFilterException &)
    {
      // Nothing changed, time to do something else
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
  dw.setCAS(cas);
  if(useIdentityResolution_)
  {
    dw.setMode(rs::DesignatorWrapper::OBJECT);
  }
  else
  {
    dw.setMode(rs::DesignatorWrapper::CLUSTER);
  }
  dw.getObjectDesignators(designatorResponse);

  designator_integration::Designator hDesig;
  dw.getHumanDesignator(hDesig);
  designatorResponse.push_back(hDesig);

  outInfo("processing finished");
}

// Call process() and decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(bool reset_pipeline_after_process)
{
  std::vector<designator_integration::Designator> d;
  process(reset_pipeline_after_process, d);
}

// Call process() and decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(bool reset_pipeline_after_process, std::vector<designator_integration::Designator> &designatorResponse)
{
  process_mutex->lock();
  outInfo(FG_CYAN << "process(bool,desig) - LOCK OBTAINED");
  process(designatorResponse, 0);
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
                                        std::vector<designator_integration::Designator> &designator_response,
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
  std::vector<designator_integration::Designator> d;
  process(annotators, reset_pipeline_after_process, d);
}

template <class T>
void RSControledAnalysisEngine::drawResulstOnImage(const std::vector<bool> &filter,
    const std::vector<designator_integration::Designator> &resultDesignators,
    designator_integration::Designator &requestDesignator)
{
  rs::SceneCas sceneCas(*cas);
  rs::Scene scene = sceneCas.getScene();
  cv::Mat rgb = cv::Mat::zeros(480, 640, CV_64FC3);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dispCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
  sensor_msgs::CameraInfo cam_info;

  sceneCas.get(VIEW_COLOR_IMAGE, rgb);
  sceneCas.get(VIEW_CAMERA_INFO, cam_info);
  sceneCas.get(VIEW_CLOUD, *dispCloud);
  uint64_t now = sceneCas.getScene().timestamp();
  std::vector<T> clusters;
  if(std::is_same<T, rs::Cluster>::value)
  {
    scene.identifiables.filter(clusters);
  }
  else
  {
    std::vector<rs::Object> allObjects;
    sceneCas.get(VIEW_OBJECTS, allObjects);
    outWarn("objects found: " << allObjects.size());
    for(size_t i = 0; i < allObjects.size(); ++i)
    {
      rs::Object &object = allObjects[i];
      double lastSeen = (now - (uint64_t)object.lastSeen()) / 1000000000.0;
      if(lastSeen < 100)
      {
        clusters.push_back(object);
      }
    }
  }

  int colorIdx = 0;

  for(int i = 0; i < filter.size(); ++i)
  {
    if(!filter[i])
    {
      continue;
    }
    designator_integration::Designator desig = resultDesignators[i];
    designator_integration::KeyValuePair *clusterId = desig.childForKey("ID");
    if(clusterId != NULL)
    {
      //very uglyu quick hack for now just so it does not crash
      int idx = atoi(clusterId->stringValue().c_str());
      //Draw cluster on image
      rs::ImageROI roi = clusters[idx].rois();
      cv::Rect cvRoi;
      rs::conversion::from(roi.roi(), cvRoi);
      cv::rectangle(rgb, cvRoi, rs::common::cvScalarColors[idx % rs::common::numberOfColors], 1.5);
      std::stringstream clusterName;
      clusterName << "cID_" << idx;
      cv::putText(rgb, clusterName.str(), cv::Point(cvRoi.x + 10, cvRoi.y - 10), cv::FONT_HERSHEY_COMPLEX, 0.7, rs::common::cvScalarColors[idx % rs::common::numberOfColors]);

      //Color points in Point Cloud
      pcl::PointIndicesPtr inliers(new pcl::PointIndices());
      rs::conversion::from(((rs::ReferenceClusterPoints)clusters[idx].points()).indices(), *inliers);
      for(unsigned int idx = 0; idx < inliers->indices.size(); ++idx)
      {
        dispCloud->points[inliers->indices[idx]].rgba = rs::common::colors[colorIdx % rs::common::numberOfColors];
        dispCloud->points[inliers->indices[idx]].a = 255;
      }
      colorIdx++;
    }
    designator_integration::KeyValuePair *handleKvp = desig.childForKey("HANDLE");
    if(handleKvp != NULL)
    {
      //color the pixels of the handle
      std::vector<rs::HandleAnnotation> handles;
      scene.annotations.filter(handles);
      for(int i = 0; i < handles.size(); ++i)
      {
        outInfo("Actual name: " << handles[i].name());
        outInfo("Queried name: " << handleKvp->stringValue());
        if(handles[i].name() == handleKvp->stringValue())
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

            dispCloud->points[indices.indices[j]].rgba = rs::common::colors[i % rs::common::numberOfColors];
            dispCloud->points[indices.indices[j]].a = 255;
          }
        }
      }
    }
  }

  if(requestDesignator.childForKey("OBJ-PART"))
  {
    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
      std::vector<rs::ClusterPart> parts;
      std::vector<rs_demos::Pizza> pizza;
      cluster.annotations.filter(parts);
      cluster.annotations.filter(pizza);

      for(int pIdx = 0; pIdx < parts.size(); ++pIdx)
      {
        rs::ClusterPart &part = parts[pIdx];
        if(strcasecmp(part.name().c_str(), requestDesignator.childForKey("OBJ-PART")->stringValue().c_str()) == 0 || requestDesignator.childForKey("OBJ-PART")->stringValue() == "")
        {
          pcl::PointIndices indices;
          rs::conversion::from(part.indices(), indices);
          for(int iIdx = 0; iIdx < indices.indices.size(); ++iIdx)
          {
            int idx = indices.indices[iIdx];
            rgb.at<cv::Vec3b>(cv::Point(idx % cam_info.width, idx / cam_info.width)) = rs::common::cvVec3bColors[pIdx % rs::common::numberOfColors];
            dispCloud->points[idx].rgba = rs::common::colors[colorIdx % rs::common::numberOfColors];
            dispCloud->points[idx].a = 255;
          }
          colorIdx++;
        }
      }
    }
  }
  if(requestDesignator.childForKey("INGREDIENT") || requestDesignator.childForKey("CAD-MODEL"))
  {
    if(sceneCas.has("VIEW_DISPLAY_IMAGE"))
    {
      outInfo("Scene has a display image");
      sceneCas.get("VIEW_DISPLAY_IMAGE", rgb);
    }
  }

  cv_bridge::CvImage outImgMsgs;
  outImgMsgs.header = cam_info.header;
  outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
  outImgMsgs.image = rgb;

  std::vector<uchar> imageData;
  std::vector<int> params = {CV_IMWRITE_JPEG_QUALITY, 90, 0};
  cv::imencode(".jpg", rgb, imageData, params);

  std::string encoded = rs::common::base64_encode(&imageData[0], imageData.size());

  std_msgs::String strMsg;
  strMsg.data = "data:image/jpg;base64," + encoded;
  base64ImgPub.publish(strMsg);
  image_pub_.publish(outImgMsgs.toImageMsg());


  dispCloud->header.frame_id = cam_info.header.frame_id;//"head_mount_kinect_rgb_optical_frame";
  //  dispCloud->header.stamp = ros::Time::now().toNSec();
  pc_pub_.publish(dispCloud);

}

template void RSControledAnalysisEngine::drawResulstOnImage<rs::Object>(const std::vector<bool> &filter,
    const std::vector<designator_integration::Designator> &resultDesignators,
    designator_integration::Designator &requestDesignator);

template void RSControledAnalysisEngine::drawResulstOnImage<rs::Cluster>(const std::vector<bool> &filter,
    const std::vector<designator_integration::Designator> &resultDesignators,
    designator_integration::Designator &requestDesignator);
