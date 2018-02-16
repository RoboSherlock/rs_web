#include <rs_queryanswering/RSProcessManager.h>

RSProcessManager::RSProcessManager(const bool useVisualizer, const bool &waitForServiceCall,
                                   const bool useCWAssumption, ros::NodeHandle n):
  engine_(n), inspectionEngine_(n), nh_(n), waitForServiceCall_(waitForServiceCall),
  useVisualizer_(useVisualizer), useCWAssumption_(useCWAssumption), withJsonProlog_(false), useIdentityResolution_(false),
  pause_(true), inspectFromAR_(false), visualizer_(".")
{

  outInfo("Creating resource manager");
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");

  switch(OUT_LEVEL)
  {
  case OUT_LEVEL_NOOUT:
  case OUT_LEVEL_ERROR:
    resourceManager.setLoggingLevel(uima::LogStream::EnError);
    break;
  case OUT_LEVEL_INFO:
    resourceManager.setLoggingLevel(uima::LogStream::EnWarning);
    break;
  case OUT_LEVEL_DEBUG:
    resourceManager.setLoggingLevel(uima::LogStream::EnMessage);
    break;
  }

  //TO DO: Designator response topic (string[])
  desig_pub_ = nh_.advertise<iai_robosherlock_msgs::PerceivedObjects>(std::string("result_advertiser"), 5);

  setContextService = nh_.advertiseService("set_context", &RSProcessManager::resetAECallback, this);
  jsonService = nh_.advertiseService("json_query", &RSProcessManager::jsonQueryCallback, this);
  triggerKRPoseUpdate_ = nh_.serviceClient<std_srvs::Trigger>("/qr_to_knowrob/update_object_positions");
}

RSProcessManager::~RSProcessManager()
{
  uima::ResourceManager::deleteInstance();
  outInfo("RSControledAnalysisEngine Stoped");
}

void RSProcessManager::init(std::string &xmlFile, std::string configFile)
{
  outInfo("initializing");
  queryInterface = new QueryInterface(withJsonProlog_);
  this->configFile_ = configFile;

  try
  {
    cv::FileStorage fs(cv::String(configFile), cv::FileStorage::READ);

    cv::FileNode m = fs["cw_assumption"];
    if (m.type() != cv::FileNode::SEQ)
    {
        outError("Somethings wrong with pipeline definition");
    }

    cv::FileNodeIterator it = m.begin(), it_end = m.end(); // Go through the node
    for (; it != it_end; ++it)
    {
        closedWorldAssumption_.push_back(*it);
    }

    if(lowLvlPipeline_.empty()) //if not set programatically, load from a config file
    {
      cv::FileNode n = fs["annotators"];
      if (n.type() != cv::FileNode::SEQ)
      {
          outError("Somethings wrong with pipeline definition");
      }

      cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
      for (; it != it_end; ++it)
      {
          lowLvlPipeline_.push_back(*it);
      }
    }

    fs.release();

  }
  catch(cv::Exception &e)
  {
    outWarn("No low-level pipeline defined. Setting empty!");
  }

//  getDemoObjects();

  if(inspectFromAR_)
  {
    outWarn("Inspection task will be performed usin AR markers");
    ros::service::waitForService("/qr_to_knowrob/update_object_positions");
  }

  engine_.init(xmlFile, lowLvlPipeline_);

  outInfo("Number of objects in closed world assumption: " << closedWorldAssumption_.size());
  if(!closedWorldAssumption_.empty() && useCWAssumption_)
  {
    for(auto cwa : closedWorldAssumption_)
    {
      outInfo(cwa);
    }
    engine_.setCWAssumption(closedWorldAssumption_);
  }
  if(useVisualizer_)
  {
    visualizer_.start();
  }
  outInfo("done intializing");
}

void RSProcessManager::getDemoObjects()
{
  json_prolog::Prolog pl;
  //movable parts
  json_prolog::PrologQueryProxy bdgs = pl.query("owl_subclass_of(A,'http://knowrob.org/kb/thorin_parts.owl#PlasticPiece')");

  for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
  {
    json_prolog::PrologBindings bdg = *it;
    std::string objectURI = bdg["A"].toString();

    uint16_t hpos = objectURI.find_last_of("#");
    thorinObjects_[std::string(objectURI.substr(hpos + 1, objectURI.npos))] = objectURI;

  }
  //fixtures
  bdgs = pl.query("owl_subclass_of(A,'http://knowrob.org/kb/thorin_parts.owl#PlasticFixture')");
  for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
  {
    json_prolog::PrologBindings bdg = *it;
    std::string objectURI = bdg["A"].toString();

    uint16_t hpos = objectURI.find_last_of("#");
    thorinObjects_[std::string(objectURI.substr(hpos + 1, objectURI.npos))] = objectURI;
  }
}


void RSProcessManager::setInspectionAE(std::string inspectionAEPath)
{
  outInfo("initializing inspection AE");
  std::vector<std::string> llvlp;
  llvlp.push_back("CollectionReader");
  inspectionEngine_.init(inspectionAEPath, llvlp);

}


void RSProcessManager::run()
{
  for(; ros::ok();)
  {
    processing_mutex_.lock();
    if(waitForServiceCall_ || pause_)
    {
      usleep(100000);
    }
    else
    {
      engine_.process(true);
    }
    processing_mutex_.unlock();
    usleep(100000);
    ros::spinOnce();
  }
}

void RSProcessManager::stop()
{
  if(useVisualizer_)
  {
    visualizer_.stop();
  }
  engine_.resetCas();
  engine_.stop();
}

bool RSProcessManager::resetAECallback(iai_robosherlock_msgs::SetRSContext::Request &req,
                                       iai_robosherlock_msgs::SetRSContext::Response &res)
{
  std::string newContextName = req.newAe;

  if(resetAE(newContextName))
  {
    return true;
  }
  else
  {
    outError("Contexts need to have a an AE defined");
    outInfo("releasing lock");
    processing_mutex_.unlock();
    return false;
  }
}

bool RSProcessManager::resetAE(std::string newContextName)
{
  std::string contextAEPath;
  if(rs::common::getAEPaths(newContextName, contextAEPath))
  {
    outInfo("Setting new context: " << newContextName);
    cv::FileStorage fs(cv::String(configFile_), cv::FileStorage::READ);

    cv::FileNode n = fs["annotators"];
    if (n.type() != cv::FileNode::SEQ)
    {
        outError("Somethings wrong with pipeline definition");
        return 1;
    }

    std::vector<std::string> lowLvlPipeline;
    cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; ++it)
    {
        lowLvlPipeline.push_back(*it);
    }

    fs.release();

    processing_mutex_.lock();
    this->init(contextAEPath, configFile_);
    processing_mutex_.unlock();

    //shouldn't there be an fs.release() here?

    return true;
  }
  else
  {
    return false;
  }
}

bool RSProcessManager::jsonQueryCallback(iai_robosherlock_msgs::RSQueryService::Request &req,
    iai_robosherlock_msgs::RSQueryService::Response &res)
{
  outInfo("JSON Reuqest: " << req.query);
  rapidjson::Document doc;
  doc.Parse(req.query.c_str());
  queryInterface->parseQuery(req.query);
  std::vector<std::string> newPipelineOrder;
  QueryInterface::QueryType queryType = queryInterface->processQuery(newPipelineOrder);
  processing_mutex_.lock();
  if(queryType == QueryInterface::QueryType::DETECT){

      RSQuery *query = new RSQuery();
      std::string superClass = "";
      //  rs::DesignatorWrapper::req_designator = req;
      //check Designator type...for some stupid reason req->type ==Designator::ACTION did not work

      //these are hacks,, where we need the
      query->asJson = req.query;
      outInfo("Query as Json: " << query->asJson);


      //TODO maybe get rid of the RSQuery
      if(req.query != NULL)
      {
        rapidjson::Document request;
        request.Parse(req.query.c_str());
        //these checks are there for annotators that need to know
        // about the query and the value queried for
        if(request.HasMember("timestamp"))
        {
          std::string ts = request["timestamp"].GetString();
          query->timestamp = std::stoll(ts);
          outInfo("received timestamp:" << query->timestamp);
        }
        if(request.HasMember("location"))
        {
          if(request["location"].HasMember("on"))
          {
            query->location = request["location"]["on"].GetString();

            outInfo("received location:" << query->location);
          }
          if(request["location"].HasMember("in"))
          {
            query->location = request["location"]["in"].GetString();
            outInfo("received location:" << query->location);
          }
        }
        if(request.HasMember("obj-part") || request.HasMember("inspect"))
        {
          query->objToInspect = request["obj-part"].GetString();
          outInfo("received obj-part request for object: " << query->objToInspect);
        }
        if(request.HasMember("ingredient"))
        {
          query->ingredient = request["ingredien"].GetString();
          outInfo("received request for detection ingredient: " << query->ingredient);
        }
        if(request.HasMember("type"))
        {
          superClass = request["type"].GetString();
        }
      }

      std::vector<std::string> resultDesignators;


      outInfo(FG_BLUE << "Executing Pipeline # generated by query");
      engine_.process(newPipelineOrder, true, &resultDesignators, query);
      outInfo("Executing pipeline generated by query: done");


      std::vector<std::string> filteredResponse;
      std::vector<bool> desigsToKeep;

      queryInterface->filterResults(resultDesignators, filteredResponse, desigsToKeep, superClass);


      if(useIdentityResolution_)
      {
        engine_.drawResulstOnImage<rs::Object>(desigsToKeep, resultDesignators, req.query);
      }
      else
      {
        engine_.drawResulstOnImage<rs::Cluster>(desigsToKeep, resultDesignators, req.query);
      }

      res.answer.insert(res.answer.end(), filteredResponse.begin(), filteredResponse.end());

      processing_mutex_.unlock();
      return true;
  } else if(queryType == QueryInterface::QueryType::INSPECT){
      if(!newPipelineOrder.empty())
      {
        outInfo("planned new pipeline: ");
        for(auto s : newPipelineOrder)
        {
          outInfo(s);
        }
        inspectionEngine_.setNextPipeline(newPipelineOrder);
        inspectionEngine_.applyNextPipeline();
        inspectionEngine_.process();
      }

      processing_mutex_.unlock();
      return true;
  }

  processing_mutex_.unlock();
  return false;

}


bool RSProcessManager::renderOffscreen(std::string object)
{
  RSQuery *query = new RSQuery();
  processing_mutex_.lock();

  //these are hacks,, where we need the
  query->asJson = "{\"render\":\""+object+"\"}";

  std::vector<std::string> newPipelineOrder = {"CollectionReader",
                                               "ImagePreprocessor",
                                               "RegionFilter",
                                               "PlaneAnnotator",
                                               "ObjectIdentityResolution",
                                               "GetRenderedViews"};

  std::for_each(newPipelineOrder.begin(), newPipelineOrder.end(), [](std::string & p)
  {
    outInfo(p);
  });
  std::vector<std::string> resultDesignators;
  outInfo(FG_BLUE << "Executing offscreen rendering pipeline");
  engine_.process(newPipelineOrder, true, &resultDesignators, query);
  outInfo("Executingoffscreen rendering pipeline: done");

  processing_mutex_.unlock();
  delete query;
  return true;
}




