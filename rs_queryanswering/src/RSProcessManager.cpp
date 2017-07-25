#include <rs_queryanswering/RSProcessManager.h>

using namespace designator_integration;


RSProcessManager::RSProcessManager(const bool useVisualizer, const std::string &savePath,
                                   const bool &waitForServiceCall, const bool useCWAssumption, ros::NodeHandle n):
  engine_(n), inspectionEngine_(n), nh_(n), waitForServiceCall_(waitForServiceCall),
  useVisualizer_(useVisualizer), useCWAssumption_(useCWAssumption), withJsonProlog_(false), useIdentityResolution_(false),
  pause_(true), visualizer_(savePath), inspectFromAR_(false), seenObjects_()
{

  outInfo("Creating resource manager"); // TODO: DEBUG
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock"); // TODO: change topic?

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

  desig_pub_ = nh_.advertise<designator_integration_msgs::DesignatorResponse>(std::string("result_advertiser"), 5);

  // Call this service, if RoboSherlock should try out only
  // the pipeline with all Annotators, that provide the requested types (for example shape)
  singleService = nh_.advertiseService("designator_request/single_solution",
                                       &RSProcessManager::designatorSingleSolutionCallback, this);

  // Call this service to switch between AEs
  setContextService = nh_.advertiseService("set_context", &RSProcessManager::resetAECallback, this);

  jsonService = nh_.advertiseService("json_query", &RSProcessManager::jsonQueryCallback, this);
  triggerKRPoseUpdate_ = nh_.serviceClient<std_srvs::Trigger>("/qr_to_knowrob/update_object_positions");
  semrecClient = NULL;
  ctxMain = NULL;
}

RSProcessManager::~RSProcessManager()
{
  delete semrecClient;
  delete ctxMain;
  uima::ResourceManager::deleteInstance();
  outInfo("RSControledAnalysisEngine Stoped");
}

void RSProcessManager::init(std::string &xmlFile, std::string configFile)
{
  outInfo("initializing");
  prologInterface = new PrologInterface(withJsonProlog_);
  this->configFile_ = configFile;

  try
  {
    cv::FileStorage fs(configFile, cv::FileStorage::READ);
    fs["cw_assumption"] >> closedWorldAssumption_;
    if(lowLvlPipeline_.empty()) //if not set programatically, load from a config file
    {
      fs["annotators"] >> lowLvlPipeline_;
    }
  }
  catch(cv::Exception &e)
  {
    outWarn("No low-level pipeline defined. Setting empty!");
  }

  ros::service::waitForService("/json_prolog/simple_query");

  getDemoObjects();

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


std::string RSProcessManager::getObjectByID(std::string OID, std::string type)
{
  json_prolog::Prolog pl;
  std::string q = "knowrob_beliefstate:get_object_transform('" + OID + "', T)";
  outInfo("query: " << q);
  json_prolog::PrologQueryProxy bdgs = pl.query(q);
  for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
  {
    json_prolog::PrologBindings bdg = *it;
    outInfo("T = " << bdg["T"]);
    std::vector<json_prolog::PrologValue> poseList = bdg["T"].as<std::vector<json_prolog::PrologValue>>();
    assert(poseList.size() == 4);
    tf::StampedTransform transform;
    for(int i = 0; i < poseList.size(); ++i)
    {
      switch(i)
      {
      case 0:
        {
	  transform.frame_id_ = poseList[0].as<std::string>();
          transform.stamp_ = ros::Time::now();
          break;
        }
      case 1:
        {
          transform.child_frame_id_ = poseList[1].as<std::string>();
          break;
        }
      case 2:
        {
          std::vector<json_prolog::PrologValue> positionValues = poseList[2].as<std::vector<json_prolog::PrologValue>>();
          assert(positionValues.size() == 3);
          tf::Vector3 vec;
	  vec.setX(std::atof(positionValues[0].toString().c_str()));//sad: posigionValues[0].as<double>() sometimes segfaults :(
	  vec.setY(std::atof(positionValues[1].toString().c_str()));
	  vec.setZ(std::atof(positionValues[2].toString().c_str()));
          transform.setOrigin(vec);
          break;
        }
      case 3:
        {
          std::vector<json_prolog::PrologValue> orientationValues = poseList[3].as<std::vector<json_prolog::PrologValue>>();
          assert(orientationValues.size() == 4);
          tf::Quaternion quat;//no clue why the same conversion used for position does not work here
          quat.setX(std::atof(orientationValues[0].toString().c_str()));
          quat.setY(std::atof(orientationValues[1].toString().c_str()));
          quat.setZ(std::atof(orientationValues[2].toString().c_str()));
          quat.setW(std::atof(orientationValues[3].toString().c_str()));
          transform.setRotation(quat);
          break;
        }
      default:
        outError("How the hell did I end up here with an assert before the code ? ");
        break;
      }
    }
    outInfo("converting to Json");
    return (toJson(transform, OID, type));
  }
  return "";
}


std::string RSProcessManager::toJson(const tf::StampedTransform &pose, std::string OID, std::string type)
{
  rapidjson::StringBuffer s;
  rapidjson::Writer<rapidjson::StringBuffer> jsonWriter(s);

  //OMG writing it like this is reeeeaally shitty
  jsonWriter.StartObject();
  jsonWriter.String("id");
  jsonWriter.String(OID.c_str());
  jsonWriter.String("pose");

  jsonWriter.StartObject();
  jsonWriter.String("transform");

  jsonWriter.StartObject();
  jsonWriter.String("frame_id");
  jsonWriter.String(pose.frame_id_.c_str());
  jsonWriter.String("child_frame_id");
  jsonWriter.String(pose.child_frame_id_.c_str());
  jsonWriter.String("stamp");
  jsonWriter.Uint64(pose.stamp_.toNSec());
  jsonWriter.String("pos_x");
  jsonWriter.Double(pose.getOrigin().x());
  jsonWriter.String("pos_y");
  jsonWriter.Double(pose.getOrigin().y());
  jsonWriter.String("pos_z");
  jsonWriter.Double(pose.getOrigin().z());
  jsonWriter.String("rot_x");
  jsonWriter.Double(pose.getRotation().x());
  jsonWriter.String("rot_y");
  jsonWriter.Double(pose.getRotation().y());
  jsonWriter.String("rot_z");
  jsonWriter.Double(pose.getRotation().z());
  jsonWriter.String("rot_w");
  jsonWriter.Double(pose.getRotation().w());
  jsonWriter.EndObject();

  jsonWriter.String("source");
  jsonWriter.String("Simulation");
  jsonWriter.EndObject();

  jsonWriter.String("class");

  jsonWriter.StartObject();
  jsonWriter.String("name");
  jsonWriter.String(type.c_str());
  jsonWriter.String("confidence");
  jsonWriter.Double(1.0);
  jsonWriter.EndObject();

  jsonWriter.EndObject();

  outInfo(s.GetString());
  return s.GetString();
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
    cv::FileStorage fs(configFile_, cv::FileStorage::READ);
    std::vector<std::string> lowLvlPipeline;
    fs["annotators"] >> lowLvlPipeline;

    processing_mutex_.lock();
    this->init(contextAEPath, configFile_);
    processing_mutex_.unlock();

    return true;
  }
  else
  {
    return false;
  }
}

bool RSProcessManager::handleSemrec(const rapidjson::Document &doc)
{
  std::string value = doc["semrec"].GetString();
  if(value == "start")
  {
    if(!semrecClient)
    {
      outInfo("Starting Semantic Logging");
      semrecClient = new semrec_client::BeliefstateClient("robosherlock");
      semrecClient->setMetaDataField("experiment", "ex1");
      semrecClient->startNewExperiment();
      semrecClient->registerOWLNamespace("rs_queryanswering", "http://robosherlock.org/#");
      return true;
    }
    else
    {
      outError("Logging allready started");
      return false;
    }
  }
  else if(value == "stop")
  {
    if(semrecClient)
    {
      outInfo("Stopping Semantic Logging");
      semrecClient->exportFiles("robosherlock");
      delete semrecClient;
      semrecClient = NULL;
      return true;
    }
    else
    {
      outError("There is no logging started");
      return false;
    }
  }
  else
  {
    outError("Undefined command!");
    return false;
  }
  return true;
}

bool RSProcessManager::jsonQueryCallback(iai_robosherlock_msgs::RSQueryService::Request &req,
    iai_robosherlock_msgs::RSQueryService::Response &res)
{
  outInfo("JSON Reuqest: " << req.query);
  rapidjson::Document doc;
  doc.Parse(req.query.c_str());
  if(doc.HasMember("semrec"))
  {
    return handleSemrec(doc);
  }
  else if(doc.HasMember("detect"))
  {
    const rapidjson::Value &val = doc["detect"];
    rapidjson::StringBuffer strBuff;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strBuff);
    val.Accept(writer);


    Designator reqDesig;
    reqDesig.fillFromJSON(std::string(strBuff.GetString()));
    reqDesig.printDesignator();
    designator_integration_msgs::DesignatorCommunication::Request reqMsg;
    designator_integration_msgs::DesignatorCommunication::Response respMsg;

    reqMsg.request.designator = reqDesig.serializeToMessage();
    designatorCallbackLogic(reqMsg, respMsg, false);

    for(auto resp : respMsg.response.designators)
    {
      Designator d(resp);
      d.setType(Designator::OBJECT);
      res.answer.push_back(d.serializeToJSON());
    }
    for(auto & r : res.answer)
    {
      outInfo("Before:" << r);
      rapidjson::Document d;
      d.Parse(r.c_str());
      std::string className;
      if(d.HasMember("class"))
      {
        rapidjson::Value &jClass = d["class"];
        className = jClass["name"].GetString();
      }

      std::string objectID = "http://knowrob.org/kb/thorin_simulation.owl#" + className + "1";
      std::string childFrameId = className + "1";

      if(std::find(seenObjects_.begin(), seenObjects_.end(), className) == seenObjects_.end())
      {
        outInfo("this is the first time I see this object. Get new ID");
        seenObjects_.push_back(className);
        outInfo(thorinObjects_[className]);

        json_prolog::Prolog pl;
        std::string query = "get_new_object_id('" + thorinObjects_[className] + "',OID)";
        outInfo("Asking query: " << query);
        json_prolog::PrologQueryProxy bdgs = pl.query(query);
        for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
        {
          outInfo("Got Object ID: " << (*it)["OID"].toString());
        }
      }

      if(d.HasMember("pose"))
      {
        outInfo("Foind pose in result string...looking for transform");
        rapidjson::Value &jPose = d["pose"];
        if(jPose.HasMember("transform"))
        {
          outInfo("Found the transform");
          rapidjson::Value &jTransf = jPose["transform"];
          if(std::strcmp(jTransf["child_frame_id"].GetString(), "") == 0)
          {
            outInfo("found emtpy child_frame_id. Overwriting");
            jTransf["child_frame_id"].SetString(childFrameId.c_str(), childFrameId.length());
          }
        }
      }
      if(d.HasMember("id"))
      {
        outInfo("found id in response. resolving through KR");
        d["id"].SetString(objectID.c_str(), objectID.length());
      }

      rapidjson::StringBuffer buffer;
      rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
      d.Accept(writer);
      r = buffer.GetString();
      outInfo("After: " << buffer.GetString());
    }
    //handle the json
    return true;
  }
  else if(doc.HasMember("inspect"))
  {
    const rapidjson::Value &val = doc["inspect"];
    std::vector<std::string> inspKeys;
    assert(val.IsObject());

    if(val.HasMember("type"))
    {
      outInfo("getting type: " << val["type"].GetString());
    }
    else
    {
      outError("Malformed inspection query. You need to specify which object you want to inspect using the type keypword!");
      return false;
    }

    if(val.HasMember("for"))
    {
      if(!val["for"].IsArray())
      {
        return false;
      }
      for(rapidjson::SizeType i = 0; i < val["for"].Size(); i++)
      {
        outInfo("       inspect for: " << val["for"][i].GetString());
        inspKeys.push_back(val["for"][i].GetString());
      }
    }

    std::string objToInspect = "";
    for(rapidjson::Value::ConstMemberIterator it = val.MemberBegin(); it != val.MemberEnd(); ++it)
    {
      if(std::strcmp(it->name.GetString(), "type") == 0)
      {
        objToInspect = it->value.GetString();
      }
    }

    std::string objToQueryFor = "";
    //replace this if-else uglyness with a json_prolog call
    if(objToInspect == "ChassisHolder")
    {
      objToQueryFor = "Chassis";
    }
    else if(objToInspect == "AxleHolder")
    {
      objToQueryFor = "Axle";
    }
    if(std::find(inspKeys.begin(), inspKeys.end(), "pose") != inspKeys.end())
    {
      objToQueryFor = objToInspect;
    }


    std_srvs::Trigger triggerSrv;
    if(inspectFromAR_)
    {
      if(triggerKRPoseUpdate_.call(triggerSrv))
      {
        outInfo("Called update KR from QR successfully");
      }
      else
      {
        outError("Is the node for updating object positions from AR markers still running? Calling qr_to_kr failed!");
        return false;
      }

      if(objToQueryFor != "")
      {

        json_prolog::Prolog pl;
        json_prolog::PrologQueryProxy bdgs = pl.query("owl_individual_of(I,'" + thorinObjects_[objToQueryFor] + "')");
        for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
        {
          res.answer.push_back(getObjectByID((*it)["I"].toString(), objToQueryFor).c_str());
        }
      }
      else
      {
        outError("Object to inspect is invalid!");
        return false;
      }
    }
    else
    {
      outWarn("******************************************");
      inspectionEngine_.process();
      outWarn("******************************************");
    }
    return true;
  }
  //old query stuff
  else if(req.query != "")
  {
    Designator reqDesig;
    reqDesig.fillFromJSON(std::string(req.query));
    designator_integration_msgs::DesignatorCommunication::Request reqMsg;
    designator_integration_msgs::DesignatorCommunication::Response respMsg;

    reqMsg.request.designator = reqDesig.serializeToMessage();
    designatorCallbackLogic(reqMsg, respMsg, false);

    for(auto resp : respMsg.response.designators)
    {
      Designator d(resp);
      d.setType(Designator::OBJECT);
      res.answer.push_back(d.serializeToJSON());
    }
    return true;
  }
  else
  {
    outInfo("False");
    return false;
  }

}

bool RSProcessManager::designatorSingleSolutionCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
    designator_integration_msgs::DesignatorCommunication::Response &res)
{
  return designatorCallbackLogic(req, res, false);
}


bool RSProcessManager::designatorCallbackLogic(designator_integration_msgs::DesignatorCommunication::Request &req,
    designator_integration_msgs::DesignatorCommunication::Response &res, bool allSolutions)
{
  if(rs::DesignatorWrapper::req_designator)
  {
    delete rs::DesignatorWrapper::req_designator;
  }
  rs::DesignatorWrapper::req_designator = new Designator(req.request.designator);
  rs::DesignatorWrapper::req_designator->setType(Designator::ACTION);
  rs::DesignatorWrapper::req_designator->printDesignator();

  semrec_client::Context *ctxRSEvent = NULL;
  if(semrecClient)
  {
    ctxRSEvent = new semrec_client::Context(this->semrecClient, "RoboSherlockEvent", "&rs_queryanswering;", "RoboSherlockEvent");
    ctxRSEvent->addDesignator(rs::DesignatorWrapper::req_designator, "rs_queryanswering:eventRequest", "&rs_queryanswering;", "eventRequest");
  }

  std::vector<Designator> filteredResponse;
  handleQuery(rs::DesignatorWrapper::req_designator, filteredResponse);

  std::vector<std::string> executedPipeline = engine_.getNextPipeline();

  // Define an ACTION designator with the planned pipeline
  Designator *pipeline_action = new Designator();
  pipeline_action->setType(Designator::ACTION);
  std::list<KeyValuePair *> lstDescription;
  for(auto & annotatorName : executedPipeline)
  {
    KeyValuePair *oneAnno = new KeyValuePair();
    oneAnno->setValue(annotatorName);
    lstDescription.push_back(oneAnno);
  }
  pipeline_action->setValue("pipeline-id", 0);
  pipeline_action->setValue("annotators", KeyValuePair::LIST, lstDescription);
  //  filteredResponse.push_back(pipeline_action);

  if(ctxRSEvent != NULL)
  {
    ctxRSEvent->addDesignator(pipeline_action, "rs_queryanswering:eventHandler", "&rs_queryanswering;", "eventHandler");
  }

  //   Delete the allocated keyvalue pairs for the annotator names

  designator_integration_msgs::DesignatorResponse topicResponse;
  for(auto & designator : filteredResponse)
  {
    res.response.designators.push_back(designator.serializeToMessage());
    if(ctxRSEvent)
    {
      //this needs to be an object->create copy constructor in Object class
      ctxRSEvent->addObject(new semrec_client::Object(&designator, "&rs_queryanswering;", "objectPerceived"), "rs_queryanswering:objectPerceived");
    }
  }
  if(ctxRSEvent != NULL)
  {
    ctxRSEvent->end();
  }
  delete ctxRSEvent;

  for(auto & kvpPtr : lstDescription)
  {
    delete kvpPtr;
  }
  delete pipeline_action;
  outWarn("RS Query service call ended");
  return true;
}

bool RSProcessManager::handleQuery(Designator *req, std::vector<Designator> &resp)
{
  RSQuery *query = new RSQuery();
  std::string superClass = "";
  //  rs::DesignatorWrapper::req_designator = req;
  //check Designator type...for some stupid reason req->type ==Designator::ACTION did not work
  processing_mutex_.lock();

  //these are hacks,, where we need the
  query->asJson = req->serializeToJSON();
  outInfo("Query as Json: " << query->asJson);
  if(req != NULL)
  {
    std::list<std::string> keys =  req->keys();
    for(auto key : keys)
    {
      if(key == "timestamp")
      {
        KeyValuePair *kvp = req->childForKey("timestamp");
        std::string ts = kvp->stringValue();
        query->timestamp = std::stoll(ts);
        outInfo("received timestamp:" << query->timestamp);
      }
      if(key == "location")
      {
        KeyValuePair *kvp1 = req->childForKey("location")->childForKey("on");
        KeyValuePair *kvp2 = req->childForKey("location")->childForKey("in");
        if(kvp1)
        {
          query->location = kvp1->stringValue();

          outInfo("received location:" << query->location);
        }
        else if(kvp2)
        {
          query->location = kvp2->stringValue();
          outInfo("received location:" << query->location);
        }
      }
      if(key == "obj-part" || key == "inspect")
      {
        KeyValuePair *kvp =  req->childForKey("obj-part");
        query->objToInspect = kvp->stringValue();
        outInfo("received obj-part request for object: " << query->objToInspect);
      }
      if(key == "ingredient")
      {
        KeyValuePair *kvp =  req->childForKey("ingredient");
        query->ingredient = kvp->stringValue();
        outInfo("received request for detection ingredient: " << query->ingredient);
      }
      if(key == "TYPE" || key == "type")
      {
        KeyValuePair *kvp =  req->childForKey("type");
        superClass = kvp->stringValue();
      }
    }
  }
  std::vector<std::string> keys;
  std::vector<std::string> new_pipeline_order;
  prologInterface->extractQueryKeysFromDesignator(req, keys);
  try
  {
    prologInterface->planPipelineQuery(keys, new_pipeline_order);
  }
  catch(std::exception e)
  {
    outError("calling json_prolog was not successfull. Is the node running?");
    processing_mutex_.unlock();
    return false;
  }

  if(new_pipeline_order.empty())
  {
    outInfo("Can't find solution for pipeline planning");
    processing_mutex_.unlock();
    return false; // Indicate failure
  }
  std::for_each(new_pipeline_order.begin(), new_pipeline_order.end(), [](std::string & p)
  {
    outInfo(p);
  });

  //always estimate a pose...these could go directly into the planning phase in Prolog?
  if(std::find(new_pipeline_order.begin(), new_pipeline_order.end(), "Cluster3DGeometryAnnotator") == new_pipeline_order.end())
  {
    std::vector<std::string>::iterator it = std::find(new_pipeline_order.begin(), new_pipeline_order.end(), "ClusterMerger");
    if(it != new_pipeline_order.end())
    {
      new_pipeline_order.insert(it + 1, "Cluster3DGeometryAnnotator");
    }
  }

  //for debugging advertise TF
  //  new_pipeline_order.push_back("TFBroadcaster");

  //whatever happens do ID res and spawn to gazebo...this is also pretty weird
  if(std::find(new_pipeline_order.begin(), new_pipeline_order.end(), "ObjectIdentityResolution") == new_pipeline_order.end())
  {
    new_pipeline_order.push_back("ObjectIdentityResolution");
    //    new_pipeline_order.push_back("GazeboInterface");
  }
  new_pipeline_order.push_back("StorageWriter");

  std::vector<Designator> resultDesignators;
  if(subsetOfLowLvl(new_pipeline_order))
  {
    outInfo("Query answerable by lowlvl pipeline. Executing it");
    engine_.process(resultDesignators, query);
  }
  else
  {
    outInfo(FG_BLUE << "Executing Pipeline # generated by query");
    engine_.process(new_pipeline_order, true, resultDesignators, query);
    outInfo("Executing pipeline generated by query: done");
  }

  outInfo("Returned " << resultDesignators.size() << " designators on this execution");
  filterResults(*req, resultDesignators, resp, superClass);
  outInfo("The filteredResponse size:" << resp.size());
  processing_mutex_.unlock();

  designator_integration_msgs::DesignatorResponse topicResponse;
  for(auto & designator : resp)
  {
    designator.printDesignator();
    topicResponse.designators.push_back(designator.serializeToMessage(false));
  }
  desig_pub_.publish(topicResponse);
  delete query;
  return true;

}

void RSProcessManager::filterResults(Designator &requestDesignator,
                                     std::vector<Designator> &resultDesignators,
                                     std::vector<Designator> &filteredResponse,
                                     std::string superclass)
{
  outInfo("filtering the results based on the designator request");
  outInfo("Superclass: " << superclass);
  std::vector<bool> keep_designator;
  keep_designator.resize(resultDesignators.size(), true);

  //    requestDesignator.printDesignator();
  std::list<KeyValuePair *> requested_kvps = requestDesignator.description();

  for(std::list<KeyValuePair *>::iterator it = requested_kvps.begin(); it != requested_kvps.end(); ++it)
  {
    KeyValuePair req_kvp = **it;
    if(req_kvp.key() == "timestamp" || req_kvp.key() == "location")
    {
      continue;
    }

    outInfo("No. of resulting Object Designators: " << resultDesignators.size());
    for(size_t i = 0; i < resultDesignators.size(); ++i)
    {
      Designator resDesig = resultDesignators[i];
      std::vector<KeyValuePair *> resultsForRequestedKey;
      KeyValuePair *childForRequestedKey = NULL;
      if(resDesig.childForKey("id") != NULL)
      {
        if(req_kvp.key() == "size") //size is nested get it from the bounding box..bad design
        {
          childForRequestedKey = resDesig.childForKey("boundingbox")->childForKey("size");
          resultsForRequestedKey.push_back(childForRequestedKey);
        }
        else if(req_kvp.key() == "cad-model")
        {
          childForRequestedKey = resDesig.childForKey("pose");
          resultsForRequestedKey.push_back(childForRequestedKey);
        }
        else if(req_kvp.key() == "volume")
        {
          childForRequestedKey = resDesig.childForKey("volume");
          if(childForRequestedKey != NULL)
          {
            resultsForRequestedKey.push_back(childForRequestedKey);
          }
          else
          {
            keep_designator[i] = false;
          }
        }
        else if(req_kvp.key() == "contains")
        {
          childForRequestedKey = resDesig.childForKey("contains");
          if(childForRequestedKey != NULL)
          {
            resultsForRequestedKey.push_back(childForRequestedKey);
          }
          else
          {
            keep_designator[i] = false;
          }
        }
        else if(req_kvp.key() == "shape") //there can be multiple shapes and these are not nested
        {
          std::list<KeyValuePair *> resKvPs = resDesig.description();
          for(std::list<KeyValuePair *>::iterator it2 = resKvPs.begin(); it2 != resKvPs.end(); ++it2)
          {
            KeyValuePair res_kvp = **it2;
            if(res_kvp.key() == "shape")
            {
              resultsForRequestedKey.push_back(*it2);
            }
          }
        }
        else if(req_kvp.key() == "type")//this shit needed so we don't loose al of our stuff just because all was sent instead of detection
        {
          resultsForRequestedKey.push_back(resDesig.childForKey("class"));
        }
        else if(req_kvp.key() == "ingredient")
        {
          resultsForRequestedKey.push_back(resDesig.childForKey("pizza"));
        }
        else
        {
          resultsForRequestedKey.push_back(resDesig.childForKey(req_kvp.key()));
        }
      }
      else
      {
        resultsForRequestedKey.push_back(resDesig.childForKey(req_kvp.key()));
        outWarn("No CLUSTER ID");
      }

      if(!resultsForRequestedKey.empty())
      {
        bool ok = false;
        for(int j = 0; j < resultsForRequestedKey.size(); ++j)
        {
          if(resultsForRequestedKey[j] != NULL)
          {
            if(resultsForRequestedKey[j]->key() == "pose")
            {
              std::list<KeyValuePair * > kvps_ = resultDesignators[i].description();
              std::list<KeyValuePair * >::iterator it = kvps_.begin();
              bool hasCadPose = false;
              while(it != kvps_.end())
              {
                if((*it)->key() == "pose")
                {
                  if((*it)->childForKey("source")->stringValue() == "TemplateAlignment")
                  {
                    hasCadPose = true;
                    ++it;
                  }
                  else
                  {
                    kvps_.erase(it++);
                  }
                }
                else
                {
                  ++it;
                }
              }
              ok = hasCadPose;
              resultDesignators[i].setDescription(kvps_);
            }
            if(resultsForRequestedKey[j]->key() == "obj-part")
            {
              ok = true;
              //              std::list<KeyValuePair * >::iterator it = kvps_.begin();
              //              while(it != kvps_.end())
              //              {
              //                if((*it)->key() != "OBJ-PART" && (*it)->key() != "ID" && (*it)->key() != "TIMESTAMP")
              //                  kvps_.erase(it++);
              //                else
              //                {
              //                  if((*it)->key() == "OBJ-PART")
              //                  {
              //                    if((strcasecmp((*it)->childForKey("NAME")->stringValue().c_str(), req_kvp.stringValue().c_str()) == 0) || (req_kvp.stringValue() == ""))
              //                      ++it;
              //                    else
              //                      kvps_.erase(it++);
              //                  }
              //                  else
              //                    ++it;
              //                }
              //              }
              //              resultDesignators[i].setDescription(kvps_);
            }
            if(resultsForRequestedKey[j]->key() == "pizza")
            {
              ok = true;
              std::list<KeyValuePair * > kvps_ = resultDesignators[i].description();
              std::list<KeyValuePair * >::iterator it = kvps_.begin();
              while(it != kvps_.end())
              {
                if((*it)->key() != "pizza" && (*it)->key() != "id" && (*it)->key() != "timestamp")
                {
                  kvps_.erase(it++);
                }
                else
                {
                  ++it;
                }

              }
              resultDesignators[i].setDescription(kvps_);
              resultDesignators[i].printDesignator();
            }
            //treat color differently because it is nested and has every color with ration in there
            else if(resultsForRequestedKey[j]->key() == "color")
            {
              std::list<KeyValuePair *> colorRatioPairs = resultsForRequestedKey[j]->children();
              for(auto iter = colorRatioPairs.begin(); iter != colorRatioPairs.end(); ++iter)
              {
                KeyValuePair colorRatioKvp = **iter;
                if(strcasecmp(colorRatioKvp.key().c_str(), req_kvp.stringValue().c_str()) == 0 || req_kvp.stringValue() == "")
                {
                  if(colorRatioKvp.floatValue() > 0.20)
                  {
                    ok = true;
                  }
                }
              }
            }
            else if(resultsForRequestedKey[j]->key() == "volume")
            {
              float volumeofCurrentObj = resultsForRequestedKey[j]->floatValue();
              outWarn("Volume as a float: " << volumeofCurrentObj);
              if(req_kvp.stringValue() == "")
              {
                ok = true;
              }
              else
              {
                float volumeAsked = atof(req_kvp.stringValue().c_str());
                outWarn("Volume asked as float: " << volumeAsked);
                if(volumeAsked <= volumeofCurrentObj)
                {
                  ok = true;
                }
              }
            }
            else if(resultsForRequestedKey[j]->key() == "contains")
            {
              if(req_kvp.stringValue() == "")
              {
                ok = true;
              }
              else
              {
                std::string substanceName = resultsForRequestedKey[j]->childForKey("substance")->stringValue();
                std::string substanceAsked = req_kvp.stringValue();
                outWarn("Substance asked : " << substanceAsked);
                if(strcasecmp(substanceName.c_str(), substanceAsked.c_str()) == 0)
                {
                  ok = true;
                }
              }
            }

            //another nested kv-p...we need a new interface...this one sux
            if(resultsForRequestedKey[j]->key() == "class")
            {
              std::list<KeyValuePair *> childrenPairs = resultsForRequestedKey[j]->children();
              for(auto iter = childrenPairs.begin(); iter != childrenPairs.end(); ++iter)
              {
                KeyValuePair childrenPair = **iter;
                if(childrenPair.key() == "name")
                {
                  if(superclass != "" && rs_queryanswering::krNameMapping.count(superclass) == 1)
                  {
                    try
                    {
                      ok = prologInterface->q_subClassOf(childrenPair.stringValue(), superclass);
                    }
                    catch(std::exception &e)
                    {
                      outError("Prolog Exception: Malformed owl_subclass of. Child or superclass undefined:");
                      outError("     Child: " << childrenPair.stringValue());
                      outError("     Parent: " << superclass);
                    }
                  }
                  else if(strcasecmp(childrenPair.stringValue().c_str(), req_kvp.stringValue().c_str()) == 0 || req_kvp.stringValue() == "")
                  {
                    ok = true;
                  }
                  else if(childrenPair.stringValue() == "bottle_acid" || childrenPair.stringValue() == "bottle_base")
                  {
                    std::string new_name = "bottle";
                    if(strcasecmp(new_name.c_str(), req_kvp.stringValue().c_str()) == 0)
                    {
                      ok = true;
                    }
                  }
                }
              }
            }
            else if(strcasecmp(resultsForRequestedKey[j]->stringValue().c_str(),
                               req_kvp.stringValue().c_str()) == 0 || req_kvp.stringValue() == "")
            {
              ok = true;
            }
          }
        }
        if(!ok)
        {
          keep_designator[i] = false;
        }
      }
    }
  }

  for(int i = 0; i < keep_designator.size(); ++i)
  {
    if(keep_designator[i])
    {
      filteredResponse.push_back(resultDesignators[i]);
    }
  }

  if(useIdentityResolution_)
  {
    engine_.drawResulstOnImage<rs::Object>(keep_designator, resultDesignators, requestDesignator);
  }
  else
  {
    engine_.drawResulstOnImage<rs::Cluster>(keep_designator, resultDesignators, requestDesignator);
  }
}

bool RSProcessManager::subsetOfLowLvl(const std::vector<std::string> &plannedPipeline)
{
  std::vector<std::string> intersection;
  std::set_intersection(lowLvlPipeline_.begin(), lowLvlPipeline_.end(), plannedPipeline.begin(), plannedPipeline.end(),  std::back_inserter(intersection));
  return intersection.size() == plannedPipeline.size() ? true : false;
}
