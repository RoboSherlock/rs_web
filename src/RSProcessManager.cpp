#include <rs_kbreasoning/RSProcessManager.h>

using namespace designator_integration;

void RSProcessManager::run()
{
  for(; ros::ok();)
  {
    if(waitForServiceCall_)
    {
      usleep(100000);
    }
    else
    {
      processing_mutex_.lock();
      engine.process(true);
      processing_mutex_.unlock();
    }
    ros::spinOnce();
  }
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
    cv::FileStorage fs(configFile, cv::FileStorage::READ);
    std::vector<std::string> lowLvlPipeline;
    fs["annotators"] >> lowLvlPipeline;

    processing_mutex_.lock();
    this->init(contextAEPath, configFile);
    processing_mutex_.unlock();

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
  if(doc.HasMember("semrec"))
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
        semrecClient->registerOWLNamespace("rs_kbreasoning", "http://robosherlock.org/#");
      }
      else
      {
        outError("Logging allready started");
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
      }
      else
      {
        outError("There is no logging started");
      }
    }
    else
    {
      outError("Undefined command!");
    }
  }
  else
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
  }
  return true;
}

bool RSProcessManager::designatorAllSolutionsCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
    designator_integration_msgs::DesignatorCommunication::Response &res)
{
  return designatorCallbackLogic(req, res, true);
}

bool RSProcessManager::designatorSingleSolutionCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
    designator_integration_msgs::DesignatorCommunication::Response &res)
{
  return designatorCallbackLogic(req, res, false);
}

bool RSProcessManager::handleQuery(Designator* req, std::vector<Designator> &resp)
{
  RSQuery *query = new RSQuery();
  std::string superClass = "";
//  rs::DesignatorWrapper::req_designator = req;
  //check Designator type...for some stupid reason req->type ==Designator::ACTION did not work


  //these are hacks,, where we need the
  query->asJson = req->serializeToJSON();
  if(req != NULL)
  {
    std::list<std::string> keys =  req->keys();
    for(auto key : keys)
    {
      if(key == "TIMESTAMP")
      {
        KeyValuePair *kvp = req->childForKey("TIMESTAMP");
        std::string ts = kvp->stringValue();
        query->timestamp = std::stoll(ts);
        outInfo("received timestamp:" << query->timestamp);
      }
      if(key == "LOCATION")
      {
        KeyValuePair *kvp = req->childForKey("LOCATION");
        query->location = kvp->stringValue();
        outInfo("received location:" << query->location);
      }
      if(key == "OBJ-PART" || key == "INSPECT")
      {
        KeyValuePair *kvp =  req->childForKey("OBJ-PART");
        query->objToInspect = kvp->stringValue();
        outInfo("received obj-part request for object: " << query->objToInspect);
      }
      if(key == "INGREDIENT")
      {
        KeyValuePair *kvp =  req->childForKey("INGREDIENT");
        query->ingredient = kvp->stringValue();
        outInfo("received request for detection ingredient: " << query->ingredient);
      }
      if(key == "TYPE")
      {
        KeyValuePair *kvp =  req->childForKey("TYPE");
        superClass = kvp->stringValue();
      }
    }
  }

  std::string prologQuery = "";

  if(!jsonPrologInterface_.buildPrologQueryFromDesignator(req, prologQuery))
  {
    outInfo("Aborting Prolog Query... The generated Prolog Command is invalid");
    return false;
  }
  outInfo("Query Prolog with the following command: " << prologQuery);

  json_prolog::Prolog pl;
  json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery);

  if(bdgs.begin() == bdgs.end())
  {
    outInfo("Can't find solution for pipeline planning");
    return false; // Indicate failure
  }

  processing_mutex_.lock();
  outInfo(FG_CYAN << "LOCK ACQUIRED");

  std::vector<Designator> resultDesignators;
  std::vector<std::string> new_pipeline_order;
  for(auto bdg : bdgs)
  {
    new_pipeline_order = jsonPrologInterface_.createPipelineFromPrologResult(bdg["A"].toString());

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
    new_pipeline_order.push_back("TFBroadcaster");

    //whatever happens do ID res and spawn to gazebo...this is also pretty weird
    if(std::find(new_pipeline_order.begin(), new_pipeline_order.end(), "ObjectIdentityResolution") == new_pipeline_order.end())
    {
      new_pipeline_order.push_back("ObjectIdentityResolution");
      new_pipeline_order.push_back("GazeboInterface");
    }
    new_pipeline_order.push_back("StorageWriter");

    if(/*!allSolutions*/true)
    {
      break;  // Only take the first solution if allSolutions == false
    }
  }

  if(subsetOfLowLvl(new_pipeline_order))
  {
    outInfo("Query answerable by lowlvl pipeline. Executing it");
    engine.process(resultDesignators, query);
  }
  else
  {
    outInfo(FG_BLUE << "Executing Pipeline # generated by query");
    engine.process(new_pipeline_order, true, resultDesignators, query);
    outInfo("Executing pipeline generated by query: done");
  }

  outInfo("Returned " << resultDesignators.size() << " designators on this execution");
  filterResults(*rs::DesignatorWrapper::req_designator, resultDesignators, resp, superClass);
  outInfo("The filteredResponse size:" << resp.size());
  processing_mutex_.unlock();
  delete query;
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

  //log the req desig with semrec
  semrec_client::Context *ctxRSEvent = NULL;
  if(semrecClient)
  {
    ctxRSEvent = new semrec_client::Context(this->semrecClient, "RoboSherlockEvent", "&rs_kbreasoning;", "RoboSherlockEvent");
    //    ctxRSEvent->addObject(new semrec_client::Object(rs::DesignatorWrapper::req_designator,"&rs_kbreasoning;","eventRequest"),"rs_kbreasoning:eventRequest");
    ctxRSEvent->addDesignator(rs::DesignatorWrapper::req_designator, "rs_kbreasoning:eventRequest", "&rs_kbreasoning;", "eventRequest");
    //    eveReq->end();
    //    delete eveReq;
  }


  std::vector<Designator> filteredResponse;
  handleQuery(rs::DesignatorWrapper::req_designator, filteredResponse);

  std::vector<std::string> executedPipeline = engine.getNextPipeline();

  for(auto & designator : filteredResponse)
  {
    designator.setValue("PIPELINEID", 0);
  }

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
  pipeline_action->setValue("PIPELINEID", 0);
  pipeline_action->setValue("ANNOTATORS", KeyValuePair::LIST, lstDescription);
  //  filteredResponse.push_back(pipeline_action);

  if(ctxRSEvent != NULL)
  {
    ctxRSEvent->addDesignator(pipeline_action, "rs_kbreasoning:eventHandler", "&rs_kbreasoning;", "eventHandler");
  }

  //   Delete the allocated keyvalue pairs for the annotator names

  designator_integration_msgs::DesignatorResponse topicResponse;
  for(auto & designator : filteredResponse)
  {
    designator.printDesignator();
    res.response.designators.push_back(designator.serializeToMessage());
    topicResponse.designators.push_back(designator.serializeToMessage(false));
    if(ctxRSEvent)
    {
      //this needs to be an object->create copy constructor in Object class
      ctxRSEvent->addObject(new semrec_client::Object(&designator, "&rs_kbreasoning;", "objectPerceived"), "rs_kbreasoning:objectPerceived");
    }
  }

  if(ctxRSEvent != NULL)
  {
    ctxRSEvent->end();
  }
  delete ctxRSEvent;

  desig_pub_.publish(topicResponse);
  outInfo(FG_CYAN << "LOCK RELEASE");
  for(auto & kvpPtr : lstDescription)
  {
    delete kvpPtr;
  }
  delete pipeline_action;
  outWarn("RS Query service call ended");
  return true;
}


void RSProcessManager::filterResults(Designator &requestDesignator,
                                     std::vector<Designator> &resultDesignators,
                                     std::vector<Designator> &filteredResponse,
                                     std::string superclass)
{
  outInfo("filtering the results based on the designator request");

  std::vector<bool> keep_designator;
  keep_designator.resize(resultDesignators.size(), true);

  //    requestDesignator.printDesignator();
  std::list<KeyValuePair *> requested_kvps = requestDesignator.description();

  for(std::list<KeyValuePair *>::iterator it = requested_kvps.begin(); it != requested_kvps.end(); ++it)
  {
    KeyValuePair req_kvp = **it;
    if(req_kvp.key() == "TIMESTAMP" || req_kvp.key() == "LOCATION")
    {
      continue;
    }

    outInfo("No. of resulting Object Designators: " << resultDesignators.size());
    for(size_t i = 0; i < resultDesignators.size(); ++i)
    {
      Designator resDesig = resultDesignators[i];
      std::vector<KeyValuePair *> resultsForRequestedKey;
      KeyValuePair *childForRequestedKey = NULL;
      if(resDesig.childForKey("ID") != NULL)
      {
        if(req_kvp.key() == "SIZE") //size is nested get it from the bounding box..bad design
        {
          childForRequestedKey = resDesig.childForKey("BOUNDINGBOX")->childForKey("SIZE");
          resultsForRequestedKey.push_back(childForRequestedKey);
        }
        else if(req_kvp.key() == "CAD-MODEL")
        {
          childForRequestedKey = resDesig.childForKey("POSE");
          resultsForRequestedKey.push_back(childForRequestedKey);
        }
        else if(req_kvp.key() == "VOLUME")
        {
          childForRequestedKey = resDesig.childForKey("VOLUME");
          if(childForRequestedKey != NULL)
          {
            resultsForRequestedKey.push_back(childForRequestedKey);
          }
          else
          {
            keep_designator[i] = false;
          }
        }
        else if(req_kvp.key() == "CONTAINS")
        {
          childForRequestedKey = resDesig.childForKey("CONTAINS");
          if(childForRequestedKey != NULL)
          {
            resultsForRequestedKey.push_back(childForRequestedKey);
          }
          else
          {
            keep_designator[i] = false;
          }
        }
        else if(req_kvp.key() == "SHAPE") //there can be multiple shapes and these are not nested
        {
          std::list<KeyValuePair *> resKvPs = resDesig.description();
          for(std::list<KeyValuePair *>::iterator it2 = resKvPs.begin(); it2 != resKvPs.end(); ++it2)
          {
            KeyValuePair res_kvp = **it2;
            if(res_kvp.key() == "SHAPE")
            {
              resultsForRequestedKey.push_back(*it2);
            }
          }
        }
        else if(req_kvp.key() == "TYPE")//this shit needed so we don't loose al of our stuff just because all was sent instead of detection
        {
          resultsForRequestedKey.push_back(resDesig.childForKey("DETECTION"));
        }
        else if(req_kvp.key() == "INGREDIENT")
        {
          resultsForRequestedKey.push_back(resDesig.childForKey("PIZZA"));
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
            if(resultsForRequestedKey[j]->key() == "POSE")
            {
              std::list<KeyValuePair * > kvps_ = resultDesignators[i].description();
              std::list<KeyValuePair * >::iterator it = kvps_.begin();
              bool hasCadPose = false;
              while(it != kvps_.end())
              {
                if((*it)->key() == "POSE")
                {
                  if((*it)->childForKey("SOURCE")->stringValue() == "TemplateAlignment")
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
            if(resultsForRequestedKey[j]->key() == "OBJ-PART")
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
            if(resultsForRequestedKey[j]->key() == "PIZZA")
            {
              ok = true;
              std::list<KeyValuePair * > kvps_ = resultDesignators[i].description();
              std::list<KeyValuePair * >::iterator it = kvps_.begin();
              while(it != kvps_.end())
              {
                if((*it)->key() != "PIZZA" && (*it)->key() != "ID" && (*it)->key() != "TIMESTAMP")
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
            else if(resultsForRequestedKey[j]->key() == "COLOR")
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
            else if(resultsForRequestedKey[j]->key() == "VOLUME")
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
            else if(resultsForRequestedKey[j]->key() == "CONTAINS")
            {
              if(req_kvp.stringValue() == "")
              {
                ok = true;
              }
              else
              {
                std::string substanceName = resultsForRequestedKey[j]->childForKey("SUBSTANCE")->stringValue();
                std::string substanceAsked = req_kvp.stringValue();
                outWarn("Substance asked : " << substanceAsked);
                if(strcasecmp(substanceName.c_str(), substanceAsked.c_str()) == 0)
                {
                  ok = true;
                }
              }
            }

            //another nested kv-p...we need a new interface...this one sux
            if(resultsForRequestedKey[j]->key() == "DETECTION")
            {
              std::list<KeyValuePair *> childrenPairs = resultsForRequestedKey[j]->children();
              for(auto iter = childrenPairs.begin(); iter != childrenPairs.end(); ++iter)
              {
                KeyValuePair childrenPair = **iter;
                if(childrenPair.key() == "CLASS")
                {
                  if(superclass != "" && rs_kbreasoning::krNameMapping.count(superclass) == 1)
                  {
                    std::stringstream prologQuery;
                    prologQuery << "owl_subclass_of(" << rs_kbreasoning::krNameMapping[childrenPair.stringValue()] << "," << rs_kbreasoning::krNameMapping[superclass] << ").";
                    outInfo("Asking Query: " << prologQuery.str());
                    json_prolog::Prolog pl;
                    json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
                    bdgs.begin() == bdgs.end() ? ok = false : ok = true;
                    if(ok)
                    {
                      outInfo(rs_kbreasoning::krNameMapping[childrenPair.stringValue()] << " IS " << rs_kbreasoning::krNameMapping[superclass]);
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
      outInfo("Designator: " << i << " is a match");
      filteredResponse.push_back(resultDesignators[i]);
    }
  }

  if(useIdentityResolution_)
  {
    engine.drawResulstOnImage<rs::Object>(keep_designator, resultDesignators, requestDesignator);
  }
  else
  {
    engine.drawResulstOnImage<rs::Cluster>(keep_designator, resultDesignators, requestDesignator);
  }
}

bool RSProcessManager::subsetOfLowLvl(const std::vector<std::string> &plannedPipeline)
{
  std::vector<std::string> intersection;
  std::set_intersection(lowLvlPipeline_.begin(), lowLvlPipeline_.end(), plannedPipeline.begin(), plannedPipeline.end(),  std::back_inserter(intersection));
  return intersection.size() == plannedPipeline.size() ? true : false;
}
