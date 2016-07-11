#include <rs_kbreasoning/RSControledAEManager.h>

void RSControledAEManager::run()
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

bool RSControledAEManager::resetAECallback(iai_robosherlock_msgs::SetRSContext::Request &req,
    iai_robosherlock_msgs::SetRSContext::Response &res)
{
  outInfo("acquiring lock");
  processing_mutex_.lock();
  outInfo(FG_CYAN << "LOCK ACQUIRED");
  std::string newContextName = req.newAe, contextAEPath;

  if(rs::common::getAEPaths(newContextName, contextAEPath))
  {
    outInfo("Setting new context: " << newContextName);
    this->init(contextAEPath);
    outInfo("releasing lock");
    processing_mutex_.unlock();
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

bool RSControledAEManager::jsonQueryCallback(iai_robosherlock_msgs::RSQueryService::Request &req,
    iai_robosherlock_msgs::RSQueryService::Response &res)
{
  designator_integration::Designator reqDesig;
  reqDesig.fillFromJSON(std::string(req.query));
  designator_integration_msgs::DesignatorCommunication::Request reqMsg;
  designator_integration_msgs::DesignatorCommunication::Response respMsg;
  reqMsg.request.designator = reqDesig.serializeToMessage();
  designatorCallbackLogic(reqMsg, respMsg, false);
  for (auto resp:respMsg.response.designators)
  {
    designator_integration::Designator d(resp);
    d.setType(designator_integration::Designator::OBJECT);
    res.answer.push_back(d.serializeToJSON());
  }
  return true;
}

bool RSControledAEManager::designatorAllSolutionsCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
    designator_integration_msgs::DesignatorCommunication::Response &res)
{
  return designatorCallbackLogic(req, res, true);
}

bool RSControledAEManager::designatorSingleSolutionCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
    designator_integration_msgs::DesignatorCommunication::Response &res)
{
  return designatorCallbackLogic(req, res, false);
}

bool RSControledAEManager::designatorCallbackLogic(designator_integration_msgs::DesignatorCommunication::Request &req,
    designator_integration_msgs::DesignatorCommunication::Response &res, bool allSolutions)
{
  if(rs::DesignatorWrapper::req_designator)
  {
    delete rs::DesignatorWrapper::req_designator;
  }
  rs::DesignatorWrapper::req_designator = new designator_integration::Designator(req.request.designator);
  outInfo("Type of request designator: "<< rs::DesignatorWrapper::req_designator->type());
  //this is weird, output before and after setting type is the same, but serializeToJson only works if I set it here explicitly
  rs::DesignatorWrapper::req_designator->setType(designator_integration::Designator::OBJECT);
  outInfo("Type of request designator: "<< rs::DesignatorWrapper::req_designator->type());
  outInfo("Received Designator call: " << rs::DesignatorWrapper::req_designator->serializeToJSON());
  rs::DesignatorWrapper::req_designator->printDesignator();


  RSQuery *query = new RSQuery();
  std::string superClass = "";

  if(rs::DesignatorWrapper::req_designator->type() != designator_integration::Designator::OBJECT)
  {
    outInfo(" ***** RECEIVED SERVICE CALL WITH UNHANDELED DESIGNATOR TYPE (everything != OBJECT) ! Aborting... ****** ");
    return false;
  }

  //these are hacks,, where we need the
  if(rs::DesignatorWrapper::req_designator != NULL)
  {
    std::list<std::string> keys =  rs::DesignatorWrapper::req_designator->keys();
    for(auto key : keys)
    {
      if(key == "TIMESTAMP")
      {
        designator_integration::KeyValuePair *kvp = rs::DesignatorWrapper::req_designator->childForKey("TIMESTAMP");
        std::string ts = kvp->stringValue();
        query->timestamp = std::stoll(ts);
        outInfo("received timestamp:" << query->timestamp);
      }
      if(key == "LOCATION")
      {
        designator_integration::KeyValuePair *kvp = rs::DesignatorWrapper::req_designator->childForKey("LOCATION");
        query->location = kvp->stringValue();
        outInfo("received location:" << query->location);
      }
      if(key == "OBJ-PARTS")
      {
        designator_integration::KeyValuePair *kvp =  rs::DesignatorWrapper::req_designator->childForKey("OBJ-PARTS");
        query->objToInspect = kvp->stringValue();
        outInfo("received obj-part request for object: " << query->objToInspect);
      }
      if(key == "IGREDIENT")
      {
        designator_integration::KeyValuePair *kvp =  rs::DesignatorWrapper::req_designator->childForKey("INGREDIENT");
        query->ingredient = kvp->stringValue();
        outInfo("received request for detection ingredient: " << query->objToInspect);
      }
      if(key == "TYPE")
      {
        designator_integration::KeyValuePair *kvp =  rs::DesignatorWrapper::req_designator->childForKey("TYPE");
        superClass = kvp->stringValue();
      }
    }
  }

  std::string prologQuery = "";

  if(!jsonPrologInterface_.buildPrologQueryFromDesignator(rs::DesignatorWrapper::req_designator, prologQuery))
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

  int pipelineId = 0;

  processing_mutex_.lock();
  outInfo(FG_CYAN << "LOCK ACQUIRED");

  std::vector<designator_integration::Designator> resultDesignators;
  for(auto bdg : bdgs)
  {
    std::string prologResult = bdg["A"].toString();
    std::vector<std::string> new_pipeline_order = jsonPrologInterface_.createPipelineFromPrologResult(bdg["A"].toString());

    outInfo(FG_BLUE << "Executing Pipeline # " << pipelineId << " generated by service call");
    engine.process(new_pipeline_order, true, resultDesignators, query);
    outInfo("Returned " << resultDesignators.size() << " designators on this execution");


    //Add the PIPELINEID to reference the pipeline that was responsible for detecting the returned object
    //    for(auto & designator : resultDesignators)
    //    {
    //      designator.setValue("PIPELINEID", pipelineId);
    //    }

    // Define an ACTION designator with the planned pipeline
    //    designator_integration::Designator pipeline_action;
    //    pipeline_action.setType(designator_integration::Designator::ACTION);
    //    std::list<designator_integration::KeyValuePair *> lstDescription;
    //    for(auto & annotatorName : new_pipeline_order)
    //    {
    //      designator_integration::KeyValuePair *oneAnno = new designator_integration::KeyValuePair();
    //      oneAnno->setValue(annotatorName);
    //      lstDescription.push_back(oneAnno);
    //    }
    //    pipeline_action.setValue("PIPELINEID", pipelineId);
    //    pipeline_action.setValue("ANNOTATORS", designator_integration::KeyValuePair::LIST, lstDescription);
    //    resultDesignators.push_back(pipeline_action);

    // Delete the allocated keyvalue pairs for the annotator names
    //    for(auto & kvpPtr : lstDescription)
    //    {
    //      delete kvpPtr;
    //    }
    outInfo("Executing pipeline generated by service call: done");

    if(!allSolutions)
    {
      break;  // Only take the first solution if allSolutions == false
    }
    pipelineId++;
  }

  std::vector<designator_integration::Designator> filteredResponse;
  filterResults(*rs::DesignatorWrapper::req_designator, resultDesignators, filteredResponse, superClass);
  outInfo("The filteredResponse size:" << filteredResponse.size());

  designator_integration_msgs::DesignatorResponse topicResponse;
  for(auto & designator : filteredResponse)
  {
    outInfo(designator.serializeToJSON());
    designator.printDesignator();
    topicResponse.designators.push_back(designator.serializeToMessage());
    res.response.designators.push_back(designator.serializeToMessage());
  }
  desig_pub_.publish(topicResponse);
  processing_mutex_.unlock();
  outInfo(FG_CYAN << "LOCK RELEASE");
  delete query;
  outWarn("RS Query service call ended");
  return true;
}


void RSControledAEManager::filterResults(designator_integration::Designator &requestDesignator,
    const std::vector<designator_integration::Designator> &resultDesignators,
    std::vector<designator_integration::Designator> &filteredResponse,
    std::string superclass)
{
  outInfo("filtering the results based on the designator request");

  std::vector<bool> keep_designator;
  keep_designator.resize(resultDesignators.size(), true);

  //    requestDesignator.printDesignator();
  std::list<designator_integration::KeyValuePair *> requested_kvps = requestDesignator.description();

  for(std::list<designator_integration::KeyValuePair *>::iterator it = requested_kvps.begin(); it != requested_kvps.end(); ++it)
  {
    designator_integration::KeyValuePair req_kvp = **it;
    if(req_kvp.key() == "TIMESTAMP" || req_kvp.key() == "LOCATION")
    {
      continue;
    }
    if(req_kvp.key() == "HANDLE")
    {
      outInfo("Handle requested, nothing to do here");
      continue;
    }

    outInfo("No. of result Designators: " << resultDesignators.size());
    for(size_t i = 0; i < resultDesignators.size(); ++i)
    {
      designator_integration::Designator resDesig = resultDesignators[i];
      //        resDesig.printDesignator();
      //and here come the hacks
      std::vector<designator_integration::KeyValuePair *> resultsForRequestedKey;
      designator_integration::KeyValuePair *childForRequestedKey = NULL;
      if(resDesig.childForKey("CLUSTERID") != NULL)
      {
        if(req_kvp.key() == "SIZE") //size is nested get it from the bounding box..bad design
        {
          childForRequestedKey = resDesig.childForKey("BOUNDINGBOX")->childForKey("SIZE");
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
          std::list<designator_integration::KeyValuePair *> resKvPs = resDesig.description();
          for(std::list<designator_integration::KeyValuePair *>::iterator it2 = resKvPs.begin(); it2 != resKvPs.end(); ++it2)
          {
            designator_integration::KeyValuePair res_kvp = **it2;
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
        else
        {
          resultsForRequestedKey.push_back(resDesig.childForKey(req_kvp.key()));
        }
      }
      else
      {
        outWarn("No CLUSTER ID");
      }

      if(!resultsForRequestedKey.empty())
      {
        bool ok = false;
        for(int j = 0; j < resultsForRequestedKey.size(); ++j)
        {
          if(resultsForRequestedKey[j] != NULL)
          {
            //I am doing this wrong...why do I want to look at this shit again?
            //treat color differently because it is nested and has every color with ration in there
            if(resultsForRequestedKey[j]->key() == "COLOR")
            {
              std::list<designator_integration::KeyValuePair *> colorRatioPairs = resultsForRequestedKey[j]->children();
              for(auto iter = colorRatioPairs.begin(); iter != colorRatioPairs.end(); ++iter)
              {
                designator_integration::KeyValuePair colorRatioKvp = **iter;
                if(strcasecmp(colorRatioKvp.key().c_str(), req_kvp.stringValue().c_str()) == 0 || req_kvp.stringValue() == "")
                {
                  outInfo("Color name mathces, ratio is: " << colorRatioKvp.floatValue());
                  if(colorRatioKvp.floatValue() > 0.20)
                  {
                    ok = true;
                  }
                }
              }
            }
            if(resultsForRequestedKey[j]->key() == "VOLUME")
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
            if(resultsForRequestedKey[j]->key() == "CONTAINS")
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
              std::list<designator_integration::KeyValuePair *> childrenPairs = resultsForRequestedKey[j]->children();
              for(auto iter = childrenPairs.begin(); iter != childrenPairs.end(); ++iter)
              {
                designator_integration::KeyValuePair childrenPair = **iter;
                if(childrenPair.key() == "TYPE")
                {
                  if(superclass != "")
                  {
                    std::stringstream prologQuery;
                    prologQuery << "owl_subclass_of(" << rs_kbreasoning::krNameMapping[childrenPair.stringValue()] << "," << rs_kbreasoning::krNameMapping[superclass] << ").";
                    json_prolog::Prolog pl;
                    json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
                    if(bdgs.begin() == bdgs.end())
                    {
                      ok = false;
                    }
                    else
                    {
                      outInfo(rs_kbreasoning::krNameMapping[childrenPair.stringValue()] << " IS " << rs_kbreasoning::krNameMapping[superclass]);
                      ok = true;
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
  engine.drawResulstOnImage(keep_designator, resultDesignators);
}
