#include<rs_queryanswering/QueryInterface.h>
#include<rs_queryanswering/DesignatorWrapper.h>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

//RapidJson
#include "rapidjson/pointer.h"

bool QueryInterface::parseQuery(std::string query)
{
  this->query.Parse(query);
  return true;
}

QueryInterface::QueryType QueryInterface::processQuery(std::vector<std::string> &res)
{
  if(query.HasMember("detect"))
  {
    handleDetect(res);
    return DETECT;
  }

  else if(query.HasMember("inspect"))
  {
    handleInspect(res);
    return INSPECT;
  }
  return NONE;
}

bool QueryInterface::handleInspect(std::vector<std::string> &res)
{

  const rapidjson::Value &val = query["inspect"];
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

  json_prolog::Prolog pl;
  std::string plQueryString = "class_properties('" + thorinObjects_[objToInspect] +
                              +"','http://knowrob.org/kb/knowrob_assembly.owl#canHoldObject', B)";
  outInfo("asking query: " << plQueryString);
  json_prolog::PrologQueryProxy bdgs = pl.query(plQueryString);
  for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
  {
    objToQueryFor = (*it)["B"].toString();
    outInfo("got a result: " << objToQueryFor);
  }

  if(std::find(inspKeys.begin(), inspKeys.end(), "pose") != inspKeys.end())
  {
    objToQueryFor = objToInspect;
  }


  std_srvs::Trigger triggerSrv;
  //TODO is this needed?
  //    if(inspectFromAR_)
  //    {
  //      if(triggerKRPoseUpdate_.call(triggerSrv))
  //      {
  //        outInfo("Called update KR from QR successfully");
  //      }
  //      else
  //      {
  //        outError("Is the node for updating object positions from AR markers still running? Calling qr_to_kr failed!");
  //        return false;
  //      }

  //      if(objToQueryFor != "")
  //      {

  //        json_prolog::Prolog pl;
  //        json_prolog::PrologQueryProxy bdgs = pl.query("owl_individual_of(I,'" + thorinObjects_[objToQueryFor] + "')");
  //        for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
  //        {
  //          res.push_back(getObjectByID((*it)["I"].toString(), objToQueryFor).c_str());
  //        }
  //      }
  //      else
  //      {
  //        outError("Object to inspect is invalid!");
  //        return false;
  //      }
  //    }
  //    else
  {
    outWarn("******************************************");
    std::string inspectionAnnotator = "";
    json_prolog::Prolog pl;
    std::string plQueryString = "class_properties('" + thorinObjects_[objToInspect] +
                                +"','http://knowrob.org/kb/thorin_parts.owl#inspectableBy', A)";
    outInfo("asking query: " << plQueryString);
    json_prolog::PrologQueryProxy bdgs = pl.query(plQueryString);
    for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
    {
      inspectionAnnotator = (*it)["A"].toString();
      outInfo(objToInspect << " inspectable by: " << inspectionAnnotator);
    }

    std::vector<std::string> inspectionPipeline;
    if(inspectionAnnotator != "")
    {
      outInfo("Planning a pipeline for this annotator");
      plQueryString = "build_pipeline(['" + inspectionAnnotator + "'], P ).";
      outInfo("asking query: " << plQueryString);
      bdgs = pl.query(plQueryString);

      for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
      {
        std::string pipeline = (*it)["P"].toString();
        inspectionPipeline =  prologInterface->createPipelineFromPrologResult(pipeline);
      }
    }
    res.insert(res.end(), inspectionPipeline.begin(), inspectionPipeline.end());

  }
  return true;
}

bool QueryInterface::handleDetect(std::vector<std::string> &res)
{

  const rapidjson::Value &val = query["detect"];
  rapidjson::StringBuffer strBuff;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strBuff);
  val.Accept(writer);

  std::string req = strBuff.GetString();

  if(rs::DesignatorWrapper::req_designator)
  {
    delete rs::DesignatorWrapper::req_designator;
  }

  rs::DesignatorWrapper::req_designator = new rapidjson::Document();
  rs::DesignatorWrapper::req_designator->Parse(req.c_str());
  rs::DesignatorWrapper::req_designator->AddMember("type", "action", rs::DesignatorWrapper::req_designator->GetAllocator());

  std::vector<std::string> keys;
  std::vector<std::string> new_pipeline_order;
  prologInterface->extractQueryKeysFromDesignator(&req, keys);
  try
  {
    prologInterface->planPipelineQuery(keys, new_pipeline_order);
  }
  catch(std::exception e)
  {
    outError("calling json_prolog was not successfull. Is the node running?");
    return false;
  }

  if(new_pipeline_order.empty())
  {
    outInfo("Can't find solution for pipeline planning");
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
    new_pipeline_order.push_back("BeliefToKnowRob");
  }
  new_pipeline_order.push_back("StorageWriter");

  res.insert(res.end(), new_pipeline_order.begin(), new_pipeline_order.end());

  //    for(auto & r : res)
  //    {
  //      outInfo("Before:" << r);
  //      rapidjson::Document d;
  //      d.Parse(r.c_str());
  //      std::string className;
  //      if(d.HasMember("class"))
  //      {
  //        rapidjson::Value &jClass = d["class"];
  //        className = jClass["name"].GetString();
  //      }

  //      std::string objectID = "http://knowrob.org/kb/thorin_simulation.owl#" + className + "1";
  //      std::string childFrameId = className + "1";

  //      if(std::find(seenObjects_.begin(), seenObjects_.end(), className) == seenObjects_.end())
  //      {
  //        outInfo("this is the first time I see this object. Get new ID");
  //        seenObjects_.push_back(className);
  //        outInfo(thorinObjects_[className]);

  //        json_prolog::Prolog pl;
  //        std::string query = "get_new_object_id('" + thorinObjects_[className] + "',OID)";
  //        outInfo("Asking query: " << query);
  //        json_prolog::PrologQueryProxy bdgs = pl.query(query);
  //        for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
  //        {
  //          outInfo("Got Object ID: " << (*it)["OID"].toString());
  //        }
  //      }

  //      if(d.HasMember("pose"))
  //      {
  //        outInfo("Foind pose in result string...looking for transform");
  //        rapidjson::Value &jPose = d["pose"];
  //        if(jPose.HasMember("transform"))
  //        {
  //          outInfo("Found the transform");
  //          rapidjson::Value &jTransf = jPose["transform"];
  //          if(std::strcmp(jTransf["child_frame_id"].GetString(), "") == 0)
  //          {
  //            outInfo("found emtpy child_frame_id. Overwriting");
  //            jTransf["child_frame_id"].SetString(childFrameId.c_str(), childFrameId.length());
  //          }
  //        }
  //      }
  //      if(d.HasMember("id"))
  //      {
  //        outInfo("found id in response. resolving through KR");
  //        d["id"].SetString(objectID.c_str(), objectID.length());
  //      }

  //      rapidjson::StringBuffer buffer;
  //      rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  //      d.Accept(writer);
  //      r = buffer.GetString();
  //      outInfo("After: " << buffer.GetString());
  //    }
  //handle the json
  return true;

}

bool getConfigForKey(std::string key, std::string &location, std::string &check)
{
  //TODO Error handling
  const std::string &configFile = ros::package::getPath("rs_queryanswering") + "/config/filter_config.ini";

  outInfo("Path to config file: " FG_BLUE << configFile);
  boost::property_tree::ptree pt;
  try
  {
    boost::property_tree::ini_parser::read_ini(configFile, pt);
    boost::optional<boost::property_tree::ptree&> child = pt.get_child_optional(key);
    //Use config if we have one, otherwise use default
    if(child) {
        location = pt.get<std::string>(key + ".location");
        check = pt.get<std::string>(key + ".check");
    } else {
        location = "/"+ key;
        check = "EQUAL";
    }
    return true;
  }
  catch(boost::property_tree::ini_parser::ini_parser_error &e)
  {
    throw_exception_message("Error opening config file: " + configFile);
    return false;
  }
  return false;
}

//TODO this should be more generic, a few hacks in here. Currently first checking subclass, then checking equal
//if superclass not set
bool QueryInterface::checkSubClass(const std::string &resultValue, const std::string &superclass, const std::string &queryValue){
    bool ok = false;
    if(superclass != "" && rs_queryanswering::krNameMapping.count(superclass) == 1)
    {
      try
      {
        ok = prologInterface->q_subClassOf(resultValue, superclass);
      }
      catch(std::exception &e)
      {
        outError("Prolog Exception: Malformed owl_subclass of. Child or superclass undefined:");
        outError("     Child: " << resultValue);
        outError("     Parent: " << superclass);
      }
    }
    else if(strcasecmp(resultValue.c_str(), queryValue.c_str()) == 0 || queryValue == "")
    {
      ok = true;
    }
    else if(resultValue == "bottle_acid" || resultValue == "bottle_base")
    {
      std::string new_name = "bottle";
      if(strcasecmp(new_name.c_str(), queryValue.c_str()) == 0)
      {
        ok = true;
      }
    }
    return ok;
}

void QueryInterface::filterResults(std::vector<std::string> &resultDesignators,
                                   std::vector<std::string> &filteredResponse,
                                   std::vector<bool> &designatorsToKeep,
                                   const std::string superclass)
{

  const rapidjson::Value &detectQuery = query["detect"];
  designatorsToKeep.resize(resultDesignators.size(), true);

  for(rapidjson::Value::ConstMemberIterator queryIt = detectQuery.MemberBegin(); queryIt != detectQuery.MemberEnd(); ++queryIt)
  {
    std::string location, check;
    std::string key = queryIt->name.GetString();
    getConfigForKey(key, location, check);
    const std::string queryValue = queryIt->value.GetString();

    outInfo("No. of resulting Object Designators: " << resultDesignators.size());
    for(size_t i = 0; i < resultDesignators.size(); ++i)
    {
      rapidjson::Document resultJson;
      resultJson.Parse(resultDesignators[i].c_str());

      //check if this query key exists in the result
      if(rapidjson::Value *value = rapidjson::Pointer(location).Get(resultJson))
      {
        if(check == "EQUAL")
        {
          std::string resultValue = value->GetString();;
          if(resultValue != queryValue) {
              designatorsToKeep[i] = false;
          }
        }
        else if(check == "CLASS")
        {
            const std::string resultValue = value->GetString();
            if(!checkSubClass(resultValue, superclass, queryValue)) {
                designatorsToKeep[i] = false;
            }
        }
        else if(check == "GEQ") {
            float volumeofCurrentObj = value->GetDouble();
            float volumeAsked = atof(queryValue.c_str());
              outWarn("Volume asked as float: " << volumeAsked);
              if(volumeAsked > volumeofCurrentObj)
              {
                designatorsToKeep[i] = false;
              }
        }
      }
      else {
        designatorsToKeep[i] = false;
      }

    }
  }
  for(int i = 0; i < designatorsToKeep.size(); ++i)
  {
    if(designatorsToKeep[i])
    {
      filteredResponse.push_back(resultDesignators[i]);
    }
  }
}


void QueryInterface::oldFilter(std::vector<std::string> &resultDesignators,
                               std::vector<std::string> &filteredResponse,
                               std::vector<bool> &designatorsToKeep,
                               const std::string superclass)
{
  outInfo("filtering the results based on the designator request");
  outInfo("Superclass: " << superclass);
  designatorsToKeep.resize(resultDesignators.size(), true);
  const rapidjson::Value &detectQuery = query["detect"];

  for(rapidjson::Value::ConstMemberIterator it = detectQuery.MemberBegin(); it != detectQuery.MemberEnd(); ++it)
  {
    if(it->name == "timestamp" || it->name == "location")
    {
      continue;
    }

    outInfo("No. of resulting Object Designators: " << resultDesignators.size());
    for(size_t i = 0; i < resultDesignators.size(); ++i)
    {
      rapidjson::Document resDesig;
      resDesig.Parse(resultDesignators[i].c_str());
      rapidjson::Document resultsForRequestedKey;
      resultsForRequestedKey.SetObject();
      if(resDesig.HasMember("id"))
      {
        if(it->name == "size") //size is nested get it from the bounding box..bad design
        {
          if(resDesig.HasMember("boundingbox"))
          {
            resultsForRequestedKey.AddMember("size", resDesig["boundingbox"]["size"], resultsForRequestedKey.GetAllocator());
          }
          else
          {
            designatorsToKeep[i] = false;
          }
        }
        else if(it->name == "cad-model")
        {
          resultsForRequestedKey.AddMember("cad-model", resDesig["pose"], resultsForRequestedKey.GetAllocator());
        }
        else if(it->name == "volume")
        {
          if(resDesig.HasMember("volume"))
          {
            resultsForRequestedKey.AddMember("volume", resDesig["volume"], resultsForRequestedKey.GetAllocator());
          }
          else
          {
            designatorsToKeep[i] = false;
          }
        }
        else if(it->name == "contains")
        {
          if(resDesig.HasMember("contains"))
          {
            resultsForRequestedKey.AddMember("contains", resDesig["contains"], resultsForRequestedKey.GetAllocator());
          }
          else
          {
            designatorsToKeep[i] = false;
          }
        }
        else if(it->name == "shape") //there can be multiple shapes and these are not nested
        {
          if(resDesig.HasMember("shape"))
          {
            resultsForRequestedKey.AddMember("shape", resDesig["shape"], resultsForRequestedKey.GetAllocator());
          }
          else
          {
            designatorsToKeep[i] = false;
          }
        }
        else if(it->name == "type")//this shit needed so we don't loose al of our stuff just because all was sent instead of detection
        {
          //TODO should it always have class?
          if(resDesig.HasMember("class"))
          {
            resultsForRequestedKey.AddMember("type", resDesig["class"], resultsForRequestedKey.GetAllocator());
          }
        }
        else if(it->name == "ingredient")
        {
          resultsForRequestedKey.AddMember("ingredient", resDesig["pizza"], resultsForRequestedKey.GetAllocator());
        }
        else
        {
          rapidjson::Value v(it->name, resultsForRequestedKey.GetAllocator());
          resultsForRequestedKey.AddMember(v, resDesig[it->name.GetString()], resultsForRequestedKey.GetAllocator());

        }
      }
      else
      {
        rapidjson::Value v(it->name, resultsForRequestedKey.GetAllocator());
        resultsForRequestedKey.AddMember(v, resDesig[it->name.GetString()], resultsForRequestedKey.GetAllocator());
        outWarn("No CLUSTER ID");
      }

      if(!resultsForRequestedKey.MemberCount() == 0)
      {
        bool ok = false;

        if(resultsForRequestedKey.HasMember("pose"))
        {
          rapidjson::Document kvps_;
          kvps_.Parse(resultDesignators[i].c_str());
          rapidjson::Value::ConstMemberIterator it = kvps_.MemberBegin();
          bool hasCadPose = false;
          while(it != kvps_.MemberEnd())
          {
            if(it->name == "pose")
            {
              if(kvps_["source"].GetString() == "TemplateAlignment")
              {
                hasCadPose = true;
                ++it;
              }
              else
              {
                kvps_.EraseMember(it++);
              }
            }
            else
            {
              ok = true;
            }
          }
          ok = hasCadPose;
          resultDesignators[i] = rs::DesignatorWrapper::jsonToString(kvps_).c_str();
        }
        if(resultsForRequestedKey.HasMember("obj-part"))
        {
          ok = true;
        }
        if(resultsForRequestedKey.HasMember("pizza"))
        {
          ok = true;
          rapidjson::Document kvps_;
          kvps_.Parse(resultDesignators[i].c_str());
          rapidjson::Value::ConstMemberIterator it = kvps_.MemberBegin();
          while(it != kvps_.MemberEnd())
          {
            if(it->name != "pizza" && it->name != "id" && it->name != "timestamp")
            {
              kvps_.EraseMember(it++);
            }
            else
            {
              ++it;
            }

          }
          resultDesignators[i] = rs::DesignatorWrapper::jsonToString(kvps_).c_str();
        }
        //treat color differently because it is nested and has every color with ration in there
        if(resultsForRequestedKey.HasMember("color"))
        {
          for(auto iter = resultsForRequestedKey["color"].MemberBegin(); iter != resultsForRequestedKey["color"].MemberEnd(); ++iter)
          {
            if(strcasecmp(iter->value.GetString(), it->value.GetString()) == 0 || it->value.GetString() == "")
            {
              if(iter->value.GetDouble() > 0.20)
              {
                ok = true;
              }
            }
          }
        }
        if(resultsForRequestedKey.HasMember("volume"))
        {
          float volumeofCurrentObj = resultsForRequestedKey["volume"].GetDouble();
          outWarn("Volume as a float: " << volumeofCurrentObj);
          if(it->value.GetString() == "")
          {
            ok = true;
          }
          else
          {
            float volumeAsked = atof(it->value.GetString());
            outWarn("Volume asked as float: " << volumeAsked);
            if(volumeAsked <= volumeofCurrentObj)
            {
              ok = true;
            }
          }
        }
        if(resultsForRequestedKey.HasMember("contains"))
        {
          if(it->value.GetString() == "")
          {
            ok = true;
          }
          else
          {
            std::string substanceName = resultsForRequestedKey["contains"]["substance"].GetString();
            std::string substanceAsked = it->value.GetString();
            outWarn("Substance asked : " << substanceAsked);
            if(strcasecmp(substanceName.c_str(), substanceAsked.c_str()) == 0)
            {
              ok = true;
            }
          }
        }

        //another nested kv-p...we need a new interface...this one sux
        if(resultsForRequestedKey.HasMember("class"))
        {
          for(auto iter = resultsForRequestedKey["class"].MemberBegin(); iter != resultsForRequestedKey["class"].MemberEnd(); ++iter)
          {
            if(iter->name.GetString() == "name")
            {
              if(superclass != "" && rs_queryanswering::krNameMapping.count(superclass) == 1)
              {
                try
                {
                  ok = prologInterface->q_subClassOf(iter->value.GetString(), superclass);
                }
                catch(std::exception &e)
                {
                  outError("Prolog Exception: Malformed owl_subclass of. Child or superclass undefined:");
                  outError("     Child: " << iter->name.GetString());
                  outError("     Parent: " << superclass);
                }
              }
              else if(strcasecmp(iter->value.GetString(), it->value.GetString()) == 0 || it->value.GetString() == "")
              {
                ok = true;
              }
              else if(iter->value.GetString() == "bottle_acid" || iter->value.GetString() == "bottle_base")
              {
                std::string new_name = "bottle";
                if(strcasecmp(new_name.c_str(), it->value.GetString()) == 0)
                {
                  ok = true;
                }
              }
            }
          }
        }

        if(strstr(rs::DesignatorWrapper::jsonToString(resultsForRequestedKey).c_str(), it->value.GetString()) != NULL
           || it->value.GetString() == "")
        {
          ok = true;
        }
        if(!ok)
        {
          designatorsToKeep[i] = false;
        }
      }
    }
  }

  for(int i = 0; i < designatorsToKeep.size(); ++i)
  {
    if(designatorsToKeep[i])
    {
      filteredResponse.push_back(resultDesignators[i]);
    }
  }
}

std::string QueryInterface::getObjectByID(std::string OID, std::string type)
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


std::string QueryInterface::toJson(const tf::StampedTransform &pose, std::string OID, std::string type)
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
