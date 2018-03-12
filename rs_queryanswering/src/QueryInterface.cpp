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

  return true;

}

bool getConfigForKey(std::string key, std::string &location, std::string &check, double &thresh, bool &keepLower)
{
  const std::string &configFile = ros::package::getPath("rs_queryanswering") + "/config/filter_config.ini";

  outInfo("Path to config file: " FG_BLUE << configFile);
  boost::property_tree::ptree pt;
  try
  {
    boost::property_tree::ini_parser::read_ini(configFile, pt);

    location = pt.get<std::string>(key + ".location", "/" + key);
    check = pt.get<std::string>(key + ".check", "EQUAL");
    thresh = pt.get<double> (key + ".threshold", 0.f);
    keepLower = pt.get<bool> (key + ".keepLower", true);

    return true;
  }
  catch(boost::property_tree::ini_parser::ini_parser_error &e)
  {
    throw_exception_message("Error opening config file: " + configFile);
    return false;
  }
  return false;
}

bool QueryInterface::checkSubClass(const std::string &resultValue, const std::string &queryValue)
{
  bool ok = false;
  if(rs_queryanswering::krNameMapping.count(queryValue) == 1)
  {
    try
    {
      ok = prologInterface->q_subClassOf(resultValue, queryValue);
    }
    catch(std::exception &e)
    {
      outError("Prolog Exception: Malformed owl_subclass of. Child or superclass undefined:");
      outError("     Child: " << resultValue);
      outError("     Parent: " << queryValue);
    }
  }
  return ok;
}

bool QueryInterface::checkThresholdOnList(rapidjson::Value &list, const float threshold, std::string requestedKey, bool keepLower)
{
  for(rapidjson::Value::ConstMemberIterator listIt = list.MemberBegin(); listIt != list.MemberEnd(); ++listIt)
  {
    if(listIt->name == requestedKey)
    {
      if(!keepLower)
        if(listIt->value.GetDouble() >= threshold)
          return true;
        else if(listIt->value.GetDouble() < threshold)
          return true;
    }
  }
  return false;
}

void QueryInterface::filterResults(std::vector<std::string> &resultDesignators,
                                   std::vector<std::string> &filteredResponse,
                                   std::vector<bool> &designatorsToKeep)
{

  const rapidjson::Value &detectQuery = query["detect"];
  designatorsToKeep.resize(resultDesignators.size(), true);

  for(rapidjson::Value::ConstMemberIterator queryIt = detectQuery.MemberBegin(); queryIt != detectQuery.MemberEnd(); ++queryIt)
  {
    std::string location, check;
    std::string key = queryIt->name.GetString();
    double thresh;
    bool keepLower;
    getConfigForKey(key, location, check, thresh, keepLower);
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
          if(value->GetType() == rapidjson::Type::kStringType)
          {
            std::string resultValue = value->GetString();;
            if(resultValue != queryValue)
              designatorsToKeep[i] = false;
          }
          else
              designatorsToKeep[i] = false;
        }
        else if(check == "CLASS")
        {
          const std::string resultValue = value->GetString();
          if(!checkSubClass(resultValue, queryValue))
            designatorsToKeep[i] = false;
        }
        else if(check == "GEQ")
        {
          float volumeofCurrentObj = value->GetDouble();
          float volumeAsked = atof(queryValue.c_str());
          outWarn("Volume asked as float: " << volumeAsked);
          if(volumeAsked > volumeofCurrentObj)
          {
            designatorsToKeep[i] = false;
          }
        }
        else if(check == "THRESHLIST")
        {
          if(!checkThresholdOnList(*value, thresh, queryValue, keepLower))
          {
            designatorsToKeep[i] = false;
          }
        }
        else if(check == "CONTAINS")
        {
          bool found = false;
          for(auto &v : value->GetArray())
            if(v == queryValue)
              found = true;
          if(!found)
            designatorsToKeep[i] = false;
        }
        else
        {
          outWarn("There is no such check: " + check + ". Please check the filter_config.ini");
          designatorsToKeep[i] = false;
        }
      }
      else if(check == "CONTAINSEQUAL")
      {
        std::string delimiter = "*";
        int delLoc = location.find(delimiter);
        std::string prefix = location.substr(0, delLoc - 1);
        std::string suffix = location.substr(delLoc + 1, location.size());
        if(rapidjson::Value *suffixVal = rapidjson::Pointer(prefix).Get(resultJson))
        {
          for(int i = 0; i < suffixVal->Size(); i ++)
          {
            std::string newLocation = prefix + "/" + std::to_string(i) + suffix;
            if(rapidjson::Value *value = rapidjson::Pointer(newLocation).Get(resultJson))
            {
              std::string resultValue = value->GetString();;
              if(resultValue != queryValue)
                designatorsToKeep[i] = false;
            }
          }
        }
      }
      else
          designatorsToKeep[i] = false;
    }
  }
  for(int i = 0; i < designatorsToKeep.size(); ++i)
  {
    if(designatorsToKeep[i])
    {
      filteredResponse.push_back(resultDesignators[i]);
    }
  }
  outInfo("Matching Object Descriptions: " << filteredResponse.size());
}
