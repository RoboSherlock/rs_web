#include <designators/Designator.h>
#include <rs_kbreasoning/RSAnalysisEngineProxy.h>
#include <rs_kbreasoning/RSControledAnalysisEngine.h>
#include <rs/utils/RSAnalysisEngine.h>
#include <uima/api.hpp>
#include <SWI-cpp.h>
#include <iostream>
#include <string>
#include <memory>

designator_integration::Designator *req_desig = NULL;

//AnalysisEngineManagerProxy *ae_Proxy;
static RSControledAnalysisEngine *ae_Proxy;

PREDICATE(hello, 1)
{
  std::cout << "Hello " << (char *)A1 << std::endl;
  return TRUE;
}

PREDICATE(init_desig, 1)
{
  if(!req_desig)
  {
    req_desig = new designator_integration::Designator();
    designator_integration::KeyValuePair *some_shit =  new designator_integration::KeyValuePair("location", "on table");
    designator_integration::KeyValuePair *links = new designator_integration::KeyValuePair("location");
    links->addChild(some_shit);
    req_desig->addChild(links);
    return TRUE;
  }
  else
  {
    std::cerr << "Designator already initialized" << std::endl;
    return FALSE;
  }
}

PREDICATE(init_rs, 2)
{
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
  ae_Proxy = new RSControledAnalysisEngine();
  std::string pipelineName((char *)A1);
  ae_Proxy->init("/home/ferenc/work/ros_ws/src/robosherlock/descriptors/analysis_engines/storage.xml");
  std::vector<std::string> next_pipeline = ae_Proxy->getNextPipeline();
  std::cerr << "length of next pipeline: " << next_pipeline.size() << std::endl;
  return A1 = (void *)ae_Proxy;
}

PREDICATE(delete_desig, 1)
{
  if(req_desig)
  {
    delete req_desig;
    req_desig = NULL;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

PREDICATE(add_kvp, 2)
{
  std::string key = (std::string)A1;
  std::string value = (std::string)A2;
  designator_integration::KeyValuePair *kvp = new designator_integration::KeyValuePair(key, value);

  if(req_desig)
  {
    req_desig->addChild(kvp);
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

PREDICATE(print_desig, 1)
{
  if(req_desig)
  {
    req_desig->printDesignator();
    return TRUE;
  }
  else
  {
    std::cerr << "Desigantor object was destoyed. Can not print" << std::endl;
    return FALSE;
  }
}

PREDICATE(install, 1)
{
  return TRUE;
}

void install(int i)
{
  std::cerr << "WTF";
}

