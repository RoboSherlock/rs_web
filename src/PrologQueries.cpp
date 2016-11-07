#include <designators/Designator.h>
#include <rs_kbreasoning/RSControledAnalysisEngine.h>
#include <rs/utils/RSAnalysisEngine.h>
#include <uima/api.hpp>
#include <SWI-cpp.h>
#include <iostream>
#include <string>
#include <memory>
#include <ros/package.h>
#include <stdio.h>
#include <dlfcn.h>

using namespace designator_integration;

Designator *req_desig = NULL;

static RSControledAnalysisEngine *ae_Proxy;
uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");

PREDICATE(add_designator,2)
{
 std::string desigType(char*(A1));
 std::cerr<<"Adding designator of type: "<<desigType<<"\n";
 Designator *desig = new Designator();
// if(desigType==std::string("Object"))
// {
//   desig->setType(Designator::OBJECT);
// }
 return A2 = static_cast<void *>(desig);
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
    return A1 = (void *)req_desig;
  }
  else
  {
    std::cerr << "Designator already initialized" << std::endl;
    return FALSE;
  }
}


/**
 * @brief initialize the AnalysisEngine object
 */
PREDICATE(init_rs, 2)
{
  if(!ae_Proxy)
  {
    ae_Proxy = new RSControledAnalysisEngine();
    dlopen("libpython2.7.so", RTLD_LAZY | RTLD_GLOBAL);
    std::string pipelineName((char *)A1);
    std::string pipelinePath;
    rs::common::getAEPaths(pipelineName, pipelinePath);
//    if(!pipelinePath.empty())
//    {
//      ae_Proxy->init(pipelinePath,"");
//      return A2 = (void *)ae_Proxy;
//    }
  }
  return FALSE;
}


/**
 * change AE file that is loaded, enables changing the context
 * e.g. from kitchen to cehmlab, where we need different parameterizations
 * */
PREDICATE(change_context, 1)
{
  if(ae_Proxy)
  {
    std::string pipelineName((char *)A1);
    std::string newPipelinePath;
    rs::common::getAEPaths(pipelineName, newPipelinePath);


    if(!newPipelinePath.empty())
    {
      std::string currentPipeline = ae_Proxy->getCurrentAEName();
      if(currentPipeline != newPipelinePath)
      {
        ae_Proxy->init(newPipelinePath,std::vector<std::string>());
        return TRUE;
      }
      else
      {
        outInfo("Already set to: "<<newPipelinePath);
        return FALSE;
      }
    }
    else
    {
      return FALSE;
    }
  }
  else
  {
    return FALSE;
  }
}

/**
 * @brief run the process function once
 */
PREDICATE(process_once, 1)
{
  void *myobj = A1;
  RSControledAnalysisEngine *ae  = (RSControledAnalysisEngine *)myobj;
  ae->process();
  return TRUE;
}

PREDICATE(set_new_pipeline, 1)
{
  if(ae_Proxy)
  {

    std::vector<std::string> new_pipeline;
    new_pipeline.push_back("CollectionReader");
    new_pipeline.push_back("ImagePreprocessor");
    ae_Proxy->setNextPipeline(new_pipeline);
    ae_Proxy->applyNextPipeline();
//    PlTail tail(A1);
//    PlTerm e;
//    while(tail.next(e))
//    {
//      std::cout << (char *)e << std::endl;
//    }
    return TRUE;
  }
  else
    return FALSE;
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

PREDICATE(write_list, 1)
{
  PlTail tail(A1);
  PlTerm e;

  while(tail.next(e))
  {
    std::cout << (char *)e << std::endl;
  }

  return TRUE;
}
