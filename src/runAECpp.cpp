/*------------------------------------------------------------------------

 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.

 --------------------------------------------------------------------------

 Test driver that reads text files or XCASs or XMIs and calls the annotator

 -------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <sstream>

#include <uima/api.hpp>
#include "uima/internal_aggregate_engine.hpp"
#include "uima/annotator_mgr.hpp"

#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/exception.h>
#include <rs/io/visualizer.h>
#include <rs/scene_cas.h>

#include <designator_integration_msgs/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>
#include <iai_robosherlock_msgs/SetRSContext.h>

#include <rs_kbreasoning/RSPipelineManager.h>
#include <rs_kbreasoning/DesignatorWrapper.h>

// designator_integration classes
#include <designators/Designator.h>


#include <ros/ros.h>
#include <ros/package.h>

#include <json_prolog/prolog.h>
// TODO
//  Allow the modifaction of multiple AEs

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG

#define SEARCHPATH "/descriptors/analysis_engines/"
#define QUERY "QUERY"
// This mutex will be locked, if:
//   1) The RSAnalysisEngineManager wants to execute pipelines
//   or
//   2) If a service call has come in and needs several process calls
//      on a RSAnalysisEngine to complete
std::mutex processing_mutex;

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

struct RSQuery
{
  uint64_t timestamp = 0;
  std::string location = "";
  std::string objToInspect = "";
};


class RSAnalysisEngine
{
  // boost::mutex mx;
  // std::mutex process_mutex;
private:
  std::string name;

  uima::AnalysisEngine *engine;
  uima::CAS *cas;

  RSPipelineManager *rspm;
  bool pipeline_ordering_to_change;
  std::vector<std::string> next_pipeline_order;

  boost::shared_ptr<std::mutex> process_mutex;

  /**need a nicer solution for this so we can set annotation to the
  CAS based on the ROS message**/


public:

  void setNextPipelineOrder(std::vector<std::string> l)
  {
    next_pipeline_order = l;
    setPipelineOrderChange(true);
  }

  std::vector<std::string> &getNextPipelineOrder()
  {
    return next_pipeline_order;
  }

  void applyNextPipelineOrder()
  {
    if(rspm)
    {
      rspm->setPipelineOrdering(next_pipeline_order);
    }
    setPipelineOrderChange(false);
  }

  RSAnalysisEngine() : engine(NULL), cas(NULL), rspm(NULL), pipeline_ordering_to_change(false)
  {
    process_mutex = boost::shared_ptr<std::mutex>(new std::mutex);
  }

  ~RSAnalysisEngine()
  {
    if(cas)
    {
      delete cas;
      cas = NULL;
    }
    if(engine)
    {
      delete engine;
      engine = NULL;
    }

    if(rspm)
    {
      delete rspm;
      rspm = NULL;
    }
  }

  bool pipelineOrderChange()
  {
    return pipeline_ordering_to_change;
  }

  void setPipelineOrderChange(bool b)
  {
    pipeline_ordering_to_change = b;
  }

  void resetPipelineOrdering()
  {
    if(rspm)
    {
      rspm->resetPipelineOrdering();
    }
  }

  bool defaultPipelineEnabled()
  {
    if(rspm)
    {
      return rspm->use_default_pipeline;
    }
    return false;
  }

  void init(const std::string &file)
  {
    uima::ErrorInfo errorInfo;

    size_t pos = file.rfind('/');
    outInfo("Creating analysis engine: " FG_BLUE << (pos == file.npos ? file : file.substr(pos)));

    engine = uima::Framework::createAnalysisEngine(file.c_str(), errorInfo);


    if(errorInfo.getErrorId() != UIMA_ERR_NONE)
    {
      outError("createAnalysisEngine failed.");
      throw uima::Exception(errorInfo);
    }
    // RSPipelineManager rspm(engine);
    rspm = new RSPipelineManager(engine);
    std::vector<icu::UnicodeString> &non_const_nodes = rspm->getFlowConstraintNodes();

    outInfo("*** Fetch the FlowConstraint nodes. Size is: "  << non_const_nodes.size());
    for(int i = 0; i < non_const_nodes.size(); i++)
    {
      //      outInfo(non_const_nodes.at(i));
    }

    rspm->aengine->getNbrOfAnnotators();
    outInfo("*** Number of Annotators in AnnotatorManager: " << rspm->aengine->getNbrOfAnnotators());

    // After all annotators have been initialized, pick the default pipeline
    std::vector<std::string> default_pipeline;
    default_pipeline.push_back("CollectionReader");
    default_pipeline.push_back("ImagePreprocessor");
    default_pipeline.push_back("RegionFilter");
    default_pipeline.push_back("NormalEstimator");
    default_pipeline.push_back("PlaneAnnotator");
    default_pipeline.push_back("PointCloudClusterExtractor");
    default_pipeline.push_back("Cluster3DGeometryAnnotator");
    default_pipeline.push_back("ClusterTFLocationAnnotator");
    default_pipeline.push_back("SacModelAnnotator");
    default_pipeline.push_back("PrimitiveShapeAnnotator");
    default_pipeline.push_back("KBResultAdvertiser");
    //    default_pipeline.push_back("StorageWriter");
    // default_pipeline.push_back("ClusterColorHistogramCalculator");
    // removed color histogram for tests
    rspm->setDefaultPipelineOrdering(default_pipeline);

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
  }

  void stop()
  {
    engine->collectionProcessComplete();
    engine->destroy();

    outInfo("Analysis engine stopped: " << name);
  }

  void process()
  {
    // TODO Use ptrs to avoid unnecessary memory allocation?
    designator_integration_msgs::DesignatorResponse d;
    process(d);
  }

  void process(designator_integration_msgs::DesignatorResponse &designator_response, RSQuery *q = NULL) //uint64_t timestamp)
  {
    outInfo("executing analisys engine: " << name);
    try
    {
      UnicodeString ustrInputText;
      ustrInputText.fromUTF8(name);
      cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
      if(q != NULL)
      {
        rs::SceneCas sceneCas(*cas);
        rs::Query ts = rs::create<rs::Query>(*cas);
        if(q->timestamp != 0)
        {
          ts.timestamp.set(q->timestamp);
        }
        if(q->location != "")
        {
          ts.location.set(q->location);
        }
        if(q->objToInspect != "")
        {
          ts.inspect.set(q->objToInspect);
        }
        outInfo("setting in CAS: ts:" << q->timestamp << " location: " << q->location);
        sceneCas.setFS(QUERY, ts);
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
    cas->reset();
    outInfo("processing finished");
  }

  // Call process() and
  // decide if the pipeline should be reset or not
  void process(bool reset_pipeline_after_process, designator_integration_msgs::DesignatorResponse &designator_response)
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
  // Call process() and
  // decide if the pipeline should be reset or not
  void process(bool reset_pipeline_after_process)
  {
    designator_integration_msgs::DesignatorResponse d;
    process(reset_pipeline_after_process, d);
  }

  // Define a pipeline that should be executed,
  // process(reset_pipeline_after_process) everything and
  // decide if the pipeline should be reset or not
  void process(std::vector<std::string> annotators, bool reset_pipeline_after_process, designator_integration_msgs::DesignatorResponse &designator_response, RSQuery *query = NULL)
  {
    process_mutex->lock();
    outInfo(FG_CYAN << "process(std::vector, bool) - LOCK OBTAINED");
    setNextPipelineOrder(annotators);
    applyNextPipelineOrder();

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
  void process(std::vector<std::string> annotators, bool reset_pipeline_after_process)
  {
    designator_integration_msgs::DesignatorResponse d;
    process(annotators, reset_pipeline_after_process, d);
  }
};


//MULTIPLE ANALYSIS ENGINE MANAGER CLASS
class RSAnalysisEngineManager
{
private:
  std::vector<RSAnalysisEngine> engines;

  const bool useVisualizer;
  const bool waitForServiceCall;
  rs::Visualizer visualizer;

  ros::NodeHandle nh_;
  ros::Publisher desig_pub;


  std::map<std::string, std::string> krNameMapping;
public:
  RSAnalysisEngineManager(const bool useVisualizer, const std::string &savePath, const bool &waitForServiceCall, ros::NodeHandle n) :
    useVisualizer(useVisualizer), waitForServiceCall(waitForServiceCall), visualizer(savePath), nh_(n)
  {
    // Create/link up to a UIMACPP resource manager instance (singleton)
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

    desig_pub = nh_.advertise<designator_integration_msgs::DesignatorResponse>(std::string("result_advertiser"), 5);

    //superclasses
    krNameMapping["DRINK"] = "knowrob:'Drink'";
    krNameMapping["FOODORDRINKORINGREDIENT"] = "knowrob:'FoodOrDrinkOrIngredient'";
    krNameMapping["CONTAINER"] = "knowrob:'Container'";
    krNameMapping["COOKING UTENSIL"] = "knowrob:'CookingUtensil'";
    krNameMapping["ELECTRICAL DEVICE"] = "knowrob:'ElectricalDevice'";

    //objects
    krNameMapping["icetea"] = "rs_test_objects:'PfannerIceTea'";
    krNameMapping["mondamin"] = "rs_test_objects:'MondaminPancakeMix'";
    krNameMapping["cereal"] = "rs_test_objects:'KellogsCornFlakes'";
    krNameMapping["plate"] = "rs_test_objects:'Plate'";
    krNameMapping["pancake_maker"] = "rs_test_objects:'PancakeMaker'";
    krNameMapping["spatula"] = "rs_test_objects:'Spatula'";
    krNameMapping["pitcher"] = "rs_test_objects:'Pitcher'";
    krNameMapping["milk"] = "rs_test_objects:'Milk'";
    krNameMapping["cup"] = "rs_test_objects:'Cup'";
    krNameMapping["bottle"] = "rs_test_objects:'KimaxBottle'";
    krNameMapping["bottle_acid"] = "rs_test_objects:'KimaxBottle'";
    krNameMapping["bottle_base"] = "rs_test_objects:'KimaxBottle'";
    krNameMapping["flask_250ml"] = "rs_test_objects:'Flask'";
    krNameMapping["flask_400ml"] = "rs_test_objects:'Flask'";
    krNameMapping["pipette"]  = "rs_test_objects:'Pipette'";
    krNameMapping["mixer_ikamag"] = "rs_test_objects:'MixerIkaMag'";

  }

  ~RSAnalysisEngineManager()
  {
    uima::ResourceManager::deleteInstance();
  }

  // Returns a string with the prolog query to execute, based on the informations in the designator
  std::string buildPrologQueryFromDesignator(designator_integration::Designator *desig, bool &success)
  {
    success = false;
    if(!desig)
    {
      return "NULL POINTER PASSED TO RSAnalysisEngineManager::buildPrologQueryFromDesignator";
    }

    std::string ret = "";
    if(desig->childForKey("OBJECT"))
    {
      // If the designator contains a "type" key, the highlevel is looking for a specific object of Type XY.
      // Use the corresponding Prolog Rule for object pipeline generation

      ret = "build_pipeline_for_object('";
      // Fetch the accepted predicates from the Designator
      ret += desig->childForKey("OBJECT")->stringValue();
      ret += "', A)";
    }
    else
    {
      ret = "build_single_pipeline_from_predicates([";
      std::vector<std::string> listOfAllPredicates;

      // Fetch the accepted predicates from the Designator
      //this would not be needed given a proper function and a clean interface and no hacks
      if(desig->childForKey("SHAPE"))
      {
        listOfAllPredicates.push_back("shape");
      }
      if(desig->childForKey("VOLUME"))
      {
        listOfAllPredicates.push_back("volume");
      }
      if(desig->childForKey("CONTAINS"))
      {
        listOfAllPredicates.push_back("contains");
      }
      if(desig->childForKey("COLOR"))
      {
        listOfAllPredicates.push_back("color");
      }
      if(desig->childForKey("SIZE"))
      {
        listOfAllPredicates.push_back("size");
      }
      if(desig->childForKey("LOCATION"))
      {
        listOfAllPredicates.push_back("location");
      }
      if(desig->childForKey("LOGO"))
      {
        listOfAllPredicates.push_back("logo");
      }
      if(desig->childForKey("TEXT"))
      {
        listOfAllPredicates.push_back("text");
      }
      if(desig->childForKey("PRODUCT"))
      {
        listOfAllPredicates.push_back("product");
      }
      //todo: this will be a bit of a hack to get the web stuff running, let's see how this works out:)
      if(desig->childForKey("INSPECT"))
      {
        listOfAllPredicates.push_back("parts");
      }
      if(desig->childForKey("DETECTION"))
      {
        listOfAllPredicates.push_back("detection");
        designator_integration::KeyValuePair *kvp = desig->childForKey("DETECTION");
        if(kvp->stringValue() == "PANCAKE")
        {
          listOfAllPredicates.push_back("pancakedetector");
        }

      }
      if(desig->childForKey("TYPE"))
      {
        listOfAllPredicates.push_back("detection");
      }
      if(desig->childForKey("HANDLE"))
      {
        listOfAllPredicates.push_back("handle");
      }
      //      if(desig->childForKey("PANCAKE"))
      //      {
      //        listOfAllPredicates.push_back("detection");
      //        listOfAllPredicates.push_back("pancakedetector");
      //      }

      for(int i = 0; i < listOfAllPredicates.size(); i++)
      {
        ret += listOfAllPredicates.at(i);
        if(i < listOfAllPredicates.size() - 1)
        {
          ret += ",";
        }
      }

      ret += "], A)";
    }
    success = true;
    return ret;
  }

  // Create a vector of Annotator Names from the result of the
  // knowrob_rs library.
  // This vector can be used as input for RSAnalysisEngine::setNextPipelineOrder
  std::vector<std::string> createPipelineFromPrologResult(std::string result)
  {
    std::vector<std::string> new_pipeline;

    // Strip the braces from the result
    result.erase(result.end() - 1);
    result.erase(result.begin());

    std::vector<std::string> list_of_annotators;
    std::stringstream resultstream(result);

    std::string token;
    while(std::getline(resultstream, token, ','))
    {
      list_of_annotators.push_back(token);
      // erase leading whitespaces
      token.erase(token.begin(), std::find_if(token.begin(), token.end(), std::bind1st(std::not_equal_to<char>(), ' ')));
      outInfo("Planned Annotator " << token);

      // From the extracted tokens, remove the prefix
      std::string prefix("http://knowrob.org/kb/rs_components.owl#");
      int prefix_length = prefix.length();

      // Erase by length, to avoid string comparison
      token.erase(0, prefix_length);
      // outInfo("Annotator name sanitized: " << token );

      new_pipeline.push_back(token);
    }
    return new_pipeline;
  }

  bool designatorAllSolutionsCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
                                      designator_integration_msgs::DesignatorCommunication::Response &res)
  {
    return designatorCallbackLogic(req, res, true);
  }
  bool resetAECallback(iai_robosherlock_msgs::SetRSContext::Request &req,
                       iai_robosherlock_msgs::SetRSContext::Response &res)
  {
    std::string newContextName = req.newAe;
    std::vector<std::string> files;
    if(newContextName == "kitchen")
    {

      files.push_back(ros::package::getPath("rs_kbreasoning") + "/descriptors/analysis_engines/kitchen.xml");
      this->init(files);
      outInfo("called the service with new ContextName: " << newContextName);
      return true;
    }
    else if(newContextName == "chemlab")
    {

      files.push_back(ros::package::getPath("rs_kbreasoning") + "/descriptors/analysis_engines/chemlab.xml");
      this->init(files);
      outInfo("called the service with new ContextName: " << newContextName);
      return true;
    }
    else
    {
      return false;
    }

  }
  bool designatorSingleSolutionCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
                                        designator_integration_msgs::DesignatorCommunication::Response &res)
  {
    return designatorCallbackLogic(req, res, false);
  }

  // Should prolog execute all solutions?
  //   -> set allSolutions=true
  bool designatorCallbackLogic(designator_integration_msgs::DesignatorCommunication::Request &req,
                               designator_integration_msgs::DesignatorCommunication::Response &res, bool allSolutions)
  {
    designator_integration::Designator *desigRequest = new designator_integration::Designator(req.request.designator);
    outInfo("RS Query service called");

    RSQuery *query = new RSQuery();
    std::string superClass = "";
    if(desigRequest != NULL)
    {
      std::list<std::string> keys = desigRequest->keys();
      bool foundTS = false;
      bool foundLocation = false;
      bool foundInspect = false;
      bool foundDirectiveAll = false;
      for(std::list<std::string>::iterator it = keys.begin(); it != keys.end(); ++it)
      {
        if(*it == "TIMESTAMP")
        {
          foundTS = true;
        }
        if(*it == "LOCATION")
        {
          foundLocation = true;
        }
        if(*it == "INSPECT")
        {
          foundInspect = true;
        }
        if(*it == "TYPE")
        {
          foundDirectiveAll = true;
        }
      }
      if(foundTS)
      {
        designator_integration::KeyValuePair *kvp = desigRequest->childForKey("TIMESTAMP");
        std::string ts = kvp->stringValue();
        query->timestamp = std::stoll(ts);
        outInfo("received timestamp:" << query->timestamp);
      }
      if(foundLocation)
      {
        designator_integration::KeyValuePair *kvp = desigRequest->childForKey("LOCATION");
        query->location = kvp->stringValue();
        outInfo("received location:" << query->location);
      }
      if(foundInspect)
      {
        designator_integration::KeyValuePair *kvp = desigRequest->childForKey("INSPECT");
        query->objToInspect = kvp->stringValue();
        outInfo("received Inspection request for object: " << query->objToInspect);
      }
      if(foundDirectiveAll)
      {
        designator_integration::KeyValuePair *kvp = desigRequest->childForKey("TYPE");
        superClass = kvp->stringValue();
      }
    }

    if(desigRequest->type() != designator_integration::Designator::OBJECT)
    {
      outInfo(" ***** RECEIVED SERVICE CALL WITH UNHANDELED DESIGNATOR TYPE (everything != OBJECT) ! Aborting... ****** ");
      return false;
    }
    if(rs::DesignatorWrapper::req_designator)
    {
      delete rs::DesignatorWrapper::req_designator;
    }

    rs::DesignatorWrapper::req_designator = new designator_integration::Designator(req.request.designator);
    outInfo("Received Designator call: ");
    rs::DesignatorWrapper::req_designator->printDesignator();

    std::string prologQuery = "";
    bool plGenerationSuccess = false;
    prologQuery = buildPrologQueryFromDesignator(desigRequest, plGenerationSuccess);

    outInfo("Query Prolog with the following command: " << prologQuery);
    if(!plGenerationSuccess)
    {
      outInfo("Aborting Prolog Query... The generated Prolog Command is invalid");
      return false;
    }

    json_prolog::Prolog pl;
    json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery);


    if(bdgs.begin() == bdgs.end())
    {
      outInfo("Can't find solution for pipeline planning");
      return false; // Indicate failure
    }

    int pipelineId = 0;

    designator_integration_msgs::DesignatorResponse full_designator_response;

    // Block the RSAnalysisEngineManager  - We need the engines now
    outInfo("acquiring lock");
    processing_mutex.lock();
    outInfo("lock acquired");
    for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin();
        it != bdgs.end(); it++)
    {
      json_prolog::PrologBindings bdg = *it;
      std::string prologResult = bdg["A"].toString();
      std::vector<std::string> new_pipeline_order = createPipelineFromPrologResult(bdg["A"].toString());

      //needed for saving results and returning them on a ros topic
      if(waitForServiceCall && !new_pipeline_order.empty())
      {
        new_pipeline_order.push_back("KBResultAdvertiser");
      }
      outInfo(FG_BLUE << "Executing Pipeline #" << pipelineId);

      // First version. Change the pipeline on the first engine
      // to a fixed set
      if(engines.size() > 0)
      {

        // This designator response will hold the OBJECT designators with the detected
        // annotations for every detected object
        designator_integration_msgs::DesignatorResponse designator_response;
        outInfo("Executing pipeline generated by service call");
        engines.at(0).process(new_pipeline_order, true, designator_response, query);

        outInfo("Returned " << designator_response.designators.size() << " designators on this execution");

        // Add the PIPELINEID to reference the pipeline that was responsible for detecting
        // the returned object
        for(auto & designator : designator_response.designators)
        {
          // Convert the designator msg object to a normal Designator
          designator_integration::Designator d(designator);
          // Insert the current PIPELINEID
          d.setValue("PIPELINEID", pipelineId);
          full_designator_response.designators.push_back(d.serializeToMessage());
        }

        // Define an ACTION designator with the planned pipeline
        designator_integration::Designator pipeline_action;
        pipeline_action.setType(designator_integration::Designator::ACTION);
        std::list<designator_integration::KeyValuePair *> lstDescription;
        for(auto & annotatorName : new_pipeline_order)
        {
          designator_integration::KeyValuePair *oneAnno = new designator_integration::KeyValuePair();
          oneAnno->setValue(annotatorName);
          lstDescription.push_back(oneAnno);
        }
        pipeline_action.setValue("PIPELINEID", pipelineId);
        pipeline_action.setValue("ANNOTATORS", designator_integration::KeyValuePair::LIST, lstDescription);
        full_designator_response.designators.push_back(pipeline_action.serializeToMessage());
        // Delete the allocated keyvalue pairs for the annotator names
        for(auto & kvpPtr : lstDescription)
        {
          delete kvpPtr;
        }
        outInfo("Executing pipeline generated by service call: done");
      }
      else
      {
        outInfo("ERROR: No engine set up");
        return false;
      }

      if(!allSolutions)
      {
        break;  // Only take the first solution if allSolutions == false
      }
      pipelineId++;
    }

    // All engine calls have been processed. Release the Lock
    processing_mutex.unlock();
    //now filter the shit out of the results:D

    std::vector<designator_integration::Designator> filteredResponse;
    std::vector<designator_integration::Designator> resultDesignators;

    for(auto & designator : full_designator_response.designators)
    {
      // Convert the designator msg object to a normal Designator
      designator_integration::Designator d(designator);
      resultDesignators.push_back(d);
      d.printDesignator();
      res.response.designators.push_back(d.serializeToMessage());
    }

    filterResults(*desigRequest, resultDesignators, filteredResponse, superClass);
    outInfo("THE filteredResponse size:" << filteredResponse.size());
    designator_integration_msgs::DesignatorResponse topicResponse;
    for(auto & designator : filteredResponse)
    {
      //overwrite the parent field -> OPENEASE HACK
      overwriteParentField(designator, 0);
      topicResponse.designators.push_back(designator.serializeToMessage(false));
    }
    desig_pub.publish(topicResponse);

    delete query;
    outWarn("RS Query service call ended");
    return true;
  }

  void overwriteParentField(designator_integration::KeyValuePair &d, int level)
  {
    std::list<designator_integration::KeyValuePair *> children =  d.children();
    if(!children.empty())
    {
      for(std::list<designator_integration::KeyValuePair *>::iterator it = children.begin(); it != children.end(); ++it)
      {
        outDebug("parent of this guy is: " << (*it)->parent());
        (*it)->setParent(level);
        outDebug("new parent of this guy is: " << (*it)->parent());
//        designator_integration::KeyValuePair kvp = (*it);
        overwriteParentField(**it, level + 1);
      }
    }
  }

  void filterResults(designator_integration::Designator &requestDesignator,
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
        designator_integration::KeyValuePair *childRequestedKey = NULL;
        if(resDesig.childForKey("CLUSTERID") != NULL)
        {
          if(req_kvp.key() == "SIZE") //size is nested get it from the bounding box..bad design
          {
            childRequestedKey = resDesig.childForKey("BOUNDINGBOX")->childForKey("SIZE");
            resultsForRequestedKey.push_back(childRequestedKey);
          }
          else if(req_kvp.key() == "VOLUME")
          {
            childRequestedKey = resDesig.childForKey("VOLUME");
            if(childRequestedKey != NULL)
            {
              resultsForRequestedKey.push_back(childRequestedKey);
            }
            else
            {
              keep_designator[i] = false;
            }
          }

          else if(req_kvp.key() == "CONTAINS")
          {
            childRequestedKey = resDesig.childForKey("CONTAINS");
            if(childRequestedKey != NULL)
            {
              resultsForRequestedKey.push_back(childRequestedKey);
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
                      outWarn("filtering based on JSON required");
                      outWarn("Object looked at: " << childrenPair.stringValue());
                      std::stringstream prologQuery;
                      outWarn("Object should be subclass of: " << superclass);


                      prologQuery << "owl_subclass_of(" << krNameMapping[childrenPair.stringValue()] << "," << krNameMapping[superclass] << ").";
                      outWarn(prologQuery.str());
                      json_prolog::Prolog pl;
                      std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
                      json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
                      std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
                      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
                      outInfo("Querying through json_prolog took: " << duration << " ms");
                      if(bdgs.begin() == bdgs.end())
                      {
                        outInfo(krNameMapping[childrenPair.stringValue()] << " IS NOT " << krNameMapping[superclass]);
                        ok = false;
                      }
                      else
                      {
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
  }

  void init(const std::vector<std::string> &files)
  {
    engines.resize(files.size());
    for(size_t i = 0; i < engines.size(); ++i)
    {
      engines[i].init(files[i]);
    }
    if(useVisualizer)
    {
      visualizer.start();
    }
  }

  void run()
  {
    for(; ros::ok();)
    {

      if(waitForServiceCall)
      {
        usleep(100000);
      }
      else
      {
        processing_mutex.lock();

        for(size_t i = 0; i < engines.size(); ++i)
        {
          engines[i].process(true);
        }
        processing_mutex.unlock();
      }

      ros::spinOnce();
    }
  }

  void stop()
  {
    if(useVisualizer)
    {
      visualizer.stop();
    }
    for(size_t i = 0; i < engines.size(); ++i)
    {
      engines[i].stop();
    }
  }
};

/* ----------------------------------------------------------------------- */
/*       Main                                                              */
/* ----------------------------------------------------------------------- */

/**
 * Error output if program is called with wrong parameter.
 */
void help()
{
  std::cout << "Usage: runAECpp [options] analysisEngine.xml [...]" << std::endl
            << "Options:" << std::endl
            << "  -wait If using piepline set this to wait for a service call" << std::endl
            << "  -visualizer  Enable visualization" << std::endl
            << "  -save PATH   Path for storing images" << std::endl;
}

int main(int argc, char *argv[])
{
  /* Access the command line arguments to get the name of the input text. */
  if(argc < 2)
  {
    help();
    return 1;
  }

  ros::init(argc, argv, std::string("RoboSherlock_") + getenv("USER"));

  // Has JSON PROLOG been enabled at compile time?

  std::vector<std::string> args;
  args.resize(argc - 1);
  for(int argI = 1; argI < argc; ++argI)
  {
    args[argI - 1] = argv[argI];
  }
  bool useVisualizer = false;
  bool waitForServiceCall = false;
  std::string savePath = getenv("HOME");

  size_t argO = 0;
  for(size_t argI = 0; argI < args.size(); ++argI)
  {
    const std::string &arg = args[argI];

    if(arg == "-visualizer")
    {
      useVisualizer = true;
    }
    else if(arg == "-wait")
    {
      waitForServiceCall = true;
    }
    else if(arg == "-save")
    {
      if(++argI < args.size())
      {
        savePath = args[argI];
      }
      else
      {
        outError("No save path defined!");
        return -1;
      }
    }
    else
    {
      args[argO] = args[argI];
      ++argO;
    }
  }
  args.resize(argO);

  struct stat fileStat;
  if(stat(savePath.c_str(), &fileStat) || !S_ISDIR(fileStat.st_mode))
  {
    outError("Save path \"" << savePath << "\" does not exist.");
    return -1;
  }

  std::vector<std::string> analysisEngineFiles;

  //generate a vector of possible paths for the analysis engine
  std::vector<std::string> searchPaths;

  //empty path for full path given as argument
  searchPaths.push_back("");
  //add core package path
  searchPaths.push_back(ros::package::getPath("robosherlock") + std::string(SEARCHPATH));

  //look for packages dependent on core and find their full path
  std::vector<std::string> child_packages;
  ros::package::command("depends-on robosherlock", child_packages);
  for(int i = 0; i < child_packages.size(); ++i)
  {
    searchPaths.push_back(ros::package::getPath(child_packages[i]) + std::string(SEARCHPATH));
  }

  analysisEngineFiles.resize(args.size(), "");
  for(int argI = 0; argI < args.size(); ++argI)
  {
    const std::string &arg = args[argI];
    struct stat fileStat;

    for(size_t i = 0; i < searchPaths.size(); ++i)
    {
      const std::string file = searchPaths[i] + arg;
      const std::string fileXML = file + ".xml";

      if(!stat(file.c_str(), &fileStat) && S_ISREG(fileStat.st_mode))
      {
        analysisEngineFiles[argI] = file;
        break;
      }
      else if(!stat(fileXML.c_str(), &fileStat) && S_ISREG(fileStat.st_mode))
      {
        analysisEngineFiles[argI] = fileXML;
        break;
      }
    }

    if(analysisEngineFiles[argI].empty())
    {
      outError("analysis engine \"" << arg << "\" not found.");
      return -1;
    }
  }

  ros::NodeHandle n("~");
  try
  {
    RSAnalysisEngineManager manager(useVisualizer, savePath, waitForServiceCall, n);
    ros::ServiceServer service, singleService, setContextService;

    // Call this service, if RoboSherlock should try out every possible pipeline
    // that has been generated by the pipeline planning
    service = n.advertiseService("designator_request/all_solutions",
                                 &RSAnalysisEngineManager::designatorAllSolutionsCallback, &manager);
    // Call this service, if RoboSherlock should try out only
    // the pipeline with all Annotators, that provide the requested types (for example shape)
    singleService = n.advertiseService("designator_request/single_solution",
                                       &RSAnalysisEngineManager::designatorSingleSolutionCallback, &manager);
    setContextService = n.advertiseService("set_context",
                                           &RSAnalysisEngineManager::resetAECallback, &manager);


    manager.init(analysisEngineFiles);
    manager.run();
    manager.stop();
  }
  catch(const rs::Exception &e)
  {
    outError("Exception: " << std::endl << e.what());
    return -1;
  }
  catch(const uima::Exception &e)
  {
    outError("Exception: " << std::endl << e);
    return -1;
  }
  catch(const std::exception &e)
  {
    outError("Exception: " << std::endl << e.what());
    return -1;
  }
  catch(...)
  {
    outError("Unknown exception!");
    return -1;
  }
  return 0;
}
