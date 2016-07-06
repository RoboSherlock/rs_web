#ifndef RSCONTROLEDAEMANAGER_H
#define RSCONTROLEDAEMANAGER_H

#include <rs/utils/RSAnalysisEngineManager.h>

#include <rs_kbreasoning/DesignatorWrapper.h>
#include <rs_kbreasoning/RSControledAnalysisEngine.h>
#include <rs_kbreasoning/KRDefinitions.h>

#include <json_prolog/prolog.h>

#include <designator_integration_msgs/DesignatorCommunication.h>
#include <iai_robosherlock_msgs/SetRSContext.h>


class RSControledAEManager
  //  public RSAnalysisEngineManager<RSControledAnalysisEngine>
{

private:

  RSControledAnalysisEngine engine;

  ros::NodeHandle nh_;
  ros::Publisher desig_pub_;
  ros::ServiceServer service, singleService, setContextService;

  const bool waitForServiceCall_;
  const bool useVisualizer_;

  std::mutex processing_mutex_;

  rs::Visualizer visualizer_;

public:
  RSControledAEManager(const bool useVisualizer, const std::string &savePath,
                       const bool &waitForServiceCall, ros::NodeHandle n):
    nh_(n), waitForServiceCall_(waitForServiceCall),
    useVisualizer_(useVisualizer),    visualizer_(savePath)
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

    service = n.advertiseService("designator_request/all_solutions",
                                 &RSControledAEManager::designatorAllSolutionsCallback, this);

    // Call this service, if RoboSherlock should try out only
    // the pipeline with all Annotators, that provide the requested types (for example shape)
    singleService = n.advertiseService("designator_request/single_solution",
                                       &RSControledAEManager::designatorSingleSolutionCallback, this);

    // Call this service to switch between AEs
    setContextService = n.advertiseService("set_context", &RSControledAEManager::resetAECallback, this);

  }
  ~RSControledAEManager()
  {
    uima::ResourceManager::deleteInstance();
  }

  /*brief
   * in desig: query as designator
   * out prologQuery: the Prolog Query as a string
   * returns true or false /success or fail
   */
  bool buildPrologQueryFromDesignator(designator_integration::Designator *desig,
                                      std::string &prologQuery);

  /*brief
   * Create a vector of Annotator Names from the result of the knowrob_rs library.
   * This vector can be used as input for RSControledAnalysisEngine::setNextPipelineOrder
   */
  std::vector<std::string> createPipelineFromPrologResult(std::string result);

  /*brief
   * init the AE
   **/
  void init(std::string &xmlFile)
  {
    engine.init(xmlFile);
    if(useVisualizer_)
    {
      visualizer_.start();
    }
  }

  /* brief
   * run the AE in the manager
   */
  void run();

  void stop()
  {
    if(useVisualizer_)
    {
      visualizer_.stop();
    }
    engine.resetCas();
    engine.stop();
  }

  bool resetAECallback(iai_robosherlock_msgs::SetRSContext::Request &req,
                       iai_robosherlock_msgs::SetRSContext::Response &res);

  bool designatorAllSolutionsCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
                                      designator_integration_msgs::DesignatorCommunication::Response &res);

  bool designatorSingleSolutionCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
                                        designator_integration_msgs::DesignatorCommunication::Response &res);

  bool designatorCallbackLogic(designator_integration_msgs::DesignatorCommunication::Request &req,
                               designator_integration_msgs::DesignatorCommunication::Response &res, bool allSolutions);

  //TODO: move to Designator wrapper or somewhere else
  void filterResults(designator_integration::Designator &requestDesignator,
                     const std::vector<designator_integration::Designator> &resultDesignators,
                     std::vector<designator_integration::Designator> &filteredResponse,
                     std::string superclass);

  void overwriteParentField(designator_integration::KeyValuePair &d, int level);

};

#endif // RSCONTROLEDAEMANAGER_H
