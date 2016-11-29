#ifndef RSCONTROLEDAEMANAGER_H
#define RSCONTROLEDAEMANAGER_H

#include <rs/utils/RSAnalysisEngineManager.h>

#include <rs_queryanswering/DesignatorWrapper.h>
#include <rs_queryanswering/RSControledAnalysisEngine.h>
#include <rs_queryanswering/KRDefinitions.h>
#include <rs_queryanswering/PrologInterface.h>


#include <designator_integration_msgs/DesignatorCommunication.h>
#include <iai_robosherlock_msgs/SetRSContext.h>
#include <iai_robosherlock_msgs/RSQueryService.h>

#include <semrec_client/BeliefstateClient.h>
#include <semrec_client/Context.h>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

class RSProcessManager
{

private:

  RSControledAnalysisEngine engine;
  PrologInterface prologInterface;

  ros::NodeHandle nh_;
  ros::Publisher desig_pub_;
  ros::ServiceServer service, singleService, setContextService, jsonService;

  const bool waitForServiceCall_;
  const bool useVisualizer_;
  const bool useCWAssumption_;
  bool useIdentityResolution_;
  bool pause_;


  std::mutex processing_mutex_;

  rs::Visualizer visualizer_;

  semrec_client::BeliefstateClient *semrecClient;
  semrec_client::Context *ctxMain;

  std::string configFile;
  std::vector<std::string> closedWorldAssumption;
  std::vector<std::string> lowLvlPipeline_;

public:

  RSProcessManager(const bool useVisualizer, const std::string &savePath,
                   const bool &waitForServiceCall, const bool useCWAssumption, ros::NodeHandle n):
    engine(n), prologInterface(), nh_(n), waitForServiceCall_(waitForServiceCall),
    useVisualizer_(useVisualizer), useCWAssumption_(useCWAssumption), useIdentityResolution_(false), pause_(true), visualizer_(savePath)
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

    service = nh_.advertiseService("designator_request/all_solutions",
                                   &RSProcessManager::designatorAllSolutionsCallback, this);

    // Call this service, if RoboSherlock should try out only
    // the pipeline with all Annotators, that provide the requested types (for example shape)
    singleService = nh_.advertiseService("designator_request/single_solution",
                                         &RSProcessManager::designatorSingleSolutionCallback, this);

    // Call this service to switch between AEs
    setContextService = nh_.advertiseService("set_context", &RSProcessManager::resetAECallback, this);

    jsonService = nh_.advertiseService("json_query", &RSProcessManager::jsonQueryCallback, this);

    semrecClient = NULL;
    ctxMain = NULL;
  }
  ~RSProcessManager()
  {
    delete semrecClient;
    delete ctxMain;
    uima::ResourceManager::deleteInstance();
    outInfo("RSControledAnalysisEngine Stoped");
  }

  /*brief
   * init the AE Manager
   **/
  void init(std::string &xmlFile, std::string configFile)
  {
    this->configFile = configFile;
    cv::FileStorage fs(configFile, cv::FileStorage::READ);
    fs["cw_assumption"] >> closedWorldAssumption;
    if(lowLvlPipeline_.empty()) //if not set programatically, load from a config file
    {
      fs["annotators"] >> lowLvlPipeline_;
    }
    engine.init(xmlFile, lowLvlPipeline_);

    outInfo("Number of objects in closed world assumption: " << closedWorldAssumption.size());
    if(!closedWorldAssumption.empty() && useCWAssumption_)
    {
      for(auto cwa : closedWorldAssumption)
      {
        outInfo(cwa);
      }
      engine.setCWAssumption(closedWorldAssumption);
    }
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

  bool jsonQueryCallback(iai_robosherlock_msgs::RSQueryService::Request &req,
                         iai_robosherlock_msgs::RSQueryService::Response &res);

  bool handleQuery(designator_integration::Designator *req,
                   std::vector<designator_integration::Designator> &resp);

  //TODO: move to Designator wrapper or somewhere else
  void filterResults(designator_integration::Designator &requestDesignator,
                     std::vector<designator_integration::Designator> &resultDesignators,
                     std::vector<designator_integration::Designator> &filteredResponse,
                     std::string superclass);

  inline void setUseIdentityResolution(bool useIdentityResoltuion)
  {
    useIdentityResolution_ = useIdentityResoltuion;
    engine.useIdentityResolution(useIdentityResoltuion);
  }

  inline std::string getEngineName()
  {
    return engine.getCurrentAEName();
  }
  inline void pause()
  {
    processing_mutex_.lock();
    pause_ = !pause_;
    processing_mutex_.unlock();
  }

  inline void setLowLvlPipeline(std::vector<std::string> llp)
  {
    lowLvlPipeline_.assign(llp.begin(), llp.end());
  }

  //reset the pipeline in the AE;
  bool resetAE(std::string);

private:
  /* *
   * returns true if the planed Pipeline is not a subset of the lowLvl Pipeline
   * */
  bool subsetOfLowLvl(const std::vector<std::string> &plannedPipeline);



};

#endif // RSCONTROLEDAEMANAGER_H
