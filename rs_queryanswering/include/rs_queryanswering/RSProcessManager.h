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


//TODO: Make this the ROS communication interface class
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
                   const bool &waitForServiceCall, const bool useCWAssumption, ros::NodeHandle n);

  ~RSProcessManager();

  void init(std::string &xmlFile, std::string configFile);

  void run();

  void stop();



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

  //reset the pipeline in the AE;
  bool resetAE(std::string);

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

private:
  /* *
   * returns true if the planed Pipeline is not a subset of the lowLvl Pipeline
   * why is this not in the Controlled AE?...MOVE
   * */
  bool subsetOfLowLvl(const std::vector<std::string> &plannedPipeline);



};

#endif // RSCONTROLEDAEMANAGER_H
