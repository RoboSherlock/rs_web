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


#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>

#include <std_srvs/Trigger.h>

//TODO: Make this the ROS communication interface class
class RSProcessManager
{

private:

  RSControledAnalysisEngine engine_;
  RSControledAnalysisEngine inspectionEngine_;

  PrologInterface *prologInterface;

  ros::NodeHandle nh_;
  ros::Publisher desig_pub_;
  ros::ServiceServer service, singleService, setContextService, jsonService;

  //specific to thorin
  ros::ServiceClient triggerKRPoseUpdate_;
  std::map<std::string,std::string> thorinObjects_;
  std::vector<std::string> seenObjects_;


  const bool waitForServiceCall_;
  const bool useVisualizer_;
  const bool useCWAssumption_;
  bool withJsonProlog_;
  bool useIdentityResolution_;
  bool pause_;
  bool inspectFromAR_;


  std::mutex processing_mutex_;

  rs::Visualizer visualizer_;

  std::string configFile_;
  std::vector<std::string> closedWorldAssumption_;
  std::vector<std::string> lowLvlPipeline_;

public:

  RSProcessManager(const bool useVisualizer, const std::string &savePath,
                   const bool &waitForServiceCall, const bool useCWAssumption, ros::NodeHandle n);

  ~RSProcessManager();

  void init(std::string &xmlFile, std::string configFile_);

  void run();

  void stop();

  //again...thorin specific
  void getDemoObjects();
  std::string toJson(const tf::StampedTransform &pose, std::string OID, std::string type);
  std::string getObjectByID(std::string OID, std::string type);


  bool resetAECallback(iai_robosherlock_msgs::SetRSContext::Request &req,
                       iai_robosherlock_msgs::SetRSContext::Response &res);

  bool designatorSingleSolutionCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
                                        designator_integration_msgs::DesignatorCommunication::Response &res);

  bool designatorCallbackLogic(designator_integration_msgs::DesignatorCommunication::Request &req,
                               designator_integration_msgs::DesignatorCommunication::Response &res);

  bool jsonQueryCallback(iai_robosherlock_msgs::RSQueryService::Request &req,
                         iai_robosherlock_msgs::RSQueryService::Response &res);

  bool handleQuery(designator_integration::Designator *req,
                   std::vector<designator_integration::Designator> &resp);

  //TODO: move to Designator wrapper or somewhere else
  void filterResults(designator_integration::Designator &requestDesignator,
                     std::vector<designator_integration::Designator> &resultDesignators,
                     std::vector<designator_integration::Designator> &filteredResponse,
                     std::vector<bool> &designatorsToKeep,
                     const std::string superclass);

  //reset the pipeline in the AE;
  bool resetAE(std::string);

  //set the AE for inspection tasks (allows for a different parametrization of components)
  void setInspectionAE(std::string);


  inline void setUseIdentityResolution(bool useIdentityResoltuion)
  {
    useIdentityResolution_ = useIdentityResoltuion;
    engine_.useIdentityResolution(useIdentityResoltuion);
  }

  inline void setUseJsonPrologInterface(bool useJson)
  {
    withJsonProlog_ = useJson;
  }

  inline void setInspectFromAR(bool b)
  {
    inspectFromAR_ = b;
  }

  inline std::string getEngineName()
  {
    return engine_.getCurrentAEName();
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
