#ifndef RSCONTROLEDAEMANAGER_H
#define RSCONTROLEDAEMANAGER_H

#include <rs/utils/RSAnalysisEngineManager.h>

#include <rs_kbreasoning/DesignatorWrapper.h>
#include <rs_kbreasoning/RSControledAnalysisEngine.h>
#include <rs_kbreasoning/KRDefinitions.h>

#include <json_prolog/prolog.h>

#include <designator_integration_msgs/DesignatorCommunication.h>
#include <iai_robosherlock_msgs/SetRSContext.h>


class RSControledAEManager:
  public RSAnalysisEngineManager<RSControledAnalysisEngine>
{

private:

  ros::NodeHandle nh_;
  ros::Publisher desig_pub_;

  const bool waitForServiceCall_;
  std::mutex processing_mutex_;
public:
  RSControledAEManager(const bool useVisualizer, const std::string &savePath,
                       const bool &waitForServiceCall, ros::NodeHandle n):
    RSAnalysisEngineManager(useVisualizer, savePath),
    nh_(n),
    waitForServiceCall_(waitForServiceCall)
  {
    desig_pub_ = nh_.advertise<designator_integration_msgs::DesignatorResponse>(std::string("result_advertiser"), 5);
  }
//  ~RSControledAEManager()
//  {
//    uima::ResourceManager::deleteInstance();
//  }

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

  /* brief
   * run the AEs in the manager
   */
  void run();

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
