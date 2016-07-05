#ifndef RSCONTROLEDAEMANAGER_H
#define RSCONTROLEDAEMANAGER_H

#include <rs/utils/RSAnalysisEngineManager.h>

#include <rs_kbreasoning/RSControledAnalysisEngine.h>
#include <rs_kbreasoning/KRDefinitions.h>
#include <designator_integration_msgs/DesignatorResponse.h>

class RSControledAEManager:
  public RSAnalysisEngineManager<RSControledAnalysisEngine>
{

private:
  ros::NodeHandle nh_;
  ros::Publisher desig_pub_;

  const bool waitForServiceCall_;


public:
  RSControledAEManager(const bool useVisualizer, const std::string &savePath,
                       const bool &waitForServiceCall, ros::NodeHandle n):
    RSAnalysisEngineManager(useVisualizer, savePath),
    nh_(n),
    waitForServiceCall_(waitForServiceCall)
  {
    desig_pub_ = nh_.advertise<designator_integration_msgs::DesignatorResponse>(std::string("result_advertiser"), 5);
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
};

#endif // RSCONTROLEDAEMANAGER_H
