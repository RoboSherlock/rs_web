#ifndef RSCONTROLEDAEMANAGER_H
#define RSCONTROLEDAEMANAGER_H

#include <rs/utils/RSAnalysisEngineManager.h>
#include <rs_kbreasoning/RSControledAnalysisEngine.h>
#include <designator_integration_msgs/DesignatorResponse.h>

class RSControledAEManager:
    public RSAnalysisEngineManager<RSControledAnalysisEngine>
  {
private:
  ros::NodeHandle nh_;
  ros::Publisher desig_pub_;

  bool waitForServiceCall_;
public:
    RSControledAEManager(const bool useVisualizer, const std::string &savePath,
                         const bool &waitForServiceCall, ros::NodeHandle n):
      RSAnalysisEngineManager(useVisualizer, savePath),
      nh_(n),
      waitForServiceCall_(waitForServiceCall)
    {
      desig_pub_ = nh_.advertise<designator_integration_msgs::DesignatorResponse>(std::string("result_advertiser"), 5);
    }
  };

#endif // RSCONTROLEDAEMANAGER_H
