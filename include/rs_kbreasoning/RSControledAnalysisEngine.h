#ifndef RSCONTROLEDANALYSISENGINE_H
#define RSCONTROLEDANALYSISENGINE_H

#include <rs/utils/common.h>
#include <rs/utils/RSAnalysisEngine.h>
#include <rs/utils/RSPipelineManager.h>
#include <rs/scene_cas.h>

#include <designator_integration_msgs/DesignatorResponse.h>
#include <designator_integration_msgs/DesignatorRequest.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <rs_kbreasoning/DesignatorWrapper.h>

/*
 * struct for passing vital information from a query to individual annotators
 *e.g. the timestamp for CollectionReader, or location for the RegionFilter
 *
*/
struct RSQuery
{
  uint64_t timestamp = 0;
  std::string location = "";
  std::string objToInspect = "";
};

class RSControledAnalysisEngine: public RSAnalysisEngine
{

private:
  RSPipelineManager *rspm;
  std::string currentAEName;
  std::vector<std::string> next_pipeline_order;
  boost::shared_ptr<std::mutex> process_mutex;

  ros::NodeHandle nh_;
  ros::Publisher base64ImgPub;
  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;

public:

  RSControledAnalysisEngine();
  ~RSControledAnalysisEngine();

  /*set the next order of AEs to be executed*/
  void setNextPipeline(std::vector<std::string> l);

  /*get the next order of AEs to be executed*/
  std::vector<std::string> &getNextPipeline();

  void applyNextPipeline();


  void resetPipelineOrdering()
  {
    if(rspm)
    {
      rspm->resetPipelineOrdering();
    }
  }

  std::string getCurrentAEName()
  {
    return currentAEName;
  }

  bool defaultPipelineEnabled()
  {
    if(rspm)
    {
      return rspm->use_default_pipeline;
    }
    return false;
  }

  void init(const std::string &file);

  void process();

  void process(designator_integration_msgs::DesignatorResponse &designator_response,
               RSQuery *q = NULL);

  void process(bool reset_pipeline_after_process,
               designator_integration_msgs::DesignatorResponse &designator_response);

  // Call process() and
  // decide if the pipeline should be reset or not
  void process(bool reset_pipeline_after_process);

  // Define a pipeline that should be executed,
  // process(reset_pipeline_after_process) everything and
  // decide if the pipeline should be reset or not
  void process(std::vector<std::string> annotators,
               bool reset_pipeline_after_process,
               designator_integration_msgs::DesignatorResponse &designator_response,
               RSQuery *query = NULL);

  // Define a pipeline that should be executed,
  // process(reset_pipeline_after_process) everything and
  // decide if the pipeline should be reset or not
  void process(std::vector<std::string> annotators, bool reset_pipeline_after_process);

  //draw results on an image
  void drawResulstOnImage(const std::vector<bool> &filter,
                          const std::vector<designator_integration::Designator> &resultDesignators);
};
#endif // RSCONTROLEDANALYSISENGINE_H
