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
#include <pcl_ros/point_cloud.h>

#include <rs_kbreasoning/DesignatorWrapper.h>

/*
 * struct for passing vital information from a query to individual annotators
 *e.g. the timestamp for CollectionReader, or location for the RegionFilter;
 *For now it's hacky consider changin interface to an wesom json thingy instead of
 * desig_integration, and then just push the whole query to the cas and let
 * individual annotators look it query contains something interesting for them
*/
struct RSQuery
{
  uint64_t timestamp = 0;
  std::string location = "";
  std::string objToInspect = "";
  std::string ingredient ="";
  std::string asJson="";
};

class RSControledAnalysisEngine: public RSAnalysisEngine
{

private:
  RSPipelineManager *rspm;
  std::string currentAEName;
  std::vector<std::string> next_pipeline_order;
  std::vector<std::string> cwObjects_;
  boost::shared_ptr<std::mutex> process_mutex;

  ros::NodeHandle nh_;
  ros::Publisher base64ImgPub;
  ros::Publisher pc_pub_;
  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;

  bool useIdentityResolution_;

public:

  RSControledAnalysisEngine() : RSAnalysisEngine(),
    rspm(NULL),currentAEName(""),nh_("~"),it_(nh_),useIdentityResolution_(false)
  {
    process_mutex = boost::shared_ptr<std::mutex>(new std::mutex);
    base64ImgPub = nh_.advertise<std_msgs::String>(std::string("image_base64"), 5);
    image_pub_ = it_.advertise("result_image", 1, true);
    pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >("points", 5 );
  }

  ~RSControledAnalysisEngine()
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

  /*set the next order of AEs to be executed*/
  void setNextPipeline(std::vector<std::string> l)
  {
    next_pipeline_order = l;
  }


  /*get the next order of AEs to be executed*/
  inline std::vector<std::string> &getNextPipeline()
  {
    return next_pipeline_order;
  }

  inline void applyNextPipeline()
  {
    if(rspm)
    {
      rspm->setPipelineOrdering(next_pipeline_order);
    }
  }

  inline void resetPipelineOrdering()
  {
    if(rspm)
    {
      rspm->resetPipelineOrdering();
    }
  }

  inline std::string getCurrentAEName()
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

  void init(const std::string &file,const std::vector<std::string> &lowLvLPipeline);

  void setCWAssumption(const std::vector<std::string>&);

  inline void useIdentityResolution(const bool useIDres)
  {
      useIdentityResolution_=useIDres;
  }

  void process();

  void process(std::vector<designator_integration::Designator> &designator_response,
               RSQuery *q = NULL);

  void process(bool reset_pipeline_after_process,
               std::vector<designator_integration::Designator>&designator_response);

  // Call process() and
  // decide if the pipeline should be reset or not
  void process(bool reset_pipeline_after_process);

  // Define a pipeline that should be executed,
  // process(reset_pipeline_after_process) everything and
  // decide if the pipeline should be reset or not
  void process(std::vector<std::string> annotators,
               bool reset_pipeline_after_process,
               std::vector<designator_integration::Designator> &designator_response,
               RSQuery *query = NULL);

  // Define a pipeline that should be executed,
  // process(reset_pipeline_after_process) everything and
  // decide if the pipeline should be reset or not
  void process(std::vector<std::string> annotators, bool reset_pipeline_after_process);

  //draw results on an image
  template <class T>
  void drawResulstOnImage(const std::vector<bool> &filter,
                          const std::vector<designator_integration::Designator> &resultDesignators,
                          designator_integration::Designator &requestDesignator);


};
#endif // RSCONTROLEDANALYSISENGINE_H
