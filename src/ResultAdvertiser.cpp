/*
 * Copyright (c) 2011, Ferenc Balint-Benczedi <balintbe@tzi.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <uima/api.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <actionlib/server/simple_action_server.h>
#include <iai_robosherlock_actions/SimplePerceiveObjectAction.h>
#include <iai_robosherlock_actions/PerceivedObjects.h>

#include <designators/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

#include <rs_demos/utils/DesignatorWrapper.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <thread>
#include <atomic>
#include <mutex>

//#undef OUT_LEVEL
//#define OUT_LEVEL OUT_LEVEL_DEBUG

using namespace designator_integration;

using namespace uima;

class ResultAdvertiser : public Annotator
{
private:
  ros::NodeHandle nh_;
  ros::Publisher object_pub;
  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;

  std::thread thread;
  rs::DesignatorWrapper dw_;
  rs::DesignatorWrapper::DesignatorProcessMode mode;
  bool filter_results_;
  std::vector<cv::Scalar> colors;

  enum COLORS
  {
    RED = 0,
    YELLOW,
    GREEN,
    CYAN,
    BLUE,
    MAGENTA,
    WHITE,
    BLACK,
    GREY,
    COUNT
  };
  std::vector<cv::Vec3b> colorsVec;
  uint64_t now;

public:
  ResultAdvertiser() : nh_("~"), it_(nh_), filter_results_(false)
  {

    object_pub = nh_.advertise<iai_robosherlock_actions::PerceivedObjects>(std::string("object_perceived"), 5);

    image_pub_ = it_.advertise("result_image", 1, true);
    mode = rs::DesignatorWrapper::CLUSTER;
    colors.resize(9);
    colors[0]     = CV_RGB(255, 0, 0);
    colors[1]  = CV_RGB(255, 255, 0);
    colors[2]   = CV_RGB(0, 255, 0);
    colors[3]    = CV_RGB(0, 255, 255);
    colors[4]    = CV_RGB(0, 0, 255);
    colors[5] = CV_RGB(255, 0, 255);
    colors[6]   = CV_RGB(255, 255, 255);
    colors[7]   = CV_RGB(0, 0, 0);
    colors[8]    = CV_RGB(127, 127, 127);



    colorsVec.resize(COUNT);
    colorsVec[RED]     = cv::Vec3b(255, 0, 0);
    colorsVec[YELLOW]  = cv::Vec3b(255, 255, 0);
    colorsVec[GREEN]   = cv::Vec3b(0, 255, 0);
    colorsVec[CYAN]    = cv::Vec3b(0, 255, 255);
    colorsVec[BLUE]    = cv::Vec3b(0, 0, 255);
    colorsVec[MAGENTA] = cv::Vec3b(255, 0, 255);
    colorsVec[WHITE]   = cv::Vec3b(255, 255, 255);
    colorsVec[BLACK]   = cv::Vec3b(0, 0, 0);
    colorsVec[GREY]    = cv::Vec3b(127, 127, 127);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("resultMode"))
    {
      std::string sMode;
      ctx.extractValue("resultMode", sMode);

      if(sMode == "Cluster")
      {
        mode = rs::DesignatorWrapper::CLUSTER;
      }
      else if(sMode == "Object")
      {
        mode = rs::DesignatorWrapper::OBJECT;
      }
    }
    if(ctx.isParameterDefined("filter_results"))
    {
      ctx.extractValue("filter_results", filter_results_);
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId typeSystemInit(TypeSystem const &type_system)
  {
    outInfo("typeSystemInit");
    return  UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    thread.join();
    return  UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process begins");
    cv::Mat rgb;
    cv_bridge::CvImage outImgMsgs;
    dw_.setCAS(&tcas);
    dw_.setMode(mode);

    rs::SceneCas cas(tcas);
    rs::Scene scene(cas.getScene());
    sensor_msgs::CameraInfo cam_info;
    cas.get(VIEW_CAMERA_INFO, cam_info);
    cas.get(VIEW_COLOR_IMAGE, rgb);
    designator_integration_msgs::DesignatorResponse res;
    iai_robosherlock_actions::PerceivedObjects objects;

    outInfo("filtering the results based on the designator request");
    std::vector<designator_integration::Designator> resultDesignators;
    dw_.getObjectDesignators(resultDesignators);
    std::vector<bool> keep_designator;
    keep_designator.resize(resultDesignators.size(), true);

    designator_integration::Designator requestDesignator = *rs::DesignatorWrapper::req_designator;
    //      requestDesignator.printDesignator();
    std::list<KeyValuePair *> requested_kvps = requestDesignator.description();


    for(std::list<KeyValuePair *>::iterator it = requested_kvps.begin(); it != requested_kvps.end(); ++it)
    {
      KeyValuePair req_kvp = **it;
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
        designator_integration::Designator &resDesig = resultDesignators[i];
        //          resDesig.printDesignator();
        //and here come the hacks
        std::vector<KeyValuePair *> resultsForRequestedKey;
        KeyValuePair *childRequestedKey = NULL;
        if(resDesig.childForKey("CLUSTERID") != NULL)
        {
          if(req_kvp.key() == "SIZE") //size is nested get it from the bounding box..bad design
          {
            childRequestedKey = resDesig.childForKey("BOUNDINGBOX")->childForKey("SIZE");
            resultsForRequestedKey.push_back(childRequestedKey);
          }
          else if(req_kvp.key() == "SHAPE") //there can be multiple shapes and these are not nested
          {
            std::list<KeyValuePair *> resKvPs = resDesig.description();
            for(std::list<KeyValuePair *>::iterator it2 = resKvPs.begin(); it2 != resKvPs.end(); ++it2)
            {
              KeyValuePair res_kvp = **it2;
              if(res_kvp.key() == "SHAPE")
              {
                resultsForRequestedKey.push_back(*it2);
              }
            }
          }
          else
          {
            resultsForRequestedKey.push_back(resDesig.childForKey(req_kvp.key()));
          }
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
                std::list<KeyValuePair *> colorRatioPairs = resultsForRequestedKey[j]->children();
                for(auto iter = colorRatioPairs.begin(); iter != colorRatioPairs.end(); ++iter)
                {
                  KeyValuePair colorRatioKvp = **iter;
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
              //another nested kv-p...we need a new interface...this one sux
              if(resultsForRequestedKey[j]->key() == "DETECTION")
              {
                std::list<KeyValuePair *> childrenPairs = resultsForRequestedKey[j]->children();
                for(auto iter = childrenPairs.begin(); iter != childrenPairs.end(); ++iter)
                {
                  KeyValuePair childrenPair = **iter;
                  if(childrenPair.key() == "TYPE")
                  {
                    if(strcasecmp(childrenPair.stringValue().c_str(), req_kvp.stringValue().c_str()) == 0 || req_kvp.stringValue() == "")
                    {
                      ok = true;
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
        outInfo("Cluster: " << i << " is a match");
        res.designators.push_back(resultDesignators[i].serializeToMessage());
      }
    }

    drawResulstOnImage(rgb, keep_designator, scene, resultDesignators);
    outImgMsgs.header = cam_info.header;
    outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
    outImgMsgs.image = rgb;

    object_pub.publish(objects);
    image_pub_.publish(outImgMsgs.toImageMsg());

    //    outInfo("took: " << clock.getTime() << " ms.");
    return  UIMA_ERR_NONE;
  }

private:
  void drawResulstOnImage(cv::Mat &rgb, const std::vector<bool> &filter,
                          rs::Scene &scene,
                          std::vector<designator_integration::Designator> &resultDesignators)
  {
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);
    //very hacky for now
    for(int i=0;i<clusters.size();++i)
    {
      rs::Cluster &cluster = clusters[i];
      std::vector<rs::ClusterPart> parts;
      cluster.annotations.filter(parts);
      if(parts.empty())
      {
        continue;
      }
      for(int pIdx = 0;pIdx<parts.size();++pIdx)
      {
        rs::ClusterPart &part = parts[pIdx];
        pcl::PointIndices indices;
        rs::conversion::from(part.indices(),indices);
        for (int iIdx = 0;iIdx<indices.indices.size();++iIdx)
        {
          int idx = indices.indices[iIdx];
          rgb.at<cv::Vec3b>(cv::Point(idx % 640, idx / 640)) = colorsVec[pIdx%9];
        }
      }
    }

    for(int i = 0; i < filter.size(); ++i)
    { 

      if(!filter[i])
      {
        continue;
      }

      designator_integration::Designator &desig = resultDesignators[i];
      KeyValuePair *clusterId = desig.childForKey("CLUSTERID");
      if(clusterId != NULL)
      {
        int idx = atoi(clusterId->stringValue().c_str());
        outInfo("Draw cluster: " << idx);
        rs::ImageROI roi = clusters[idx].rois();
        cv::Rect cvRoi;
        rs::conversion::from(roi.roi(), cvRoi);
        cv::rectangle(rgb, cvRoi, colors[idx % 9], 1.5);
        std::stringstream clusterName;
        clusterName << "clusterID_" << idx;
        cv::putText(rgb, clusterName.str(), cv::Point(cvRoi.x + 10, cvRoi.y - 10), cv::FONT_HERSHEY_COMPLEX, 0.7, colors[idx % 9]);
      }
      KeyValuePair *handleKvp = desig.childForKey("type");
      if(handleKvp != NULL)
      {
        //color the pixels of the handle
        std::vector<rs::HandleAnnotation> handles;
        scene.annotations.filter(handles);
        for(int i = 0; i < handles.size(); ++i)
        {
          pcl::PointIndices indices;
          rs::conversion::from(handles[i].indices(), indices);
          outInfo("Number of inliers in handle " << i << ": " << indices.indices.size());
          for(int j = 0; j < indices.indices.size(); ++j)
          {
            int idx = indices.indices[j];
            cv::Vec3b new_color;
            new_color[0] = 0;
            new_color[0] = 0;
            new_color[0] = 255;
            rgb.at<cv::Vec3b>(cv::Point(idx % 640, idx / 640)) = new_color;
          }
        }
      }

    }
  }



};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ResultAdvertiser)
