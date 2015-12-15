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

#include <base64.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <actionlib/server/simple_action_server.h>
#include <iai_robosherlock_msgs/SimplePerceiveObjectAction.h>
#include <iai_robosherlock_msgs/PerceivedObjects.h>

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

#include <rs_kbreasoning/DesignatorWrapper.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <thread>
#include <atomic>
#include <mutex>

#include <json_prolog/prolog.h>
#include <rs_kbreasoning/KRDefinitions.h>

//#undef OUT_LEVEL
//#define OUT_LEVEL OUT_LEVEL_DEBUG

using namespace designator_integration;

using namespace uima;

static const std::string base64_chars =
             "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";


static inline bool is_base64(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}


class ResultAdvertiser : public Annotator
{
private:
  ros::NodeHandle nh_;
  ros::Publisher base64Img;
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

    base64Img = nh_.advertise<std_msgs::String>(std::string("image_base64"), 5);

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



  std::string base64_encode(unsigned char const *bytes_to_encode, unsigned int in_len)
  {
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while(in_len--)
    {
      char_array_3[i++] = *(bytes_to_encode++);
      if(i == 3)
      {
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for(i = 0; (i < 4) ; i++)
        {
          ret += base64_chars[char_array_4[i]];
        }
        i = 0;
      }
    }

    if(i)
    {
      for(j = i; j < 3; j++)
      {
        char_array_3[j] = '\0';
      }

      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for(j = 0; (j < i + 1); j++)
      {
        ret += base64_chars[char_array_4[j]];
      }

      while((i++ < 3))
      {
        ret += '=';
      }

    }

    return ret;
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

    outInfo("filtering the results based on the designator request");
    std::vector<designator_integration::Designator> resultDesignators;
    dw_.getObjectDesignators(resultDesignators);
    std::vector<bool> keep_designator;
    keep_designator.resize(resultDesignators.size(), true);

    designator_integration::Designator requestDesignator = *rs::DesignatorWrapper::req_designator;
    //      requestDesignator.printDesignator();
    std::list<KeyValuePair *> requested_kvps = requestDesignator.description();

    std::string superClass = "";

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
      if(req_kvp.key() == "TYPE")
      {
        superClass = req_kvp.stringValue();
      }
      outWarn("superclass is " << superClass);
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
          else if(req_kvp.key() == "LOCATION")
          {
            childRequestedKey = resDesig.childForKey("LOCATION");
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

          else if(req_kvp.key() == "VOLUME")
          {
            childRequestedKey = resDesig.childForKey("VOLUME");
            if(childRequestedKey != NULL)
            {
              resultsForRequestedKey.push_back(childRequestedKey);
            }
            else
            {
              keep_designator[i] = false;
            }
          }

          else if(req_kvp.key() == "CONTAINS")
          {
            childRequestedKey = resDesig.childForKey("CONTAINS");
            if(childRequestedKey != NULL)
            {
              resultsForRequestedKey.push_back(childRequestedKey);
            }
            else
            {
              keep_designator[i] = false;
            }
          }
          else if(req_kvp.key() == "TYPE")//this shit needed so we don't loose al of our stuff just because all was sent instead of detection
          {
            resultsForRequestedKey.push_back(resDesig.childForKey("DETECTION"));
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
              if(resultsForRequestedKey[j]->key() == "VOLUME")
              {
                float volumeofCurrentObj = resultsForRequestedKey[j]->floatValue();
                outWarn("Volume as a float: " << volumeofCurrentObj);
                if(req_kvp.stringValue() == "")
                {
                  ok = true;
                }
                else
                {
                  float volumeAsked = atof(req_kvp.stringValue().c_str());
                  outWarn("Volume asked as float: " << volumeAsked);
                  if(volumeAsked <= volumeofCurrentObj)
                  {
                    ok = true;
                  }
                }
              }
              if(resultsForRequestedKey[j]->key() == "CONTAINS")
              {
                if(req_kvp.stringValue() == "")
                {
                  ok = true;
                }
                else
                {
                  std::string substanceName = resultsForRequestedKey[j]->childForKey("SUBSTANCE")->stringValue();
                  std::string substanceAsked = req_kvp.stringValue();
                  outWarn("Substance asked : " << substanceAsked);
                  if(strcasecmp(substanceName.c_str(), substanceAsked.c_str()) == 0)
                  {
                    ok = true;
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

                    if(superClass != "")
                    {
                      outWarn("filtering based on JSON required");
                      outWarn("Object looked at: " << childrenPair.stringValue());
                      std::stringstream prologQuery;
                      outWarn("Object should be subclass of: " << superClass);
                      prologQuery<<"owl_subclass_of("<<rs_kbreasoning::krNameMapping[childrenPair.stringValue()]<<",knowrob:'"<<superClass<<"').";
                      outWarn(prologQuery.str());
                      json_prolog::Prolog pl;
                      json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
                      if(bdgs.begin() == bdgs.end())
                      {
                        outInfo(rs_kbreasoning::krNameMapping[childrenPair.stringValue()]<<" IS NOT "<<superClass);
                        ok = false;
                      }
                      else
                      {
                        ok = true;
                      }
                    }

                    else if(strcasecmp(childrenPair.stringValue().c_str(), req_kvp.stringValue().c_str()) == 0 || req_kvp.stringValue() == "")
                    {
                      ok = true;
                    }
                    else if(childrenPair.stringValue() == "bottle_acid" || childrenPair.stringValue() == "bottle_base")
                    {
                      std::string new_name = "bottle";
                      if(strcasecmp(new_name.c_str(), req_kvp.stringValue().c_str()) == 0)
                      {
                        ok = true;
                      }
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

    std::vector<uchar> imageData;
    std::vector<int> params ={CV_IMWRITE_JPEG_QUALITY,90,0};
    cv::imencode(".jpg",rgb,imageData,params);

    std::string encoded = base64_encode(&imageData[0], imageData.size());


    std_msgs::String strMsg;
    strMsg.data = "data:image/jpg;base64,"+encoded;
    base64Img.publish(strMsg);
    image_pub_.publish(outImgMsgs.toImageMsg());

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
    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
      std::vector<rs::ClusterPart> parts;
      cluster.annotations.filter(parts);
      if(parts.empty())
      {
        continue;
      }
      for(int pIdx = 0; pIdx < parts.size(); ++pIdx)
      {
        rs::ClusterPart &part = parts[pIdx];
        pcl::PointIndices indices;
        rs::conversion::from(part.indices(), indices);
        for(int iIdx = 0; iIdx < indices.indices.size(); ++iIdx)
        {
          int idx = indices.indices[iIdx];
          rgb.at<cv::Vec3b>(cv::Point(idx % 640, idx / 640)) = colorsVec[pIdx % 9];
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
        clusterName << "cID_" << idx;
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
