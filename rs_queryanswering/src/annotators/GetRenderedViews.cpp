#include <uima/api.hpp>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/io/UnrealVisionBridge.h>
#include <rs/utils/common.h>
#include <rs/io/TFBroadcasterWrapper.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <iai_robosherlock_msgs/UpdateObjects.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

#include<random>
#include<cmath>
#include<chrono>

#include <visualization_msgs/MarkerArray.h>
#include <resource_retriever/retriever.h>

using namespace uima;

class GetRenderedViews : public Annotator
{

private:

  std::map<std::string, std::string> uNameMapping =
  {
    {"cup", "SM_CupEcoOrange_2"},
    {"bowl", "SM_Bowl_8"},
    {"cereal", "SM_KoellnMuesliKnusperHonigNussNew_2"},
    {"spoon", "SM_Spoon_Dessert9_2"}
  };

  ros::ServiceClient client_;
  ros::NodeHandle nh_;

  UnrealVisionBridge *unrealBridge_;

  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;
  cv::Mat object_;

  std::thread thread_;
  TFBroadcasterWrapper broadCasterObject_;
  ros::Publisher marker_pub_, marker_pu2b_;

public:
  GetRenderedViews(): nh_("~"), it_(nh_)
  {
    client_ = nh_.serviceClient<iai_robosherlock_msgs::UpdateObjects>("/update_objects");
    std::string configFile = ros::package::getPath("robosherlock") + "/config/config_unreal_vision.ini";
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(configFile, pt);
    unrealBridge_ = new UnrealVisionBridge(pt);
    image_pub_ = it_.advertise("rendered_image", 5, false);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
    marker_pu2b_ = nh_.advertise<visualization_msgs::MarkerArray>("markers_ROS", 1, true);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    outInfo("");
    thread_ = std::thread(&TFBroadcasterWrapper::run, &broadCasterObject_);
    outInfo("");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    delete unrealBridge_;
    return UIMA_ERR_NONE;
  }

  bool spawnCameraInUnreal(rs::Scene &scene, tf::Vector3 centroid)
  {

    tf::StampedTransform mapToCam;
    rs::conversion::from(scene.viewPoint.get(), mapToCam);

    float radius = centroid.length();
    pcl::PointCloud<pcl::PointXYZ>::Ptr sphereCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(float theta = M_PI_2 + M_PI / 3; theta < M_PI - 2 * (M_PI / 18); theta += M_PI / 10)
    {
      for(float phi = -M_PI; phi < M_PI; phi += M_PI_2 / 15)
      {
        pcl::PointXYZ p;
        p.x =  radius / 2 * sin(theta) * cos(phi);
        p.y =  radius / 2 * sin(phi) * sin(theta);
        p.z =  radius * cos(theta) + centroid[2];
        sphereCloud->push_back(p);
      }
    }

    sphereCloud->width = sphereCloud->points.size();
    sphereCloud->height = 1;
    outWarn("Sphere points size: " << sphereCloud->points.size());
    pcl::io::savePCDFileASCII("sphere.pcd", *sphereCloud);

    srand(time(NULL));

    /* generate secret number between 1 and 10: */
    int randomPointIdx = rand() % sphereCloud->size() + 1;
    outWarn("Random index: " << randomPointIdx);

    pcl::PointXYZ pt = sphereCloud->points[randomPointIdx];
    tf::StampedTransform camToPoint, camToCenter;

    camToPoint.child_frame_id_ = "view_point";
    camToPoint.frame_id_ = mapToCam.child_frame_id_;//kinect
    camToPoint.setOrigin(tf::Vector3(pt.x, pt.y, pt.z));

    camToCenter.child_frame_id_ = "center_point";
    camToCenter.frame_id_ = mapToCam.child_frame_id_;//kinect
    camToCenter.setOrigin(centroid);
    camToCenter.setRotation(tf::Quaternion(0, 0, 0, 1));

    tf::Vector3 newZ(tf::Vector3(centroid[0] - pt.x, centroid[1] - pt.y, centroid[2] - pt.z));
    tf::Vector3 XAxes(1.0, 0.0, 0.0);
    //    tf::Vector3 newY = newZ.cross(XAxes);
    //    tf::Vector3 newX = newZ.cross(newY);
    tf::Vector3 newY = XAxes.cross(newZ);
    tf::Vector3 newX = newY.cross(newZ);

    newX.normalize();
    newY.normalize();
    newZ.normalize();

    outInfo(newX.dot(newY) << " " << newX.dot(newZ) << " " << newZ.dot(newY));
    tf::Matrix3x3 rotationMatrix(newX.x(), newX.y(), newX.z(),
                                 newY.x(), newY.y(), newY.z(),
                                 newZ.x(), newZ.y(), newZ.z());

    camToPoint.setBasis(rotationMatrix.transpose());
    outInfo(camToPoint.getRotation().getX() << " " << camToPoint.getRotation().getY() << " " << camToPoint.getRotation().getZ() << " " << camToPoint.getRotation().getW());

    tf::Quaternion rotationAsQuat = camToPoint.getRotation().normalize();
    tf::Quaternion rot_hack;
    //Y X Z
    rot_hack.setEuler(0.0, 0.0, -M_PI);
    camToPoint.setRotation(rotationAsQuat * rot_hack);

    broadCasterObject_.clear();
    broadCasterObject_.addTransform(camToPoint);
    broadCasterObject_.addTransform(camToCenter);

    tf::Transform mapToPoint = mapToCam * camToPoint;
    broadCasterObject_.clear();
    broadCasterObject_.addTransform(camToPoint);
    broadCasterObject_.addTransform(camToCenter);

    tf::Stamped<tf::Pose> poseStamped;
    poseStamped.setOrigin(mapToPoint.getOrigin());

    tf::Quaternion rot(0, 0, 0, 1);


    //    rot_hack.setEuler(M_PI, M_PI / 2, 0.0);
    //    rot = rot * rot_hack;
    rot_hack.setEuler(0.0, 0.0, M_PI);
    rot = rot * rot_hack;
    rot_hack.setEuler(M_PI_4, 0.0, 0.0);
    rot = rot * rot_hack;
    poseStamped.setRotation(rot /** mapToPoint.getRotation()*/);

    iai_robosherlock_msgs::UpdateObjects updateCameraPose;
    updateCameraPose.request.name = "ARGBDCamera";
    tf::poseStampedTFToMsg(poseStamped, updateCameraPose.request.pose_stamped);
    if(client_.call(updateCameraPose))
    {
      outInfo("Success!");
      return true;
    }
    else
    {
      outInfo("We failed!");
      return false;
    }
  }

  void countObjInliers(cv::Vec3b objectColor, std::vector<cv::Point> &points)
  {

    int totalX = 0,
        totalY = 0;
    #pragma omp parallel
    for(size_t r = 0; r < (size_t)object_.rows; ++r)
    {
      const cv::Vec3b *itC = object_.ptr<cv::Vec3b>(r);
      for(size_t c = 0; c < (size_t)object_.cols; ++c, ++itC)
      {
        if(*itC == objectColor)
        {
          #pragma omp critical
          {
            totalX += c;
            totalY += r;
            points.push_back(cv::Point(c, r));
          }
        }
      }
    }

    if(!points.empty())
    {
      int centerX = totalX / points.size(),
          centerY = totalY / points.size();
      if(centerX < 20 || centerX > object_.cols - 20 ||
         centerY < 20 || centerY > object_.rows - 20)
      {
        points.clear();
      }
      outInfo("Obj too on the side");
    }
  }

  //  void publishMarkers(const std::vector<rs::Object>  &objects)
  //  {
  //    visualization_msgs::MarkerArray markers,markersROS;
  //    int idx = 0;
  //    for(rs::Object obj : objects)
  //    {
  //      visualization_msgs::Marker marker;
  //      marker.header.frame_id = "map";
  //      marker.header.stamp = ros::Time::now();
  //      marker.ns = "rs";
  //      marker.id = idx++;
  //      marker.action = visualization_msgs::Marker::ADD;

  //      std::vector<rs::Shape> shapes;
  //      std::vector<rs::Detection> detections;
  //      obj.annotations.filter(shapes);
  //      obj.annotations.filter(detections);

  //      if(detections.empty())
  //      {
  //        continue;
  //      }
  //      outInfo("publishing marker");
  //      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  //      std::string name = detections[0].name();
  //      if(name != "koelln_muesli_knusper_honig_nuss")
  //      {
  //        continue;
  //      }
  //      tf::Stamped<tf::Pose> pose ;
  //      pose.setOrigin(tf::Vector3(0,0,0));
  //      tf::Quaternion rotation (0,0,0,1);
  //      pose.setRotation(rotation);

  //      tf::poseTFToMsg(pose,marker.pose);

  //      resource_retriever::Retriever r;
  //      std::string mesh_resource = "package://rs_resources/objects_dataset/cad_models/" + name + "/" + name + ".dae";
  //      outInfo("publishing"<<mesh_resource);
  //      try
  //      {
  //        r.get(mesh_resource);
  //        marker.mesh_resource = mesh_resource;
  //        marker.mesh_use_embedded_materials = true;
  //        marker.scale.x = 1.0f;
  //        marker.scale.y = 1.0f;
  //        marker.scale.z = 1.0f;
  //        marker.color.a = 1.0;
  //      }
  //      catch(resource_retriever::Exception &e)
  //      {
  //        outWarn(e.what());
  //      }
  //      markers.markers.push_back(marker);

  //    }
  //    outInfo("Publishgin " << markers.markers.size() << " markers");
  //    marker_pub_.publish(markers);

  ////    marker_pu2b_.publish(markersROS);
  //  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);

    if(planes.empty())
    {
      return UIMA_ERR_NONE;
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cas.get(VIEW_CLOUD, *cloud);
    pcl::io::savePCDFileASCII("cloud.pcd", *cloud);
    rs::Plane &plane  = planes[0];
    std::vector<int> planeInliers = plane.inliers.get();
    Eigen::Vector4f temp;
    pcl::compute3DCentroid(*cloud, planeInliers, temp);
    tf::Vector3 centroid(temp[0], temp[1], temp[2]);

    if(!spawnCameraInUnreal(scene, centroid))
    {
      return UIMA_ERR_NONE;
    }

    rs::Query qs = rs::create<rs::Query>(tcas);
    std::string jsonQuery;
    if(cas.getFS("QUERY", qs))
    {
      jsonQuery = qs.asJson();
      outWarn("json query: " <<jsonQuery);
    }
    else
    {
      outWarn("No Query, skipping exec");
      return UIMA_ERR_NONE;
    }
    rapidjson::Document doc;
    doc.Parse(jsonQuery.c_str());

    std::string howToHighligh;
    if(doc.HasMember("render"))
    {
      howToHighligh = doc["render"].GetString();
    }
    outWarn(howToHighligh);
    int count = 0;
    while(!unrealBridge_->newData() && count < 5)
    {
      count++;
      outInfo("Waiting for Unreal");
      usleep(100);
    }
    if(count < 5)
    {
      unrealBridge_->setData(tcas);

      cv::Mat rgb;
      sensor_msgs::CameraInfo cam_info;
      cas.get(VIEW_OBJECT_IMAGE, object_);
      cas.get(VIEW_COLOR_IMAGE, rgb);
      cas.get(VIEW_CAMERA_INFO, cam_info);

      std::map<std::string, cv::Vec3b> objectMap;
      cas.get(VIEW_OBJECT_MAP, objectMap);
//      for(auto obj : objectMap)
//      {
//        outInfo(obj.first);
//      }
      std::vector<cv::Point> points;
      //      if(howToHighligh != "")
      //      {

      //        countObjInliers(objectMap[uNameMapping[howToHighligh]], points);
      //      }
      //      outInfo("Inliers for object: " << points.size());

      cv_bridge::CvImage outImgMsgs;
      outImgMsgs.header = cam_info.header;
      outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
      outInfo("SHOWING AS RESULT: "<<howToHighligh);
      if(howToHighligh == "mask")
      {
        outImgMsgs.image = object_;
      }
      else
      {
        outImgMsgs.image = rgb;
      }


      image_pub_.publish(outImgMsgs.toImageMsg());
      image_pub_.publish(outImgMsgs.toImageMsg());
      /*image_pub_.publish(outImgMsgs.toImageMsg());
       image_pub_.publish(outImgMsgs.toImageMsg())*/;
    }
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(GetRenderedViews)
