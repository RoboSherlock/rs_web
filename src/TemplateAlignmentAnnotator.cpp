#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/shot_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>


//RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
#include <rs/DrawingAnnotator.h>

#include <ros/package.h>

#include <rs_kbreasoning/TemplateAlignment.h>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

#include <tf_conversions/tf_eigen.h>

using namespace uima;


class TemplateAlignmentAnnotator : public DrawingAnnotator
{
private:
  std::string packagePath;

  cv::Mat dispImg;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dispCloud;
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> transfResults_;

  tf::StampedTransform camToWorld, worldToCam;

public:

  TemplateAlignmentAnnotator(): DrawingAnnotator(__func__)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    packagePath = ros::package::getPath("rs_resources");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process start");
    rs::SceneCas cas(tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_COLOR_IMAGE_HD, dispImg);
    dispCloud = cloud_ptr;

    rs::Query qs = rs::create<rs::Query>(tcas);
    std::string jsonQuery;
    if(cas.getFS("QUERY", qs))
    {
      jsonQuery = qs.asJson();
      outWarn("json query: " << qs.asJson());
    }

    rapidjson::Document doc;
    doc.Parse(jsonQuery.c_str());
    std::string templateToFit;
    if(doc.HasMember("CAD-MODEL"))
    {
      templateToFit = doc["CAD-MODEL"].GetString();
      if(templateToFit == "")
      {
        outError("No model name defined");
        return UIMA_ERR_NONE;
      }
    }

    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);
    transfResults_.clear();

    camToWorld.setIdentity();
    if(scene.viewPoint.has())
    {
      rs::conversion::from(scene.viewPoint.get(), camToWorld);
    }
    else
    {
      outInfo("No camera to world transformation!!!");
    }
    worldToCam = tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);

    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
      if(!cluster.points.has())
      {
        continue;
      }
      std::vector<rs::Detection> detections;
      cluster.annotations.filter(detections);
      if(detections.empty())
      {
        continue;
      }
      bool foundObjectForTemplate = false;

      for(auto d : detections)
      {
        if(d.name() == templateToFit)
        {
          foundObjectForTemplate = true;
          break;
        }
      }
      if(!foundObjectForTemplate)
      {
        continue;
      }
      //Fit result of drill using local features/template matching
      FeatureCloud object_template;
      std::string templateCloudPath= packagePath + "/objects_dataset/cad_models/" + templateToFit + "/" + templateToFit + ".pcd";
      outInfo("Template cloud name: "<<templateCloudPath);
      object_template.loadInputCloud(templateCloudPath);

      pcl::PointIndicesPtr indices(new pcl::PointIndices());
      rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *indices);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

      pcl::ExtractIndices<pcl::PointXYZRGBA> ei;
      ei.setInputCloud(cloud_ptr);
      ei.setIndices(indices);
      ei.filter(*cluster_cloud);

      outInfo("Before SOR filter: " << cluster_cloud->points.size());
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
      sor.setInputCloud(cluster_cloud);
      sor.setMeanK(100);
      sor.setStddevMulThresh(1.0);
      sor.filter(*cluster_cloud);
      outInfo("After SOR filter: " << cluster_cloud->points.size());

      // ... and downsampling the point cloud
      const float voxel_grid_size = 0.005f;
      pcl::VoxelGrid<pcl::PointXYZRGBA> vox_grid;
      vox_grid.setInputCloud(cluster_cloud);
      vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
      vox_grid.filter(*tempCloud);

      TemplateAlignment template_align;
      FeatureCloud target_cloud;
      target_cloud.setInputCloud(tempCloud);


      template_align.addTemplateCloud(object_template);
      template_align.setTargetCloud(target_cloud);
      TemplateAlignment::Result best_alignment;
      template_align.findBestAlignment(best_alignment);


      Eigen::Matrix3f rotation  =  best_alignment.final_transformation.block<3, 3>(0, 0);
      Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
      outInfo("=======================");
      printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
      printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
      printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
      printf("\n");
      printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
      outInfo("=======================");

      tf::Stamped<tf::Pose> poseCam;
      tf::Matrix3x3 tfRotation;
      tf::Vector3 tfVector;
      tf::matrixEigenToTF(rotation.cast<double>(),tfRotation);
      tf::vectorEigenToTF(translation.cast<double>(),tfVector);
      poseCam.setBasis(tfRotation);
      poseCam.setOrigin(tfVector);
      poseCam.frame_id_ = camToWorld.child_frame_id_;
      poseCam.stamp_ = camToWorld.stamp_;


      tf::Stamped<tf::Pose> poseWorld(camToWorld*poseCam, camToWorld.stamp_, camToWorld.frame_id_);

      rs::PoseAnnotation poseAnnotation = rs::create<rs::PoseAnnotation>(tcas);
      poseAnnotation.source.set("TemplateAlignment");
      poseAnnotation.camera.set(rs::conversion::to(tcas,poseCam));
      poseAnnotation.world.set(rs::conversion::to(tcas,poseWorld));

      cluster.annotations.append(poseAnnotation);
      transfResults_.push_back(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>()));
      pcl::transformPointCloud(*object_template.getPointCloud(), *transfResults_.back(), best_alignment.final_transformation);
    }

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = dispImg.clone();
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, bool firstRun)
  {

    if(firstRun)
    {
      visualizer.addPointCloud(dispCloud, "original");
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "original");
      for(size_t i = 0; i < transfResults_.size(); ++i)
      {
        const cv::Scalar &c =rs::common::cvScalarColors[i % rs::common::numberOfColors];
        const std::string &name = "match" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(transfResults_[i], c.val[2], c.val[1], c.val[0]);
        visualizer.addPointCloud(transfResults_[i], color, name);
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, name);
      }
    }
    else
    {
      visualizer.removeAllPointClouds();
      visualizer.addPointCloud(dispCloud, "original");
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "original");
      for(size_t i = 0; i < transfResults_.size(); ++i)
      {
        const cv::Scalar &c =rs::common::cvScalarColors[i % rs::common::numberOfColors];
        const std::string &name = "match" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(transfResults_[i], c.val[2], c.val[1], c.val[0]);
        visualizer.addPointCloud(transfResults_[i], color, name);
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, name);
      }
    }
  }
};

MAKE_AE(TemplateAlignmentAnnotator)
