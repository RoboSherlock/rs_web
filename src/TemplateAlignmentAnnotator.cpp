#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>

//RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

#include <ros/package.h>

#include <rs_kbreasoning/TemplateAlignment.h>

using namespace uima;

static const cv::Scalar colors[] =
{
  cv::Scalar(255, 0, 0),
  cv::Scalar(0, 255, 0),
  cv::Scalar(0, 0, 255),
  cv::Scalar(255, 255, 0),
  cv::Scalar(255, 0, 255),
  cv::Scalar(0, 255, 255),
  cv::Scalar(191, 0, 0),
  cv::Scalar(0, 191, 0),
  cv::Scalar(0, 0, 191),
  cv::Scalar(191, 191, 0),
  cv::Scalar(191, 0, 191),
  cv::Scalar(0, 191, 191),
  cv::Scalar(127, 0, 0),
  cv::Scalar(0, 127, 0),
  cv::Scalar(0, 0, 127),
  cv::Scalar(127, 127, 0),
  cv::Scalar(127, 0, 127),
  cv::Scalar(0, 127, 127)
};
static const size_t numberOfColors = sizeof(colors) / sizeof(colors[0]);

class Matcher : public DrawingAnnotator
{
private:
  std::string packagePath;

  cv::Mat dispImg;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dispCloud;
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> transfResults_;
public:

  Matcher(): DrawingAnnotator(__func__)
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

    rs::Scene scene = cas.getScene();
    transfResults_.clear();
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);
    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
      if(!cluster.points.has())
      {
        continue;
      }

      const std::string &templateToFit="";// = query.get name of object to fit

      //Fit result of drill using local features/template matching
      FeatureCloud object_template;
      object_template.loadInputCloud(packagePath + "/objects_dataset/cad_models/" + templateToFit + "/" + templateToFit + ".pcd");

      pcl::PointIndicesPtr indices(new pcl::PointIndices());
      rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *indices);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

      pcl::ExtractIndices<pcl::PointXYZRGBA> ei;
      ei.setInputCloud(cloud_ptr);
      ei.setIndices(indices);
      ei.filter(*cluster_cloud);

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

      Eigen::Matrix3f rotation  = best_alignment.final_transformation.block<3, 3>(0, 0);
      Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
      outInfo("=======================");
      printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
      printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
      printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
      printf("\n");
      printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
      outInfo("=======================");

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
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "original");
      for(size_t i = 0; i < transfResults_.size(); ++i)
      {
        const cv::Scalar &c = colors[i % numberOfColors];
        const std::string &name = "match" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(transfResults_[i], c.val[2], c.val[1], c.val[0]);
        visualizer.addPointCloud(transfResults_[i], color, name);
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, name);
      }
    }
    else
    {
      visualizer.removeAllPointClouds();
      visualizer.addPointCloud(dispCloud, "original");
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "original");
      for(size_t i = 0; i < transfResults_.size(); ++i)
      {
        const cv::Scalar &c = colors[i % numberOfColors];
        const std::string &name = "match" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(transfResults_[i], c.val[2], c.val[1], c.val[0]);
        visualizer.addPointCloud(transfResults_[i], color, name);
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, name);
      }
    }
  }
};

MAKE_AE(Matcher)
