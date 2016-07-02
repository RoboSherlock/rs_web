// UIMA
#include <uima/api.hpp>

// RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/segmentation/supervoxel_clustering.h>


// OpenCV
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/ocl/ocl.hpp>

//json_prolog
#include<json_prolog/prolog.h>

#include <algorithm>

using namespace uima;

class ClusterToPartsSegmenter : public DrawingAnnotator
{

private:
  typedef pcl::PointXYZRGBA PointT;
  pcl::PointCloud<PointT>::Ptr cloudPtr;
  pcl::PointCloud<PointT>::Ptr dispCloudPtr;
  pcl::PointCloud<pcl::Normal>::Ptr normalPtr;

  struct ClusterWithParts
  {
    std::vector<pcl::PointIndicesPtr> partsOfClusters;
    pcl::PointCloud<pcl::PointNormal>::Ptr svNormalCloud;

    cv::Mat msSegmentsImage, mask;
    cv::Rect clusterRoi;
    std::vector<cv::Vec3b> colorClusterLabels;
    std::map<int, std::vector<cv::Point>> colorClusters; //map label to points in the image

  };


  std::vector<ClusterWithParts> clustersWithParts;

  double pointSize;
  //supervoxels

  float voxel_resolution;
  float seed_resolution;
  float color_importance;
  float spatial_importance;
  float normal_importance;

  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxelClusters;

  pcl::PointCloud<PointT>::Ptr dispCloud;
  pcl::PointCloud<pcl::PointNormal>::Ptr svNormalCloud;

  cv::Mat dispRGB;

public:

  ClusterToPartsSegmenter(): DrawingAnnotator(__func__), dispCloudPtr(new pcl::PointCloud<PointT>), pointSize(2), voxel_resolution(0.008f),
    seed_resolution(0.1f)
  {

  }


  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("Initialize");
    color_importance = 0.4f;
    spatial_importance = 0.4f;
    normal_importance = 1.0f;
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("Destroy");
    return UIMA_ERR_NONE;
  }

private:

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("Process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;

    supervoxelClusters.clear();

    clustersWithParts.clear();

    cloudPtr.reset(new pcl::PointCloud<PointT>);
    normalPtr.reset(new pcl::PointCloud<pcl::Normal>);


    cas.get(VIEW_CLOUD, *cloudPtr);
    cas.get(VIEW_NORMALS, *normalPtr);
    cas.get(VIEW_COLOR_IMAGE_HD, dispRGB);
    scene.identifiables.filter(clusters);

    rs::Query qs = rs::create<rs::Query>(tcas);
    std::string objToProcess;
    if(cas.getFS("QUERY", qs))
    {
      objToProcess = qs.inspect();
    }
    //todo if there is time...check if object can hold other objects

    std::stringstream prologQuery;
    //    prologQuery<<"owl_subclass_of(Obj,rs_test_objects:'ObjectsToTest'),class_properties(Obj,"<<
    //                 "rs_components:'hasVisualProperty',rs_test_objects:'ObjectPart').";
    //        prologQuery<<"owl_subclass_of(Obj,rs_test_objects:'ObjectsToTest'),class_properties(Obj,"<<
    //                     "rs_components:'hasVisualProperty',rs_test_objects:'ObjectPart').";
    //    http://knowrob.org/kb/rs_test_objects.owl#Plate

    std::transform(objToProcess.begin(), objToProcess.end(), objToProcess.begin(), (int( *)(int)) std::tolower);
    if(!objToProcess.empty())
    {
      objToProcess[0] = std::toupper(objToProcess[0]);
    }
    outWarn("Object Queried for is: " << objToProcess);


    prologQuery << "class_properties(rs_test_objects:'" << objToProcess << "',rs_components:'hasVisualProperty',rs_test_objects:'ObjectPart').";
    json_prolog::Prolog pl;
    json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
    if(bdgs.begin() == bdgs.end())
    {
      outInfo("Queried Object does not meet requirements of this annotator");
      return UIMA_ERR_NONE; // Indicate failure
    }
    //    for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin();
    //        it != bdgs.end(); it++)
    //    {
    //      json_prolog::PrologBindings bdg = *it;
    //      outWarn("Result");
    //    }

    for(int i = 0; i < clusters.size(); ++i)
    {

      rs::Cluster &cluster = clusters[i];
      std::vector<rs::Detection> detections;
      cluster.annotations.filter(detections);
      if(detections.empty())
      {
        continue;
      }
      rs::Detection detection = detections[0];

      //json query here?..if object has parts

      if(boost::iequals(detection.name(), objToProcess))
      {
        ClusterWithParts clusterSplit;

        //2D

        if(cluster.rois.has())
        {
          rs::ImageROI imageRoi = cluster.rois.get();

          rs::conversion::from(imageRoi.roi_hires(), clusterSplit.clusterRoi);
          rs::conversion::from(imageRoi.mask_hires(), clusterSplit.mask);
          cv::Mat clusterImg(dispRGB, clusterSplit.clusterRoi);

          cv::ocl::oclMat oclImgRaw, oclImgConverted;
          oclImgRaw.upload(clusterImg);
          cv::ocl::cvtColor(oclImgRaw, oclImgConverted, CV_BGR2BGRA);
          cv::Mat segmentedImg;
          cv::ocl::meanShiftSegmentation(oclImgConverted, segmentedImg, 40, 40, (clusterImg.rows * clusterImg.cols) * 0.015);
          cv::cvtColor(segmentedImg, clusterSplit.msSegmentsImage, CV_BGRA2BGR);

          std::vector<int> labelCount;
          for(int y = 0; y < clusterSplit.msSegmentsImage.rows; ++y)
            for(int x = 0; x < clusterSplit.msSegmentsImage.cols; ++x)
            {

              if(clusterSplit.mask.at<uint8_t>(y, x) != 0)
              {
                bool labelExists = false;
                cv::Vec3b label = clusterSplit.msSegmentsImage.at<cv::Vec3b>(cv::Point(x, y));
                for(int l = 0; l < clusterSplit.colorClusterLabels.size(); ++l)
                {
                  if(label == clusterSplit.colorClusterLabels[l])
                  {
                    //                   outInfo("found new label: " << label);
                    labelCount[l]++;
                    labelExists = true;
                  }
                }
                if(!labelExists)
                {
                  //                  outInfo("Adding new label: " << label);
                  labelCount.push_back(0);
                  clusterSplit.colorClusterLabels.push_back(label);
                }
              }
            }

          //now create the clusters FFS
          for(int y = 0; y < clusterSplit.msSegmentsImage.rows; ++y)
            for(int x = 0; x < clusterSplit.msSegmentsImage.cols; ++x)
            {

              if(clusterSplit.mask.at<uint8_t>(y, x) != 0)
              {
                cv::Vec3b label = clusterSplit.msSegmentsImage.at<cv::Vec3b>(cv::Point(x, y));
                for(int l = 0; l < clusterSplit.colorClusterLabels.size(); ++l)
                {
                  if(label == clusterSplit.colorClusterLabels[l])
                  {
                    if(labelCount[l] > (clusterImg.rows * clusterImg.cols) * 0.015)
                    {
                      clusterSplit.colorClusters[l].push_back(cv::Point(x, y));
                    }
                  }
                }
              }
            }
          outInfo("found " << clusterSplit.colorClusterLabels.size() << " color cluster labels");
          for(int l = 0; l < labelCount.size(); ++l)
          {
            //            outInfo("Label id " << l << " has " << labelCount[l] << " pixels");
          }
          outInfo("found " << clusterSplit.colorClusters.size() << " color clusters");
        }

        //3D
        pcl::PointIndicesPtr clusterIndices(new pcl::PointIndices());
        rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *clusterIndices);
        pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());

        pcl::ExtractIndices<PointT> ei;
        ei.setInputCloud(cloudPtr);
        ei.setIndices(clusterIndices);
        ei.filter(*clusterCloud);

        //      pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterNoAlpha (new pcl::PointCloud<pcl::PointXYZRGB>);
        //      pcl::copyPointCloud(*clusterCloud,*clusterNoAlpha);

        pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution, false);
        super.setInputCloud(clusterCloud);
        super.setColorImportance(color_importance);
        super.setSpatialImportance(spatial_importance);
        super.setNormalImportance(normal_importance);
        super.extract(supervoxelClusters);

        clusterSplit.svNormalCloud = super.makeSupervoxelNormalCloud(supervoxelClusters);

        outInfo("Cluster split into: " << supervoxelClusters.size() << " supervoxels");
        //group voxels based on their surface normal in two groups...a bit hacky atm. will make it nicer eventually

        std::multimap<uint32_t, uint32_t> supervoxel_adjacency;

        super.getSupervoxelAdjacency(supervoxel_adjacency);
        std::multimap<uint32_t, uint32_t>::iterator labelItr = supervoxel_adjacency.begin();
        std::vector<bool> processed(supervoxelClusters.size(), false);

        pcl::PointIndicesPtr partOneIndices(new pcl::PointIndices());
        pcl::PointIndicesPtr partTwoIndices(new pcl::PointIndices());

        for(; labelItr != supervoxel_adjacency.end();)
        {
          uint32_t supervoxel_label = labelItr->first;
          pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxelClusters.at(supervoxel_label);
          pcl::PointNormal svNormal;
          supervoxel->getCentroidPointNormal(svNormal);
          std::multimap<uint32_t, uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
          for(; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
          {
            pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxelClusters.at(adjacent_itr->second);
            pcl::PointNormal svNeighbourNormal;
            neighbor_supervoxel->getCentroidPointNormal(svNeighbourNormal);
            Eigen::Map<const Eigen::Vector3f> point_a_normal = svNormal.normal, point_b_normal = svNeighbourNormal.normal;
            //          outInfo("Angle between svl:" << supervoxel_label << " and svl:" << adjacent_itr->second << " is: " << fabs(point_a_normal.dot(point_b_normal)));
            Eigen::Vector3f dist(svNormal.x - svNeighbourNormal.x, svNormal.y - svNeighbourNormal.y, svNormal.z - svNeighbourNormal.z);
            if(fabs(point_a_normal.dot(point_b_normal)) > 0.87 && !processed[(int)adjacent_itr->second - 1])//this seems to be a better condition, need to investigate a bit more
              //          if( (180*acos(dist.dot(point_a_normal)))/M_PI > 89 && (180*acos(dist.dot(point_a_normal)))/M_PI <92 && !processed[(int)adjacent_itr->second - 1])
            {
              processed[(int)adjacent_itr->second - 1] = true;
            }
          }

          labelItr = supervoxel_adjacency.upper_bound(supervoxel_label);
        }

        pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloud = super.getLabeledCloud();

        for(unsigned int i =  0; i < labeledCloud->points.size(); ++i)
        {
          pcl::PointXYZL &p = labeledCloud->points[i];
          if(p.label != 0 && processed[(int)p.label - 1])
          {
            partOneIndices->indices.push_back(clusterIndices->indices.at(i));
          }
          else
          {
            partTwoIndices->indices.push_back(clusterIndices->indices.at(i));
          }
        }
        clusterSplit.partsOfClusters.push_back(partOneIndices);
        clusterSplit.partsOfClusters.push_back(partTwoIndices);
        clustersWithParts.push_back(clusterSplit);


        //ok, now do something smart FFS
        std::vector<pcl::PointIndicesPtr> final_clusters;
        std::vector<bool> assignedIndices(cloudPtr->points.size(), false);
        outWarn(detection.name() << " has " << clusterSplit.colorClusters.size() << " colored parts and " << clusterSplit.partsOfClusters.size() << " shape based parts");

        //for now do it like this: if more color clusters take those if not take the others
        if(clusterSplit.colorClusters.size() >= clusterSplit.partsOfClusters.size())
        {
          cv::Rect roi = clusterSplit.clusterRoi;
          for(auto & c : clusterSplit.colorClusters)
          {
            rs::ClusterPart part = rs::create<rs::ClusterPart>(tcas);

            pcl::PointIndices coloredIndice;
            for(unsigned int j = 0; j < c.second.size(); ++j)
            {
              cv::Point p = c.second[j];
              coloredIndice.indices.push_back((p.y / 2 + roi.y / 2) * 640 + (p.x / 2 + roi.x / 2));
            }
            if(!coloredIndice.indices.empty())
            {
              part.indices.set(rs::conversion::to(tcas, coloredIndice));
              cluster.annotations.append(part);
            }
          }

        }
        else
        {
          for(int pclClIdx = 0; pclClIdx < clusterSplit.partsOfClusters.size(); pclClIdx++)
          {
            rs::ClusterPart part = rs::create<rs::ClusterPart>(tcas);
            part.indices.set(rs::conversion::to(tcas, *clusterSplit.partsOfClusters[pclClIdx]));
            cluster.annotations.append(part);
          }
        }
      }
    }

    return UIMA_ERR_NONE;
  }

  static bool enforceConvexNormalsSimilarity(const pcl::PointXYZRGBNormal &point_a, const pcl::PointXYZRGBNormal &point_b, float squared_distance)
  {
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;

    pcl::PointXYZRGBNormal temp;
    temp.x = point_a.x - point_b.x;
    temp.y = point_a.y - point_b.y;
    temp.z = point_a.z - point_b.z;

    double dot_p = point_b.normal_x * temp.x
                   + point_b.normal_y * temp.y
                   + point_b.normal_x * temp.z;

    dot_p = dot_p > 1 ? 1 : dot_p;
    dot_p = dot_p < -1 ? -1 : dot_p;

    //    if((acos(dot_p) < 70 * M_PI / 180))
    //    if(acos(point_a_normal.dot(point_b_normal)) < 70 * M_PI / 180)
    //    {
    //      return(true);
    //    }
    //    return (false);
    if(fabs(point_a.curvature - point_b.curvature) < 0.0009)
    {
      return (true);
    }
    if(acos(fabs(point_a_normal.dot(point_b_normal))) < 5 * M_PI / 180)
    {
      return (true);
    }
    return(false);
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = dispRGB.clone();
    //keep this for addign options to viewer
    //    for(unsigned int i = 0; i < clustersWithParts.size(); ++i)
    //    {
    //      cv::Mat &imgSeg = clustersWithParts[i].msSegmentsImage;
    //      cv::Mat &mask = clustersWithParts[i].mask;
    //      cv::Rect roi = clustersWithParts[i].clusterRoi;
    //      for(int y = 0; y < imgSeg.rows; ++y)
    //        for(int x = 0; x < imgSeg.cols; ++x)
    //        {
    //          if(mask.at<uint8_t>(y, x) != 0)
    //          {
    //            cv::Vec3b color = imgSeg.at<cv::Vec3b>(cv::Point(x, y));
    //            disp.at<cv::Vec3b>(cv::Point(x + roi.x, y + roi.y)) = color;
    //          }
    //        }
    //    }
    for(unsigned int i = 0; i < clustersWithParts.size(); ++i)
    {
      cv::Rect roi = clustersWithParts[i].clusterRoi;
      int idx = 0;
      std::vector<int> coloredIndice;
      for(auto & c : clustersWithParts[i].colorClusters)
      {
        for(unsigned int j = 0; j < c.second.size(); ++j)
        {
          cv::Point p = c.second[j];
          disp.at<cv::Vec3b>(cv::Point(p.x + roi.x, p.y + roi.y)) = rs::common::cvVec3bcolorsVec[idx % rs::common::cvVec3bcolorsVec.size()];
          coloredIndice.push_back((p.y + roi.y) * 0.5 * 640 + (p.x + roi.x) / 2);
        }
        idx++;
      }
      //      for(unsigned int j = 0; j < clustersWithParts[i].partsOfClusters.size(); ++j)
      //      {
      //        pcl::PointIndicesPtr &indices = clustersWithParts[i].partsOfClusters[j];
      //        for(unsigned int k = 0; k < indices->indices.size(); ++k)
      //        {
      //            int index = indices->indices[k];
      ////            auto result =  std::find(coloredIndice.begin(),coloredIndice.end(),index);
      ////            if (result == std::end(coloredIndice))
      ////            {
      //                disp.at<cv::Vec3b>(cv::Point((index%640)*2+1,(index/640)*2)) = colors[idx];
      //                disp.at<cv::Vec3b>(cv::Point((index%640)*2,(index/640)*2+1)) = colors[idx];
      //                disp.at<cv::Vec3b>(cv::Point((index%640)*2,(index/640)*2-1)) = colors[idx];
      //                disp.at<cv::Vec3b>(cv::Point((index%640)*2-1,(index/640)*2)) = colors[idx];
      ////            }
      //        }
      //        idx++;
      //      }
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {

    for(unsigned int i = 0; i < clustersWithParts.size(); ++i)
    {
      for(unsigned int j = 0; j < clustersWithParts[i].partsOfClusters.size(); ++j)
      {
        pcl::PointIndicesPtr &indices = clustersWithParts[i].partsOfClusters[j];
        for(unsigned int k = 0; k < indices->indices.size(); ++k)
        {
          int index = indices->indices[k];
          cloudPtr->points[index].rgba = rs::common::colors[j % COLOR_SIZE];
        }
      }
    }

    if(firstRun)
    {
      visualizer.addPointCloud(cloudPtr, std::string("voxel_centroids"));
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, std::string("voxel_centroids"));
    }
    else
    {
      visualizer.updatePointCloud(cloudPtr, std::string("voxel_centroids"));
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, std::string("voxel_centroids"));
    }
  }

};
MAKE_AE(ClusterToPartsSegmenter)
