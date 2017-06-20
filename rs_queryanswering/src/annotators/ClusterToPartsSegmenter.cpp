// UIMA
#include <uima/api.hpp>

// RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
#include <rs/conversion/bson.h>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <rs_queryanswering/KRDefinitions.h>
#include <rs_queryanswering/PrologInterface.h>

// OpenCV
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/ocl/ocl.hpp>

#include <algorithm>

#include <rs/utils/RSAnalysisEngine.h>

using namespace uima;

class ClusterToPartsSegmenter : public DrawingAnnotator
{

private:
  typedef pcl::PointXYZRGBA PointT;
  pcl::PointCloud<PointT>::Ptr cloudPtr_;
  pcl::PointCloud<PointT>::Ptr dispCloudPtr;
  pcl::PointCloud<pcl::Normal>::Ptr normalPtr_;

  RSAnalysisEngine engine;
  PrologInterface prologInterface;
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
  float colorImportance_;
  float spatialImportance_;
  float normalImportance_;

  pcl::PointCloud<PointT>::Ptr dispCloud_;
  pcl::PointCloud<pcl::PointNormal>::Ptr svNormalCloud;

  cv::Mat dispRGB;

public:

  ClusterToPartsSegmenter(): DrawingAnnotator(__func__), dispCloudPtr(new pcl::PointCloud<PointT>),prologInterface(), pointSize(2), voxel_resolution(0.008f),
    seed_resolution(0.1f)
  {

  }


  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("Initialize");
    colorImportance_ = 0.4f;
    spatialImportance_ = 0.4f;
    normalImportance_ = 1.0f;

    std::string lowlvlAEPath;
    rs::common::getAEPaths("lowlvl", lowlvlAEPath);
    outWarn("=============A DREAM IN A DREAM==============");
    engine.init(lowlvlAEPath);
    outWarn("=============END OF A DREAM IN A DREAM==============");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("Destroy");
    return UIMA_ERR_NONE;
  }

  void msColorSegmentations(const cv::Mat &img, ClusterWithParts &cwp)
  {
    cv::ocl::oclMat oclImgRaw, oclImgConverted;
    oclImgRaw.upload(img);
    cv::ocl::cvtColor(oclImgRaw, oclImgConverted, CV_BGR2BGRA);
    cv::Mat msClusters;
    cv::ocl::meanShiftSegmentation(oclImgConverted, msClusters, 40, 40, (img.rows * img.cols) * 0.015);
    cv::cvtColor(msClusters, cwp.msSegmentsImage, CV_BGRA2BGR);

    std::vector<int> labelCount;
    for(int y = 0; y < cwp.msSegmentsImage.rows; ++y)
      for(int x = 0; x < cwp.msSegmentsImage.cols; ++x)
      {
        if(cwp.mask.at<uint8_t>(y, x) != 0)
        {
          bool labelExists = false;
          cv::Vec3b label = cwp.msSegmentsImage.at<cv::Vec3b>(cv::Point(x, y));
          for(int l = 0; l < cwp.colorClusterLabels.size(); ++l)
          {
            if(label == cwp.colorClusterLabels[l])
            {
              labelCount[l]++;
              labelExists = true;
            }
          }
          if(!labelExists)
          {
            labelCount.push_back(0);
            cwp.colorClusterLabels.push_back(label);
          }
        }
      }

    //now create the clusters: needed because for some reason openCVs meanShift returns clusters
    // that have fewer points then set in the segm.method
    for(int y = 0; y < cwp.msSegmentsImage.rows; ++y)
      for(int x = 0; x < cwp.msSegmentsImage.cols; ++x)
      {
        if(cwp.mask.at<uint8_t>(y, x) != 0)
        {
          cv::Vec3b label = cwp.msSegmentsImage.at<cv::Vec3b>(cv::Point(x, y));
          for(int l = 0; l < cwp.colorClusterLabels.size(); ++l)
          {
            if(label == cwp.colorClusterLabels[l])
            {
              if(labelCount[l] > (img.rows * img.cols) * 0.015)
              {
                cwp.colorClusters[l].push_back(cv::Point(x, y));
              }
            }
          }
        }
      }
    outInfo("found " << cwp.colorClusterLabels.size() << " color cluster labels");
    outInfo("found " << cwp.colorClusters.size() << " color clusters");
  }

  void overSegmentAndGrow(const pcl::PointIndicesPtr &indices, ClusterWithParts &cwp)
  {
    pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> ei;
    ei.setInputCloud(cloudPtr_);
    ei.setIndices(indices);
    ei.filter(*clusterCloud);

    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxelClusters;
    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution, false);
    super.setInputCloud(clusterCloud);
    super.setColorImportance(colorImportance_);
    super.setSpatialImportance(spatialImportance_);
    super.setNormalImportance(normalImportance_);
    super.extract(supervoxelClusters);

    cwp.svNormalCloud = super.makeSupervoxelNormalCloud(supervoxelClusters);

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
        //outInfo("Angle between svl:" << supervoxel_label << " and svl:" << adjacent_itr->second << " is: " << fabs(point_a_normal.dot(point_b_normal)));
        Eigen::Vector3f dist(svNormal.x - svNeighbourNormal.x, svNormal.y - svNeighbourNormal.y, svNormal.z - svNeighbourNormal.z);
        if(std::abs(point_a_normal.dot(point_b_normal)) > 0.87 && !processed[(int)adjacent_itr->second - 1])
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
        partOneIndices->indices.push_back(indices->indices.at(i));
      }
      else
      {
        partTwoIndices->indices.push_back(indices->indices.at(i));
      }
    }
    cwp.partsOfClusters.push_back(partOneIndices);
    cwp.partsOfClusters.push_back(partTwoIndices);
  }


  void createImageRoi(const pcl::PointIndicesPtr &indices,
                      cv::Rect &roi, cv::Rect &roiHires,
                      cv::Mat &mask, cv::Mat &maskHires)
  {


    size_t width = cloudPtr_->width;
    size_t height = cloudPtr_->height;

    int min_x = width;
    int max_x = -1;
    int min_y = height;
    int max_y = -1;

    cv::Mat mask_full = cv::Mat::zeros(height, width, CV_8U);

    // get min / max extents (rectangular bounding box in image (pixel) coordinates)
    //#pragma omp parallel for
    for(size_t i = 0; i < indices->indices.size(); ++i)
    {
      const int idx = indices->indices[i];
      const int x = idx % width;
      const int y = idx / width;

      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);

      mask_full.at<uint8_t>(y, x) = 255;
    }

    roi = cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    roiHires = cv::Rect(roi.x << 1, roi.y << 1, roi.width << 1, roi.height << 1);
    mask_full(roi).copyTo(mask);
    cv::resize(mask, maskHires, cv::Size(0, 0), 2.0, 2.0, cv::INTER_NEAREST);
  }

private:

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("Process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;
    std::vector<rs::Plane> planes;
    clustersWithParts.clear();

    cloudPtr_.reset(new pcl::PointCloud<PointT>);
    normalPtr_.reset(new pcl::PointCloud<pcl::Normal>);

    cv::Mat depthHD, depth, rgb, rgbHD;
    cas.get(VIEW_CLOUD, *cloudPtr_);
    cas.get(VIEW_NORMALS, *normalPtr_);
    cas.get(VIEW_COLOR_IMAGE_HD, rgbHD);
    cas.get(VIEW_DEPTH_IMAGE_HD, depthHD);
    cas.get(VIEW_DEPTH_IMAGE, depth);
    cas.get(VIEW_COLOR_IMAGE, rgb);

    dispRGB = rgbHD.clone();

    scene.identifiables.filter(clusters);
    scene.annotations.filter(planes);

    rs::Query qs = rs::create<rs::Query>(tcas);
    std::string objToProcess;
    if(cas.getFS("QUERY", qs))
    {
      objToProcess = qs.inspect();
    }
    else
    {
      return UIMA_ERR_NONE;
    }
    //todo if there is time...check if object can hold other objects

    std::stringstream prologQuery;

    std::transform(objToProcess.begin(), objToProcess.end(), objToProcess.begin(), (int( *)(int)) std::tolower);
    //    if(!objToProcess.empty())
    //    {
    //      objToProcess[0] = std::toupper(objToProcess[0]);
    //    }
    outWarn("Object Queried for is: " << objToProcess);

    bool ok = prologInterface.q_classProperty(objToProcess, "rs_components:'hasVisualProperty'", "rs_objects:'ObjectPart'");
    if(!ok)
    {
      outInfo("Queried Object does not meet requirements of this annotator");
      return UIMA_ERR_NONE; // Indicate failure
    }

    std::vector<rs::Identifiable> mergedClusters, newClusters;

    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
      mergedClusters.push_back(cluster);
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
        ClusterWithParts clusterAsParts;

        //2D segmentation
        //        if(cluster.rois.has())
        //        {
        //          rs::ImageROI imageRoi = cluster.rois.get();
        //          rs::conversion::from(imageRoi.roi_hires(), clusterAsParts.clusterRoi);
        //          rs::conversion::from(imageRoi.mask_hires(), clusterAsParts.mask);
        //          cv::Mat clusterImg(dispRGB, clusterAsParts.clusterRoi);
        //          msColorSegmentations(clusterImg, clusterAsParts);
        //        }

        //3D segmentation
        pcl::PointIndicesPtr clusterIndices(new pcl::PointIndices());
        rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *clusterIndices);
        overSegmentAndGrow(clusterIndices, clusterAsParts);

        clustersWithParts.push_back(clusterAsParts);

        //now merge
        outWarn(detection.name() << " has " << clusterAsParts.colorClusters.size() << " colored parts and " << clusterAsParts.partsOfClusters.size() << " shape based parts");

        //for now do it like this: if more color clusters take those if not take the others

        if(clusterAsParts.colorClusters.size() >= clusterAsParts.partsOfClusters.size())
        {
          cv::Rect roi = clusterAsParts.clusterRoi;
          std::map<int, std::vector<cv::Point>>::iterator mapIt = clusterAsParts.colorClusters.begin();

          for(; mapIt != clusterAsParts.colorClusters.end(); ++mapIt)
          {
            rs::Cluster newCluster = rs::create<rs::Cluster>(*engine.getCas());
            pcl::PointIndices newClusterIndices;
            cv::Rect roiLowres, roiHires;
            outInfo("");
            roiHires = cv::boundingRect(mapIt->second);
            roiLowres.height = roiHires.height / 2;
            roiLowres.width = roiHires.width / 2;
            roiLowres.x = roiHires.x / 2;
            roiLowres.y = roiHires.y / 2;
            outInfo("");
            cv::Mat maskLowres = cv::Mat::zeros(roiLowres.height, roiLowres.width, CV_8U),
                    maskHires = cv::Mat::zeros(roiHires.height, roiHires.width, CV_8U);
            outInfo("");
            for(unsigned int j = 0; j < mapIt->second.size(); ++j)
            {
              cv::Point p = mapIt->second[j];
              maskHires.at<uint8_t>(p.y, p.x) = 255;
              maskLowres.at<uint8_t>(p.y / 2, p.x / 2) = 255;
              newClusterIndices.indices.push_back((p.y / 2 + roi.y / 2) * 640 + (p.x / 2 + roi.x / 2));
            }
            std::vector<int>::iterator it;
            it = std::unique(newClusterIndices.indices.begin(), newClusterIndices.indices.end());
            newClusterIndices.indices.resize(std::distance(newClusterIndices.indices.begin(), it));
            newClusterIndices.header = cloudPtr_->header;
            outInfo("New cluster indices size: " << newClusterIndices.indices.size());

            rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(*engine.getCas());
            rcp.indices.set(rs::conversion::to(*engine.getCas(), newClusterIndices));

            rs::ImageROI imageRoi = rs::create<rs::ImageROI>(*engine.getCas());
            imageRoi.mask(rs::conversion::to(*engine.getCas(), maskLowres));
            imageRoi.mask_hires(rs::conversion::to(*engine.getCas(), maskHires));
            imageRoi.roi(rs::conversion::to(*engine.getCas(), roiLowres));
            imageRoi.roi_hires(rs::conversion::to(*engine.getCas(), roiHires));

            rs::TFLocation location = rs::create<rs::TFLocation>(*engine.getCas());
            location.reference_desc.set("on");
            location.frame_id.set(objToProcess);

            newCluster.annotations.append(location);
            newCluster.rois.set(imageRoi);
            newCluster.points.set(rcp);
            newClusters.push_back(newCluster);
          }
        }
        else
        {
          int idxBiggest = -1;
          int nrOfIndeices = 0;
          for(int pclClIdx = 0; pclClIdx < clusterAsParts.partsOfClusters.size(); pclClIdx++)
          {
            if(clusterAsParts.partsOfClusters[pclClIdx]->indices.size() > nrOfIndeices)
            {
              nrOfIndeices = clusterAsParts.partsOfClusters[pclClIdx]->indices.size();
              idxBiggest = pclClIdx;
            }
          }
          for(int pclClIdx = 0; pclClIdx < clusterAsParts.partsOfClusters.size(); pclClIdx++)
          {
            if(pclClIdx == idxBiggest)
            {
              continue;
            }
            rs::Cluster newCluster = rs::create<rs::Cluster>(*engine.getCas());
            rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(*engine.getCas());
            rs::PointIndices uimaIndices = rs::conversion::to(*engine.getCas(), *clusterAsParts.partsOfClusters[pclClIdx]);
            rcp.indices.set(uimaIndices);

            cv::Rect roi, roiHires;
            cv::Mat mask, maskHires;
            createImageRoi(clusterAsParts.partsOfClusters[pclClIdx], roi, roiHires, mask, maskHires);

            rs::ImageROI imageRoi = rs::create<rs::ImageROI>(*engine.getCas());
            imageRoi.mask(rs::conversion::to(*engine.getCas(), mask));
            imageRoi.mask_hires(rs::conversion::to(*engine.getCas(), maskHires));
            imageRoi.roi(rs::conversion::to(*engine.getCas(), roi));
            imageRoi.roi_hires(rs::conversion::to(*engine.getCas(), roiHires));

            rs::TFLocation location = rs::create<rs::TFLocation>(*engine.getCas());
            location.reference_desc.set("on");
            location.frame_id.set(objToProcess);

            newCluster.annotations.append(location);
            newCluster.rois.set(imageRoi);
            newCluster.points.set(rcp);

            newClusters.push_back(newCluster);
          }
        }
      }
    }

    outInfo("Setting new Cas and scene and etc");
    //now run a lowLvlPipeline on the newly found objects;
    rs::SceneCas newCas(*engine.getCas());
    newCas.set(VIEW_CLOUD, *cloudPtr_);
    newCas.set(VIEW_COLOR_IMAGE_HD, rgbHD);
    newCas.set(VIEW_NORMALS, *normalPtr_);
    newCas.set(VIEW_DEPTH_IMAGE_HD, depthHD);
    newCas.set(VIEW_DEPTH_IMAGE, depth);
    newCas.set(VIEW_COLOR_IMAGE, rgb);

    rs::Scene newScene = newCas.getScene();

    tf::StampedTransform head_to_map;
    rs::conversion::from(scene.viewPoint.get(), head_to_map);

    rs::StampedTransform vp(rs::conversion::to(*engine.getCas(), head_to_map));
    newScene.viewPoint.set(vp);
    rs::Plane newPlane = rs::create<rs::Plane>(*engine.getCas());
    newPlane.id.set(planes[0].id());
    newPlane.model.set(planes[0].model());

    newScene.annotations.append(newPlane);
    newScene.identifiables.set(newClusters);
    engine.process();
    std::vector<rs::Cluster>  newClustersAnnotated;
    newScene.identifiables.filter(newClustersAnnotated);
    for(rs::Cluster c : newClustersAnnotated)
    {
      outInfo("Cluster has: " << c.annotations.size() << "annotations");
      mongo::BSONObj bson = rs::conversion::fromFeatureStructure(c, mongo::OID());
      mergedClusters.push_back(rs::conversion::toFeatureStructure(tcas, bson));
      std::vector<rs::TFLocation> locations;
      c.annotations.filter(locations);
      if(!locations.empty())
      {
        outInfo(locations[0].reference_desc() << " " << locations[0].frame_id());
      }
    }
    scene.identifiables.set(mergedClusters);
    engine.resetCas();
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
          disp.at<cv::Vec3b>(cv::Point(p.x + roi.x, p.y + roi.y)) = rs::common::cvVec3bColors[idx % rs::common::numberOfColors];
          coloredIndice.push_back((p.y + roi.y) * 0.5 * 640 + (p.x + roi.x) / 2);
        }
        idx++;
      }
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
          cloudPtr_->points[index].rgba = rs::common::colors[j % rs::common::numberOfColors];
        }
      }
    }

    if(firstRun)
    {
      visualizer.addPointCloud(cloudPtr_, std::string("voxel_centroids"));
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, std::string("voxel_centroids"));
    }
    else
    {
      visualizer.updatePointCloud(cloudPtr_, std::string("voxel_centroids"));
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, std::string("voxel_centroids"));
    }
  }

};
MAKE_AE(ClusterToPartsSegmenter)
