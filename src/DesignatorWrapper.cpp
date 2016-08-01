#include <rs_kbreasoning/DesignatorWrapper.h>

using namespace rs;

designator_integration::Designator *DesignatorWrapper::req_designator = NULL;
designator_integration::Designator *DesignatorWrapper::res_designator = NULL;

DesignatorWrapper::DesignatorWrapper()
{
  if(!req_designator)
  {
    req_designator = new designator_integration::Designator();
  }
  if(!res_designator)
  {
    res_designator = new designator_integration::Designator();
  }
  tcas = NULL;
}

DesignatorWrapper::~DesignatorWrapper()
{
  mode = CLUSTER;
}

DesignatorWrapper::DesignatorWrapper(uima::CAS *cas) : tcas(cas)
{
  if(!req_designator)
  {
    req_designator = new designator_integration::Designator();
  }
  if(!res_designator)
  {
    res_designator = new designator_integration::Designator();
  }
  mode = CLUSTER;
}


void DesignatorWrapper::setCAS(uima::CAS *cas)
{
  tcas = cas;
}

void DesignatorWrapper::setMode(DesignatorProcessMode m)
{
  mode = m;
}

designator_integration_msgs::DesignatorResponse DesignatorWrapper::getDesignatorResponseMsg()
{
  std::vector<designator_integration::Designator> objDesignators;
  getObjectDesignators(objDesignators);
  designator_integration_msgs::DesignatorResponse designatorResponseMsg;
  for(size_t i = 0; i < objDesignators.size(); ++i)
  {
    if(objDesignators[i].children().size() > 0)
    {
      designatorResponseMsg.designators.push_back(objDesignators[i].serializeToMessage());
    }
  }
  outInfo("designatorResponseMsg.designators.size= " << designatorResponseMsg.designators.size());
  return designatorResponseMsg;
}

bool DesignatorWrapper::getObjectDesignators(std::vector<designator_integration::Designator> &objectDesignators)
{
  if(!tcas)
  {
    std::cout << "NULL Pointer in DesignatorWrapper::getDesignatorResponse. tcas is not set! Use DesignatorWrapper::setCAS before calling this method" << std::endl;
    designator_integration::Designator d;
    return false;
  }

  rs::SceneCas cas(*tcas);
  rs::Scene scene = cas.getScene();

  now = scene.timestamp();

  designator_integration::Designator res;

  std::vector<rs::HandleAnnotation> handles;
  std::vector<rs::ARMarker> arMarkers;
  scene.annotations.filter(handles);
  scene.annotations.filter(arMarkers);
  for(int i = 0; i < handles.size(); i++)
  {
    convert(handles[i], res);
    objectDesignators.push_back(res);
    res.clear();
  }
  for(int i = 0; i < arMarkers.size(); i++)
  {
    convert(arMarkers[i], res);
    objectDesignators.push_back(res);
    res.clear();
  }

  if(mode == CLUSTER)
  {
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);
    process(clusters, objectDesignators);
  }
  else
  {
    uint64_t now = ros::Time::now().toNSec();
    std::vector<rs::Object> allObjects, objects;
    cas.get(VIEW_OBJECTS, allObjects);
    for(size_t i = 0; i < allObjects.size(); ++i)
    {
      rs::Object &object = allObjects[i];
      double lastSeen = (now - (uint64_t)object.lastSeen()) / 1000000000.0;
      if(lastSeen < 600)
      {
        objects.push_back(object);
      }
    }
    process(objects, objectDesignators);
  }
  return true;
}

void DesignatorWrapper::filterClusters(const std::vector<rs::Cluster> input, const designator_integration::Designator *output) const
{

}

void DesignatorWrapper::convert(rs::Cluster &input, const size_t id, designator_integration::KeyValuePair *object)
{
  object->setValue("CLUSTERID", id);
}

void DesignatorWrapper::convert(rs::Object &input, const size_t id, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("RESOLUTION");
  valuePair->setValue("OBJECTID", id);
  valuePair->setValue("LASTSEEN", now - input.lastSeen());
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::Detection &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("DETECTION");
  valuePair->setValue("CONFIDENCE", input.confidence());
  valuePair->setValue("SOURCE", input.source());
  valuePair->setValue("TYPE", input.name());
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::TFLocation &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("LOCATION");
  valuePair->setValue("RELATION", input.reference_desc());
  valuePair->setValue("FRAME", input.frame_id());
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::Segment &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *dimensions = new designator_integration::KeyValuePair("DIMENSIONS-2D");
  dimensions->setValue("WIDTH", input.lengthX());
  dimensions->setValue("HEIGHT", input.lengthY());

  designator_integration::KeyValuePair *seg = new designator_integration::KeyValuePair("SEGMENT");
  //seg->setValue("POSE", pose_stamped_msgs);
  seg->addChild(dimensions);

  object->addChild(seg);
}

void DesignatorWrapper::convert(rs::Geometry &input, designator_integration::KeyValuePair *object)
{
  rs::BoundingBox3D bb = input.boundingBox();

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose_stamped_msgs;
  rs::conversion::from(input.camera(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose_stamped_msgs);

  designator_integration::KeyValuePair *dimensions = new designator_integration::KeyValuePair("DIMENSIONS-3D");
  dimensions->setValue("WIDTH", bb.width());
  dimensions->setValue("HEIGHT", bb.height());
  dimensions->setValue("DEPTH", bb.depth());

  designator_integration::KeyValuePair *box = new designator_integration::KeyValuePair("BOUNDINGBOX");
  box->setValue("POSE", pose_stamped_msgs);
  box->setValue("SIZE", input.size());
  box->addChild(dimensions);

  object->addChild(box);
}

void DesignatorWrapper::convert(rs::Shape &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("SHAPE");
  valuePair->setValue(input.shape());
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::PoseAnnotation &input, designator_integration::KeyValuePair *object)
{
  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose_stamped_msgs;
  rs::conversion::from(input.camera(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose_stamped_msgs);
  object->setValue("POSE", pose_stamped_msgs);
}

void DesignatorWrapper::convert(rs::SemanticColor &input, designator_integration::KeyValuePair *object)
{
  const std::vector<std::string> &colors = input.color();
  const std::vector<float> &ratios = input.ratio();
  designator_integration::KeyValuePair *valuePairs = new designator_integration::KeyValuePair("COLOR");
  for(size_t i = 0; i < colors.size(); ++i)
  {
    const std::string &color = colors[i];
    const float &ratio = ratios[i];
    valuePairs->setValue(color, ratio);
  }
  object->addChild(valuePairs);
}

void DesignatorWrapper::convert(rs::MLNAtoms &input, designator_integration::KeyValuePair *object)
{
  const std::vector<std::string> &atoms = input.atoms();
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("MLNATOMS");

  for(size_t i = 0; i < atoms.size(); ++i)
  {
    designator_integration::KeyValuePair *temp = new designator_integration::KeyValuePair("ATOM");
    std::stringstream id;
    id << "atom_ " << i;
    temp->setValue(id.str(), atoms[i]);
    valuePair->addChild(temp);
  }
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::NamedLink &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("NAMEDLINK");
  valuePair->setValue("NAME", input.name());
  valuePair->setValue("URL", input.url());
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::Goggles &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("GOGGLES");
  valuePair->setValue("CATEGORY", input.category());
  valuePair->setValue("TITLE", input.title());
  valuePair->setValue("PREVIEW-LINK", input.preview_link());

  std::vector<rs::NamedLink> namedlinks = input.links();
  designator_integration::KeyValuePair *links = new designator_integration::KeyValuePair("NAMEDLINKS");
  convertAll(namedlinks, links);

  valuePair->addChild(links);

  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::Features &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("RESPONSE");

  std::vector<rs::Response> resps = input.response();
  if(resps.empty())
  {
    return;
  }
  rs::Response &resp = resps[0];

  std::vector<std::string> classes = resp.classNames();
  cv::Mat respM;
  rs::conversion::from(resp.response.get(), respM);
  if((size_t)respM.at<float>(0) > classes.size())
  {
    return;
  }

  valuePair->setValue(classes[(size_t)respM.at<float>(0)]);
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs_demos::Volume &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *volume = new designator_integration::KeyValuePair("VOLUME");
  volume->setValue(input.volume());
  object->addChild(volume);
}

void DesignatorWrapper::convert(rs_demos::Substance &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *substance = new designator_integration::KeyValuePair("CONTAINS");
  substance->setValue("SUBSTANCE", input.substanceName());
  object->addChild(substance);
}

iai_robosherlock_msgs::PerceivedObjects DesignatorWrapper::getObjectsMsgs()
{
  return objects_;
}

void DesignatorWrapper::convert(rs::ARMarker &input, designator_integration::Designator &arDesignator)
{
  arDesignator.setValue("type", "ARMARKER");
  arDesignator.setValue("ID", input.name());

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose_stamped_msgs;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose_stamped_msgs);
  arDesignator.setValue("POSE", pose_stamped_msgs);
  //  res.designators.push_back(arDesignator.serializeToMessage());


}

void DesignatorWrapper::convert(rs::HandleAnnotation &input,
    designator_integration::Designator &handleDesignator)
{
  handleDesignator.setValue("type", "HANDLE");
  handleDesignator.setValue("NAME", input.name());
  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose_stamped_msgs;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose_stamped_msgs);
  handleDesignator.setValue("POSE", pose_stamped_msgs);
}
