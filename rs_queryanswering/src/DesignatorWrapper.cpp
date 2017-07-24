#include <rs_queryanswering/DesignatorWrapper.h>

namespace rs
{

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
    std::vector<rs::Object> allObjects, objects;
    cas.get(VIEW_OBJECTS, allObjects);
    outWarn("objects found: " << allObjects.size());
    for(size_t i = 0; i < allObjects.size(); ++i)
    {
      rs::Object &object = allObjects[i];
      double lastSeen = (now - (uint64_t)object.lastSeen()) / 1000000000.0;
      if(lastSeen == 0)
      {
        objects.push_back(object);
      }
    }
    process(objects, objectDesignators);
  }
  return true;
}

void DesignatorWrapper::convert(rs::Cluster &input, const size_t id, designator_integration::KeyValuePair *object)
{
  object->setValue("id", std::to_string(id));
}

void DesignatorWrapper::convert(rs::Object &input, const size_t id, designator_integration::KeyValuePair *object)
{
  //  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("RESOLUTION");
  //  valuePair->setValue("ID", id);
  //  valuePair->setValue("LASTSEEN", now - input.lastSeen());
  //  object->addChild(valuePair);
  object->setValue("id", std::to_string(id));
}

void DesignatorWrapper::convert(rs::Detection &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("class");
  valuePair->setValue("confidence", input.confidence());
  valuePair->setValue("source", input.source());
  valuePair->setValue("name", input.name());
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::TFLocation &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("location");
  valuePair->setValue("relation", input.reference_desc());
  valuePair->setValue("frame", input.frame_id());
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::Segment &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *dimensions = new designator_integration::KeyValuePair("dimensions-2D");
  dimensions->setValue("width", input.lengthX());
  dimensions->setValue("height", input.lengthY());

  designator_integration::KeyValuePair *seg = new designator_integration::KeyValuePair("segment");
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

  designator_integration::KeyValuePair *dimensions = new designator_integration::KeyValuePair("dimensions-3D");
  dimensions->setValue("width", bb.width());
  dimensions->setValue("height", bb.height());
  dimensions->setValue("depth", bb.depth());

  designator_integration::KeyValuePair *box = new designator_integration::KeyValuePair("boundingbox");
  box->setValue("pose", pose_stamped_msgs);
  box->setValue("size", input.size());
  box->setValue("dist-to-plane", input.distanceToPlane());
  box->addChild(dimensions);

  object->addChild(box);
}

void DesignatorWrapper::convert(rs::Shape &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("shape");
  valuePair->setValue(input.shape());
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::PoseAnnotation &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("pose");

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose_stamped_msgs;
  rs::conversion::from(input.camera(), tf_stamped_pose);

  tf::StampedTransform transf(tf::Transform(tf_stamped_pose.getRotation(),tf_stamped_pose.getOrigin()),tf_stamped_pose.stamp_,tf_stamped_pose.frame_id_,"");
  transf.child_frame_id_ = "";
  geometry_msgs::TransformStamped transf_stamped_msg;


  tf::poseStampedTFToMsg(tf_stamped_pose, pose_stamped_msgs);
  tf::transformStampedTFToMsg(transf,transf_stamped_msg);

  valuePair->setValue("transform", transf_stamped_msg);
  valuePair->setValue("source", input.source());
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::SemanticColor &input, designator_integration::KeyValuePair *object)
{
  const std::vector<std::string> &colors = input.color();
  const std::vector<float> &ratios = input.ratio();
  designator_integration::KeyValuePair *valuePairs = new designator_integration::KeyValuePair("color");
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
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("mln-atoms");

  for(size_t i = 0; i < atoms.size(); ++i)
  {
    designator_integration::KeyValuePair *temp = new designator_integration::KeyValuePair("atom");
    std::stringstream id;
    id << "atom_ " << i;
    temp->setValue(id.str(), atoms[i]);
    valuePair->addChild(temp);
  }
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::NamedLink &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("namedlink");
  valuePair->setValue("name", input.name());
  valuePair->setValue("url", input.url());
  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::Goggles &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("goggles");
  valuePair->setValue("category", input.category());
  valuePair->setValue("title", input.title());
  valuePair->setValue("preview-link", input.preview_link());

  std::vector<rs::NamedLink> namedlinks = input.links();
  designator_integration::KeyValuePair *links = new designator_integration::KeyValuePair("namedlinks");
  convertAll(namedlinks, links);
  valuePair->addChild(links);

  object->addChild(valuePair);
}

void DesignatorWrapper::convert(rs::Features &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *valuePair = new designator_integration::KeyValuePair("response");

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

void DesignatorWrapper::convert(rs::ClusterPart &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *part = new designator_integration::KeyValuePair(std::to_string(input.clID()));

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose_stamped_msgs;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose_stamped_msgs);
  part->setValue("name", input.name());
  part->setValue("pose", pose_stamped_msgs);
  object->addChild(part);
}

void DesignatorWrapper::convert(rs_demos::Volume &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *volume = new designator_integration::KeyValuePair("volume");
  volume->setValue(input.volume());
  object->addChild(volume);
}

void DesignatorWrapper::convert(rs_demos::Substance &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *substance = new designator_integration::KeyValuePair("contains");
  substance->setValue("substance", input.substanceName());
  object->addChild(substance);
}

iai_robosherlock_msgs::PerceivedObjects DesignatorWrapper::getObjectsMsgs()
{
  return objects_;
}

void DesignatorWrapper::convert(rs::ARMarker &input, designator_integration::Designator &arDesignator)
{
  arDesignator.setValue("type", "armarker");
  arDesignator.setValue("id", input.name());

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose_stamped_msgs;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose_stamped_msgs);
  arDesignator.setValue("pose", pose_stamped_msgs);
  //  res.designators.push_back(arDesignator.serializeToMessage());

}

void DesignatorWrapper::convert(rs::HandleAnnotation &input,
                                designator_integration::Designator &handleDesignator)
{
  handleDesignator.setValue("handle", input.name());
  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose_stamped_msgs;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose_stamped_msgs);
  handleDesignator.setValue("pose", pose_stamped_msgs);
}

void DesignatorWrapper::convert(rs_demos::Pizza &input, designator_integration::KeyValuePair *object)
{
  designator_integration::KeyValuePair *pizza = new designator_integration::KeyValuePair("pizza");

  rs::PoseAnnotation pose = input.pose();
  convert(pose, pizza);

  designator_integration::KeyValuePair *size = new designator_integration::KeyValuePair("size");
  size->setValue("width", input.size().width());
  size->setValue("height", input.size().height());
  pizza->addChild(size);

  designator_integration::KeyValuePair *gridSize = new designator_integration::KeyValuePair("dimensions");
  gridSize->setValue("width", input.dimensions().width());
  gridSize->setValue("height", input.dimensions().height());
  pizza->addChild(gridSize);

  designator_integration::KeyValuePair *fieldSize = new designator_integration::KeyValuePair("field-size");
  fieldSize->setValue("width", input.fieldSize().width());
  fieldSize->setValue("height", input.fieldSize().height());
  pizza->addChild(fieldSize);

  designator_integration::KeyValuePair *fields = new designator_integration::KeyValuePair("fields");
  std::vector<rs_demos::PizzaField> fieldsVec = input.fields();
  for(size_t i = 0; i < fieldsVec.size(); ++i)
  {
    rs_demos::PizzaField &inputF = fieldsVec[i];
    designator_integration::KeyValuePair *field = new designator_integration::KeyValuePair(std::to_string(i));

    rs::PoseAnnotation pose = inputF.pose();
    convert(pose, field);

    field->setValue("x", inputF.position().x());
    field->setValue("y", inputF.position().y());

    designator_integration::KeyValuePair *ingredients = new designator_integration::KeyValuePair("ingredient");
    std::vector<std::string> ingredientsVec = inputF.ingredients();
    for(size_t j = 0; j < ingredientsVec.size(); ++j)
    {
      ingredients->setValue(std::to_string(j), ingredientsVec[j]);
    }
    field->setValue("top-ingredient", ingredientsVec[ingredientsVec.size() - 1]);
    field->addChild(ingredients);
    fields->addChild(field);
  }
  pizza->addChild(fields);
  object->addChild(pizza);
}

template<>
void DesignatorWrapper::convertAll(std::vector<rs::ClusterPart> &all, designator_integration::KeyValuePair *object)
{
  if(!all.empty())
  {
    designator_integration::KeyValuePair *objParts = new designator_integration::KeyValuePair("obj-part");
    for(rs::ClusterPart input : all)
    {
      convert(input, objParts);
    }
    object->addChild(objParts);
  }
}
}
