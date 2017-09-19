#include <rs_queryanswering/DesignatorWrapper.h>

namespace rs
{

rapidjson::Document *DesignatorWrapper::req_designator = NULL;
rapidjson::Document *DesignatorWrapper::res_designator = NULL;

DesignatorWrapper::DesignatorWrapper()
{
  if(!req_designator)
  {
    req_designator = new rapidjson::Document();
  }
  if(!res_designator)
  {
    res_designator = new rapidjson::Document();
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
    req_designator = new rapidjson::Document();
  }
  if(!res_designator)
  {
    res_designator = new rapidjson::Document();
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

bool DesignatorWrapper::getObjectDesignators(std::vector<std::string> &objectDesignators)
{
  if(!tcas)
  {
    std::cout << "NULL Pointer in DesignatorWrapper::getDesignatorResponse. tcas is not set! Use DesignatorWrapper::setCAS before calling this method" << std::endl;
    return false;
  }

  rs::SceneCas cas(*tcas);
  rs::Scene scene = cas.getScene();

  now = scene.timestamp();
  rapidjson::Document res;

  std::vector<rs::HandleAnnotation> handles;
  std::vector<rs::ARMarker> arMarkers;
  scene.annotations.filter(handles);
  scene.annotations.filter(arMarkers);
  for(int i = 0; i < handles.size(); i++, res.Clear())
  {
    convert(handles[i], res);
    objectDesignators.push_back(jsonToString(&res));
  }
  for(int i = 0; i < arMarkers.size(); i++, res.Clear())
  {
    convert(arMarkers[i], res);
    objectDesignators.push_back(jsonToString(&res));
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

void DesignatorWrapper::convert(rs::Cluster &input, const size_t id, rapidjson::Document *object)
{
  object->AddMember("id", std::to_string(id),object->GetAllocator());
}

void DesignatorWrapper::convert(rs::Object &input, const size_t id, rapidjson::Document *object)
{
  //  rapidjson::Document *valuePair = new rapidjson::Document("RESOLUTION");
  //  valuePair->setValue("ID", id);
  //  valuePair->setValue("LASTSEEN", now - input.lastSeen());
  //  object->addChild(valuePair);
   object->AddMember("id", std::to_string(id),object->GetAllocator());
}

void DesignatorWrapper::convert(rs::Detection &input, rapidjson::Document *object)
{
  rapidjson::Value nestedValue;
  nestedValue.SetObject();
  nestedValue.AddMember("confidence",input.confidence(),object->GetAllocator());
  nestedValue.AddMember("source",input.source(),object->GetAllocator());
  nestedValue.AddMember("name",input.name(),object->GetAllocator());

  object->AddMember("class",nestedValue,object->GetAllocator());
}

void DesignatorWrapper::convert(rs::TFLocation &input, rapidjson::Document *object)
{
  rapidjson::Value nestedValue;
  nestedValue.SetObject();
  nestedValue.AddMember("relation",input.reference_desc(),object->GetAllocator());
  nestedValue.AddMember("frame",input.frame_id(),object->GetAllocator());

  object->AddMember("location",nestedValue,object->GetAllocator());
}

void DesignatorWrapper::convert(rs::Segment &input, rapidjson::Document *object)
{
  rapidjson::Value nestedValue, segment;
  nestedValue.SetObject();
  nestedValue.AddMember("width",input.lengthX(),object->GetAllocator());
  nestedValue.AddMember("height",input.lengthY(),object->GetAllocator());

  segment.SetObject();
  segment.AddMember("dimensions-2D",nestedValue,object->GetAllocator());

  object->AddMember("segment",segment,object->GetAllocator());
}

void DesignatorWrapper::convert(rs::Geometry &input, rapidjson::Document *object)
{
  rs::BoundingBox3D bb = input.boundingBox();

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose;
  rs::conversion::from(input.camera(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose);

  rapidjson::Value nestedValue;
  rapidjson::Document boundingBox;
  nestedValue.SetObject();
  nestedValue.AddMember("width",bb.width(),boundingBox.GetAllocator());
  nestedValue.AddMember("height",bb.height(),boundingBox.GetAllocator());
  nestedValue.AddMember("depth",bb.depth(),boundingBox.GetAllocator());

  boundingBox.AddMember("dimensions-3D",nestedValue,boundingBox.GetAllocator());
  boundingBox.AddMember("size",input.size(),boundingBox.GetAllocator());
  boundingBox.AddMember("dist-to-plane",input.distanceToPlane(),boundingBox.GetAllocator());

  char* poseJson = NULL;
  std::sprintf(poseJson,"{\"header\":{\"seq\":%d,\"stamp\":{\"sec\":%d,\"nsec\":%d},\"frame_id\":\"%s\"},\"pose\":{\"position\":{\"x\":%f,\"y\":%f,\"z\":%f},\"orientation\":{\"x\":%f,\"y\":%f,\"z\":%f,\"w\":%f}}}",
                                pose.header.seq,pose.header.stamp.sec,pose.header.stamp.nsec,pose.header.frame_id.c_str(),
                                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
  rapidjson::Document poseJsonObj;
  poseJsonObj.Parse(poseJson);
  mergeJson(boundingBox,poseJsonObj,"pose");
  mergeJson(*object,boundingBox,"boundingBox");
}

void DesignatorWrapper::convert(rs::Shape &input, rapidjson::Document *object)
{
  object->AddMember("shape",input.shape(),object->GetAllocator());
}

void DesignatorWrapper::convert(rs::PoseAnnotation &input, rapidjson::Document *object)
{
  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose_stamped_msgs;
  rs::conversion::from(input.camera(), tf_stamped_pose);

  tf::StampedTransform transf(tf::Transform(tf_stamped_pose.getRotation(),tf_stamped_pose.getOrigin()),tf_stamped_pose.stamp_,tf_stamped_pose.frame_id_,"");
  transf.child_frame_id_ = "";
  geometry_msgs::TransformStamped transf_msg;

  tf::poseStampedTFToMsg(tf_stamped_pose, pose_stamped_msgs);
  tf::transformStampedTFToMsg(transf,transf_msg);

  rapidjson::Document nestedValue;
  //nestedValue.AddMember("transform",transf_stamped_msg,nestedValue.GetAllocator());
  nestedValue.AddMember("source",input.source(),nestedValue.GetAllocator());

  char* poseJson = NULL;
  std::sprintf(poseJson,"{\"header\":{\"seq\":%d,\"stamp\":{\"sec\":%d,\"nsec\":%d},\"frame_id\":\"%s\"},\"child_frame_id\":\"%s\",\"transform\":{\"translation\":{\"x\":%f,\"y\":%f,\"z\":%f},\"rotation\":{\"x\":%f,\"y\":%f,\"z\":%f,\"w\":%f}}}",
                                transf_msg.header.seq,transf_msg.header.stamp.sec,transf_msg.header.stamp.nsec,transf_msg.header.frame_id.c_str(),transf_msg.child_frame_id.c_str(),
                                transf_msg.transform.translation.x,transf_msg.transform.translation.y,transf_msg.transform.translation.z,transf_msg.transform.rotation.x,transf_msg.transform.rotation.y,transf_msg.transform.rotation.z,transf_msg.transform.rotation.w);
  rapidjson::Document poseJsonObj;
  poseJsonObj.Parse(poseJson);
  mergeJson(nestedValue,poseJsonObj,"transform");
  mergeJson(*object,nestedValue,"pose");
}

void DesignatorWrapper::convert(rs::SemanticColor &input, rapidjson::Document *object)
{
  const std::vector<std::string> &colors = input.color();
  const std::vector<float> &ratios = input.ratio();

  rapidjson::Value nestedValue;

  for(size_t i = 0; i < colors.size(); ++i)
  {
    const float &ratio = ratios[i];
    rapidjson::Value v(colors[i].c_str(),object->GetAllocator());
    nestedValue.AddMember(v,ratio,object->GetAllocator());
  }
  object->AddMember("color",nestedValue,object->GetAllocator());
}

void DesignatorWrapper::convert(rs::MLNAtoms &input, rapidjson::Document *object)
{
  const std::vector<std::string> &atoms = input.atoms();

  rapidjson::Document *nestedValue = new rapidjson::Document();

  rapidjson::Value atomArray;
  atomArray.SetArray();

  for(size_t i = 0; i < atoms.size(); ++i)
  {
    std::stringstream id;
    rapidjson::Value atom;
    atom.SetObject();
    id << "atom_ " << i;
    rapidjson::Value v(id.str().c_str(),nestedValue->GetAllocator());
    rapidjson::Value value(atoms[i],nestedValue->GetAllocator());
    atom.AddMember(v,value,nestedValue->GetAllocator());
    atomArray.PushBack(atom,nestedValue->GetAllocator());
  }
  nestedValue->AddMember("atom",atomArray,nestedValue->GetAllocator());
  mergeJson(*object,*nestedValue,"mln-atoms");
}

void DesignatorWrapper::convert(rs::NamedLink &input, rapidjson::Document *object)
{
  rapidjson::Value nestedValue;
  nestedValue.SetObject();
  nestedValue.AddMember("name",input.name(),object->GetAllocator());
  nestedValue.AddMember("url",input.url(),object->GetAllocator());
  object->AddMember("namedlink",nestedValue,object->GetAllocator());
}

void DesignatorWrapper::convert(rs::Goggles &input, rapidjson::Document *object)
{
  rapidjson::Document nestedValue;
  nestedValue.AddMember("category",input.category(),nestedValue.GetAllocator());
  nestedValue.AddMember("title",input.title(),nestedValue.GetAllocator());
  nestedValue.AddMember("preview-link",input.preview_link(),nestedValue.GetAllocator());

  std::vector<rs::NamedLink> namedlinks = input.links();
  rapidjson::Document *links = new rapidjson::Document();
  convertAll(namedlinks, links);

  mergeJson(nestedValue,*links,"namedlinks");
  mergeJson(*object,nestedValue,"goggles");
}

void DesignatorWrapper::convert(rs::Features &input, rapidjson::Document *object)
{
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

  object->AddMember("response",classes[(size_t)respM.at<float>(0)],object->GetAllocator());
}

void DesignatorWrapper::convert(rs::ClusterPart &input, rapidjson::Document *object)
{
  rapidjson::Document nestedValue;

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose);
  nestedValue.AddMember("name",input.name(),nestedValue.GetAllocator());

  char* poseJson = NULL;
  std::sprintf(poseJson,"{\"header\":{\"seq\":%d,\"stamp\":{\"sec\":%d,\"nsec\":%d},\"frame_id\":\"%s\"},\"pose\":{\"position\":{\"x\":%f,\"y\":%f,\"z\":%f},\"orientation\":{\"x\":%f,\"y\":%f,\"z\":%f,\"w\":%f}}}",
                                pose.header.seq,pose.header.stamp.sec,pose.header.stamp.nsec,pose.header.frame_id.c_str(),
                                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
  rapidjson::Document poseJsonObj;
  poseJsonObj.Parse(poseJson);
  mergeJson(nestedValue,poseJsonObj,"pose");
  mergeJson(*object,nestedValue,std::to_string(input.clID()));
}

void DesignatorWrapper::convert(rs_demos::Volume &input, rapidjson::Document *object)
{
  object->AddMember("volume",input.volume(),object->GetAllocator());
}

void DesignatorWrapper::convert(rs_demos::Substance &input, rapidjson::Document *object)
{
  rapidjson::Value substance;
  substance.SetObject();
  substance.AddMember("substance",input.substanceName(),object->GetAllocator());
  object->AddMember("contains",substance,object->GetAllocator());
}

iai_robosherlock_msgs::PerceivedObjects DesignatorWrapper::getObjectsMsgs()
{
  return objects_;
}

void DesignatorWrapper::convert(rs::ARMarker &input, rapidjson::Document &arDesignator)
{
  arDesignator.AddMember("type","armarker",arDesignator.GetAllocator());
  arDesignator.AddMember("id",input.name(),arDesignator.GetAllocator());

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose);

  char* poseJson = NULL;
  std::sprintf(poseJson,"{\"header\":{\"seq\":%d,\"stamp\":{\"sec\":%d,\"nsec\":%d},\"frame_id\":\"%s\"},\"pose\":{\"position\":{\"x\":%f,\"y\":%f,\"z\":%f},\"orientation\":{\"x\":%f,\"y\":%f,\"z\":%f,\"w\":%f}}}",
                                pose.header.seq,pose.header.stamp.sec,pose.header.stamp.nsec,pose.header.frame_id.c_str(),
                                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
  rapidjson::Document poseJsonObj;
  poseJsonObj.Parse(poseJson);
  mergeJson(arDesignator,poseJsonObj,"pose");
}

void DesignatorWrapper::convert(rs::HandleAnnotation &input,
                                rapidjson::Document &handleDesignator)
{
  handleDesignator.AddMember("handle", input.name(),handleDesignator.GetAllocator());
  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose);

  char* poseJson = NULL;
  std::sprintf(poseJson,"{\"header\":{\"seq\":%d,\"stamp\":{\"sec\":%d,\"nsec\":%d},\"frame_id\":\"%s\"},\"pose\":{\"position\":{\"x\":%f,\"y\":%f,\"z\":%f},\"orientation\":{\"x\":%f,\"y\":%f,\"z\":%f,\"w\":%f}}}",
                                pose.header.seq,pose.header.stamp.sec,pose.header.stamp.nsec,pose.header.frame_id.c_str(),
                                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
  rapidjson::Document poseJsonObj;
  poseJsonObj.Parse(poseJson);
  mergeJson(handleDesignator,poseJsonObj,"pose");
}

void DesignatorWrapper::convert(rs_demos::Pizza &input, rapidjson::Document *object)
{
  rapidjson::Document *pizza = new rapidjson::Document();

  rs::PoseAnnotation pose = input.pose();
  convert(pose, pizza);

  rapidjson::Value size;
  size.SetObject();
  size.AddMember("width", input.size().width(),pizza->GetAllocator());
  size.AddMember("height", input.size().height(),pizza->GetAllocator());
  pizza->AddMember("size",size,pizza->GetAllocator());

  rapidjson::Value gridSize;
  gridSize.SetObject();
  gridSize.AddMember("width", input.dimensions().width(),pizza->GetAllocator());
  gridSize.AddMember("height", input.dimensions().height(),pizza->GetAllocator());
  pizza->AddMember("dimensions",gridSize,pizza->GetAllocator());

  rapidjson::Value fieldSize;
  fieldSize.SetObject();
  fieldSize.AddMember("width", input.fieldSize().width(),pizza->GetAllocator());
  fieldSize.AddMember("height", input.fieldSize().height(),pizza->GetAllocator());
  pizza->AddMember("field-size",fieldSize,pizza->GetAllocator());

  rapidjson::Document *fields = new rapidjson::Document();
  std::vector<rs_demos::PizzaField> fieldsVec = input.fields();
  for(size_t i = 0; i < fieldsVec.size(); ++i)
  {
    rs_demos::PizzaField &inputF = fieldsVec[i];
    rapidjson::Document *field = new rapidjson::Document();

    rs::PoseAnnotation pose = inputF.pose();
    convert(pose, field);

    field->AddMember("x", inputF.position().x(),field->GetAllocator());
    field->AddMember("y", inputF.position().y(),field->GetAllocator());

    rapidjson::Document *ingredients = new rapidjson::Document();
    std::vector<std::string> ingredientsVec = inputF.ingredients();
    for(size_t j = 0; j < ingredientsVec.size(); ++j)
    {
      rapidjson::Value v(std::to_string(i).c_str(),ingredients->GetAllocator());
      ingredients->AddMember(v, ingredientsVec[j],ingredients->GetAllocator());
    }
    field->AddMember("top-ingredient", ingredientsVec[ingredientsVec.size() - 1],field->GetAllocator());
    mergeJson(*field,*ingredients,"ingredient");
    mergeJson(*fields,*field,std::to_string(i));
  }
  mergeJson(*pizza,*fields,"fields");
  mergeJson(*object,*pizza,"pizza");
}

template<>
void DesignatorWrapper::convertAll(std::vector<rs::ClusterPart> &all, rapidjson::Document *object)
{
  if(!all.empty())
  {
    rapidjson::Document *objParts = new rapidjson::Document();
    for(rs::ClusterPart input : all)
    {
      convert(input, objParts);
    }

    mergeJson(*object,*objParts,"obj-part");
  }
}

void DesignatorWrapper::mergeJson (rapidjson::Document &destination, rapidjson::Document &source, std::string fieldName)
{
  rapidjson::Value v(fieldName.c_str(),destination.GetAllocator());
  destination.AddMember(v,rapidjson::Value(),destination.GetAllocator());
  for(auto cpy = source.MemberBegin();cpy <= source.MemberEnd(); cpy++)
  {
    destination[fieldName].AddMember(cpy->name,cpy->value,destination.GetAllocator());
  }
}

std::string DesignatorWrapper::jsonToString(rapidjson::Document *res)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  res->Accept(writer);
  return buffer.GetString();
}
}
