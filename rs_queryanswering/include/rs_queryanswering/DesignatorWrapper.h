#pragma once
#ifndef DESIGNATORSTORAGE_H
#define DESIGNATORSTORAGE_H


#include <uima/api.hpp>

#include <rs/scene_cas.h>
#include <rs/utils/output.h>

#include <rs/types/all_types.h>
#include <rs_demos/types/acat_types.h>
#include <rs_demos/types/robohow_types.h>

#include <designators/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <map>

#include <iai_robosherlock_msgs/PerceivedObjects.h>

namespace rs
{

class DesignatorWrapper
{
public:

  enum DesignatorProcessMode
  {
    CLUSTER = 0,
    OBJECT
  };

  DesignatorProcessMode mode;
  // Pointer to the one persistent object
  static designator_integration::Designator *req_designator;
  static designator_integration::Designator *res_designator;

  designator_integration_msgs::DesignatorResponse res;
  iai_robosherlock_msgs::PerceivedObjects objects_;

  uint64_t now;
  uima::CAS *tcas;

  DesignatorWrapper();

  DesignatorWrapper(uima::CAS *cas);

  void setCAS(uima::CAS *cas);
  void setMode(DesignatorProcessMode m);

  virtual ~DesignatorWrapper();

  void filterClusters(const std::vector<rs::Cluster> input, const designator_integration::Designator *out) const;
  void updateDesignator();

  //  void notifyObserversDesignatorAdded(Designator d);

  designator_integration_msgs::DesignatorResponse getDesignatorResponseMsg();
  iai_robosherlock_msgs::PerceivedObjects getObjectsMsgs();

  bool getObjectDesignators(std::vector<designator_integration::Designator> &);

  template<class T>
  void process(std::vector<T> &elements, std::vector<designator_integration::Designator> &objectDesignators)
  {
    objects_.objects.clear();
    for(size_t i = 0; i < elements.size(); ++i)
    {
      outDebug("reading object: " << i);

      T &element = elements[i];
      designator_integration::Designator objectDesignator;

      double seconds = now / 1000000000;
      double nsec = (now % 1000000000) / 1000000000.0;

      outDebug("Seconds: " << std::setprecision(12) << seconds);
      outDebug("NanoSecs: " << std::setprecision(12) << nsec);
      double time = seconds + nsec;

      outDebug("time: " << std::setprecision(20) << time);
      objectDesignator.setValue("timestamp", time);

      convert(element, i, &objectDesignator);

      std::vector<rs::Geometry> geometry;
      std::vector<rs::PoseAnnotation> poses;
      std::vector<rs::Detection> detections;
      std::vector<rs::SemanticColor> semanticColors;
      std::vector<rs::Shape> shapes;
      std::vector<rs::TFLocation> locations;
      std::vector<rs::MLNAtoms> atoms;
      std::vector<rs::Goggles> goggles;
      std::vector<rs::Features> features;
      std::vector<rs::ClusterPart> clusterParts;
      std::vector<rs_demos::Volume> volume;
      std::vector<rs_demos::Substance> substance;
      std::vector<rs_demos::Pizza> pizza;

      element.annotations.filter(geometry);
      element.annotations.filter(poses);
      element.annotations.filter(detections);
      element.annotations.filter(semanticColors);
      element.annotations.filter(shapes);
      element.annotations.filter(locations);
      element.annotations.filter(atoms);
      element.annotations.filter(goggles);
      element.annotations.filter(features);
      element.annotations.filter(clusterParts);
      element.annotations.filter(volume);
      element.annotations.filter(substance);
      element.annotations.filter(pizza);

      outDebug("Number of volume annotations: " << volume.size());
      outDebug("Number of substance annotations: " << substance.size());

      convertAll(geometry, &objectDesignator);
      convertAll(detections, &objectDesignator);
      convertAll(poses, &objectDesignator);
      convertAll(semanticColors, &objectDesignator);
      convertAll(shapes, &objectDesignator);
      convertAll(locations, &objectDesignator);
      convertAll(atoms, &objectDesignator);
      convertAll(goggles, &objectDesignator);
      convertAll(features, &objectDesignator);
      convertAll(clusterParts, &objectDesignator);
      convertAll(volume, &objectDesignator);
      convertAll(substance, &objectDesignator);
      convertAll(pizza, &objectDesignator);


      iai_robosherlock_msgs::PerceivedObject object;
      if(!detections.empty() && !poses.empty())
      {
        rs::Detection a = detections[0];
        rs::PoseAnnotation b = poses[0];
        object.id = a.name();
        tf::Stamped<tf::Pose> pose;
        rs::conversion::from(b.camera(), pose);
        geometry_msgs::PoseStamped pose_stamped_msgs;

        tf::poseStampedTFToMsg(pose, pose_stamped_msgs);
        object.transform = pose_stamped_msgs;
        objects_.objects.push_back(object);
      }

      outDebug("no. of children: " << objectDesignator.children().size());
      if(objectDesignator.children().size() > 0)
      {
        objectDesignators.push_back(objectDesignator);
      }
    }
  }

  // Converter methods to take scene annotations and map them to Designator Keywords
  void convert(rs::Cluster &input, const size_t id, designator_integration::KeyValuePair *object);
  void convert(rs::Object &input, const size_t id, designator_integration::KeyValuePair *object);

  // TODO How can i define this in the cpp file?
  // If i do it naively, i get a runtime error that the method can't be found + the mangeled method name
  template<class T>
  void convertAll(std::vector<T> &all, designator_integration::KeyValuePair *object)
  {
    for(T input : all)
    {
      convert(input, object);
    }
  }

  void convert(rs::Detection &input, designator_integration::KeyValuePair *object);
  void convert(rs::TFLocation &input, designator_integration::KeyValuePair *object);
  void convert(rs::Segment &input, designator_integration::KeyValuePair *object);
  void convert(rs::Geometry &input, designator_integration::KeyValuePair *object);
  void convert(rs::Shape &input, designator_integration::KeyValuePair *object);
  void convert(rs::PoseAnnotation &input, designator_integration::KeyValuePair *object);
  void convert(rs::SemanticColor &input, designator_integration::KeyValuePair *object);
  void convert(rs::MLNAtoms &input, designator_integration::KeyValuePair *object);
  void convert(rs::NamedLink &input, designator_integration::KeyValuePair *object);
  void convert(rs::Goggles &input, designator_integration::KeyValuePair *object);
  void convert(rs::Features &input, designator_integration::KeyValuePair *object);
  void convert(rs::ClusterPart &input, designator_integration::KeyValuePair *object);
  void convert(rs_demos::Volume &input, designator_integration::KeyValuePair *object);
  void convert(rs_demos::Substance &input, designator_integration::KeyValuePair *object);

  void convert(rs::ARMarker &input, designator_integration::Designator &res);
  void convert(rs::HandleAnnotation &input, designator_integration::Designator &res);
  void convert(rs_demos::Pizza &input, designator_integration::KeyValuePair *object);

};

}

#endif // DESIGNATORSTORAGE_H
