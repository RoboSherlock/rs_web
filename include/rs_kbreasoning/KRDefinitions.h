#ifndef KRDEFINITIONS_H
#define KRDEFINITIONS_H

#include <map>
#include <vector>
#include <string>


/* for now this is OK, but to scale a diferent strategy needs to be implemented
 * - main issue here is that classifiers are not trained on the names in knowrob so we need a mapping
 * - the query terms should be stored somewhere else to facilitate easy extenstion
*/
namespace rs_kbreasoning
{

static std::map<std::string, std::string> krNameMapping =
{
  //superclasses
  {"DRINK", "knowrob:'Drink'"},
  {"FOODORDRINKORINGREDIENT", "knowrob:'FoodOrDrinkOrIngredient'"},
  {"CONTAINER", "knowrob:'Container'"},
  {"COOKING UTENSIL", "knowrob:'CookingUtensil'"},
  {"ELECTRICAL DEVICE", "knowrob:'ElectricalDevice'"},

  //objects
  {"icetea", "rs_objects:'PfannerIceTea'"},
  {"mondamin", "rs_objects:'MondaminPancakeMix'"},
  {"cereal", "rs_objects:'KellogsCornFlakes'"},
  {"plate", "rs_objects:'Plate'"},
  {"pancake_maker", "rs_objects:'PancakeMaker'"},
  {"spatula", "rs_objects:'Spatula'"},
  {"pitcher", "rs_objects:'Pitcher'"},
  {"milk", "rs_objects:'Milk'"},
  {"cup", "rs_objects:'Cup'"},
  {"bottle", "rs_objects:'KimaxBottle'"},
  {"bottle_acid", "rs_objects:'KimaxBottle'"},
  {"bottle_base", "rs_objects:'KimaxBottle'"},
  {"flask_250ml", "rs_objects:'Flask'"},
  {"flask_400ml", "rs_objects:'Flask'"},
  {"pipette", "rs_objects:'Pipette'"},
  {"mixer_ikamag", "rs_objects:'MixerIkaMag'"}
};

static std::vector<std::string> rsQueryTerms =
  { "shape","volume","contains","color","size",
    "location","logo","text","product","obj-parts",
    "detection","type","handle","ingredient"
  };
}

#endif // KRDEFINITIONS_H
