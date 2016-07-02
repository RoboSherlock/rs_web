#ifndef KRDEFINITIONS_H
#define KRDEFINITIONS_H

#include <map>
#include <string>


/* for now this is OK, but to scale a diferent strategy needs to be implemented
 * - main issue here is that classifiers are not trained on the names in knowrob so we need a mapping
 * - the query terms should be stored somewhere else to facilitate easy extenstion
*/
namespace rs_kbreasoning
{

std::map<std::string, std::string> krNameMapping =
{
  //superclasses
  {"DRINK", "knowrob:'Drink'"},
  {"FOODORDRINKORINGREDIENT", "knowrob:'FoodOrDrinkOrIngredient'"},
  {"CONTAINER", "knowrob:'Container'"},
  {"COOKING UTENSIL", "knowrob:'CookingUtensil'"},
  {"ELECTRICAL DEVICE", "knowrob:'ElectricalDevice'"},

  //objects
  {"icetea", "rs_test_objects:'PfannerIceTea'"},
  {"mondamin", "rs_test_objects:'MondaminPancakeMix'"},
  {"cereal", "rs_test_objects:'KellogsCornFlakes'"},
  {"plate", "rs_test_objects:'Plate'"},
  {"pancake_maker", "rs_test_objects:'PancakeMaker'"},
  {"spatula", "rs_test_objects:'Spatula'"},
  {"pitcher", "rs_test_objects:'Pitcher'"},
  {"milk", "rs_test_objects:'Milk'"},
  {"cup", "rs_test_objects:'Cup'"},
  {"bottle", "rs_test_objects:'KimaxBottle'"},
  {"bottle_acid", "rs_test_objects:'KimaxBottle'"},
  {"bottle_base", "rs_test_objects:'KimaxBottle'"},
  {"flask_250ml", "rs_test_objects:'Flask'"},
  {"flask_400ml", "rs_test_objects:'Flask'"},
  {"pipette", "rs_test_objects:'Pipette'"},
  {"mixer_ikamag", "rs_test_objects:'MixerIkaMag'"}
};

std::vector<std::string> rsQueryTerms =
  { "shape","volume","contains","color","size",
    "location","logo","text","product","obj-parts",
    "detection","type","handle"
  };
}

#endif // KRDEFINITIONS_H
