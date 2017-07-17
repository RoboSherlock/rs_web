#ifndef KRDEFINITIONS_H
#define KRDEFINITIONS_H

#include <map>
#include <vector>
#include <string>
#include <sstream>


/* for now this is OK, but to scale a diferent strategy needs to be implemented
 * - main issue here is that classifiers are not trained on the names in knowrob so we need a mapping
 * - the query terms should be stored somewhere else to facilitate easy extenstion
*/
namespace rs_queryanswering
{

static std::map<std::string, std::string> krNameMapping =
{
  //superclasses
  {"Drink", "knowrob:'Drink'"},
  {"FoodOrDrinkOrIngredient", "knowrob:'FoodOrDrinkOrIngredient'"},
  {"Container", "knowrob:'Container'"},
  {"CookingUtensil", "knowrob:'CookingUtensil'"},
  {"ElectricalDevice", "knowrob:'ElectricalDevice'"},
  {"Cutlery", "rs_objects:'Cutlery'"},
  {"Fork", "rs_objects:'Fork'"},
  {"Knife", "rs_objects:'Knife'"},
  {"Cup", "rs_objects:'Cup'"},
  {"Bottle", "knowrob:'Bottle'"},
  {"Plate", "rs_objects:'Plate'"},
  {"plate", "rs_objects:'Plate'"},


  //OBJECTS:
  //kitchen should solve this with some conversion and naming guideline
  {"icetea", "rs_objects:'PfannerPfirschIcetea'"},
  {"mondamin", "rs_objects:'Mondamin'"},
  {"cereal", "rs_objects:'KellogsCornFlakes'"},
  {"red_spotted_plate", "rs_objects:'RedSpottedPlate'"},
  {"pancake_maker", "rs_objects:'PancakeMaker'"},
  {"spatula", "rs_objects:'FryingSpatula'"},
  {"pitcher", "rs_objects:'YellowPitcher'"},
  {"milk", "rs_objects:'VollMilch'"},
  {"cup", "rs_objects:'BlueCup'"},
  {"albi_himbeer_juice", "rs_objects:'AlbiHimbeerJuice'"},
  {"black_bowl", "rs_objects:'BlackBowl'"},
  {"blue_cup", "rs_objects:'BlueCup'"},
  {"blue_plastic_bowl", "rs_objects:'BluePlasticBowl'"},
  {"blue_spotted_plate", "rs_objects:'BlueSpottedPlate'"},
  {"cappuccino", "rs_objects:'Cappuccino'"},
  {"coffee_el_bryg", "rs_objects:'CoffeeElBryg'"},
  {"fork_blue_plastic", "rs_objects:'ForkBluePlastic'"},
  {"fork_red_plastic", "rs_objects:'ForkRedPlastic'"},
  {"fork_ycb", "rs_objects:'ForkYcb'"},
  {"frying_pan", "rs_objects:'FryingPan'"},
  {"frying_spatula", "rs_objects:'FryingSpatula'"},
  {"hela_curry_ketchup", "rs_objects:'HelaCurryKetchup'"},
  {"ja_milch", "rs_objects:'JaMilch'"},
  {"jod_salz", "rs_objects:'JodSalz'"},
  {"kelloggs_corn_flakes", "rs_objects:'KelloggsCornFlakes'"},
  {"kelloggs_toppas_mini", "rs_objects:'KellogsToppasMini'"},
  {"knife_blue_plastic", "rs_objects:'KnifeBluePlastic'"},
  {"knife_red_plastic", "rs_objects:'KnifeRedPlastic'"},
  {"knife_ycb", "rs_objects:'KnifeYcb'"},
  {"knusper_schoko_keks", "rs_objects:'KnusperSchokoKeks'"},
  {"large_grey_spoon", "rs_objects:'LargeGreySpoon'"},
  {"linux_cup", "rs_objects:'LinuxCup'"},
  {"lion_cereal", "rs_objects:'LionCereal'"},
  {"marken_salz", "rs_objects:'MarkenSalz'"},
  {"meer_salz", "rs_objects:'MeerSalz'"},
  {"mondamin", "rs_objects:'Mondamin'"},
  {"nesquik_cereal", "rs_objects:'NesquikCereal'"},
  {"pancake_maker", "rs_objects:'PancakeMaker'"},
  {"pfanner_grune_icetea", "rs_objects:'PfannerGruneIcetea'"},
  {"pfanner_pfirsch_icetea", "rs_objects:'PfannerPfirschIcetea'"},
  {"pitcher_white", "rs_objects:'PitcherWhite'"},
  {"pitcher_yellow", "rs_objects:'PitcherYellow'"},
  {"pot", "rs_objects:'Pot'"},
  {"pringles_paprika", "rs_objects:'PringlesPaptrika'"},
  {"pringles_salt", "rs_objects:'PringlesSalt'"},
  {"pringles_vinegar", "rs_objects:'PringlesVinegar'"},
  {"red_spotted_bowl", "rs_objects:'RedSpottedBowl'"},
  {"red_spotted_cup", "rs_objects:'RedSpottedCup'"},
  {"red_spotted_plate", "rs_objects:'RedSpottedPlate'"},
  {"reine_butter_milch", "rs_objects:'ReineButterMilch'"},
  {"slotted_spatula", "rs_objects:'SlottedSpatula'"},
  {"soja_milch", "rs_objects:'SojaMilch'"},
  {"spitzen_reis", "rs_objects:'SpitzenReis'"},
  {"toaster", "rs_objects:'Toaster'"},
  {"tomato_al_gusto_basilikum", "rs_objects:'TomatoAlGustoBasilikum'"},
  {"tomato_sauce_oro_di_parma", "rs_objects:'TomatoSauceOroDiParma'"},
  {"voll_milch", "rs_objects:'VollMilch'"},
  {"white_bowl", "rs_objects:'WhiteBowl'"},
  {"yellow_plate", "rs_objects:'YellowPlate'"},
  {"cup_eco_orange", "rs_objects:'CupEcoOrange'"},
  {"blue_camping_cup", "rs_objects:'BlueCampingCup'"},
  {"sigg_bottle", "rs_objects:'SiggBottle'"},
  {"white_bottle", "rs_objects:'WhiteBottle'"},

  //Chemlab
  {"bottle_kimax", "rs_objects:'KimaxBottle'"},
  {"bottle_acid", "rs_objects:'KimaxBottle'"},
  {"bottle_base", "rs_objects:'KimaxBottle'"},
  {"flask_250ml", "rs_objects:'Flask'"},
  {"flask_400ml", "rs_objects:'Flask'"},
  {"pipette", "rs_objects:'Pipette'"},
  {"mixer_ikamag", "rs_objects:'MixerIkaMag'"},


  //assembly
  {"ChassisHolder","'http://knowrob.org/kb/thorin_simulation.owl#ChassisHolder'"},
  {"AxleHolder","'http://knowrob.org/kb/thorin_simulation.owl#AxleHolder'"},
  {"AccessoryHolder","'http://knowrob.org/kb/thorin_simulation.owl#AccessoryHolder'"},
  {"CamaroBody","'http://knowrob.org/kb/thorin_simulation.owl#CamaroBody'"}
};



static std::vector<std::string> rsQueryTerms =
{
  "shape", "volume", "contains", "color", "size",
  "location", "logo", "text", "product", "obj-part",
  "detection", "type", "handle", "ingredient", "cad-model", "inspect"
};


//make uri from namespace and class name
static std::string makeUri(const std::string &a)
{
  std::string prefix("http://knowrob.org/kb/");
  int tokenizerIdx = a.find(":");
  std::string begin = a.substr(0, tokenizerIdx);
  std::string end = a.substr(tokenizerIdx+1, a.length()-tokenizerIdx);
  //remove quoatation marks;
  end.erase(end.end()-1);
  end.erase(end.begin());

  std::stringstream stream;
  stream<<prefix<<begin<<".owl#"<<end;
  return stream.str();
}

}//end of namespace


#endif // KRDEFINITIONS_H
