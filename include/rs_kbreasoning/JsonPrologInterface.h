#ifndef JSONPROLOGINTERFACE_H
#define JSONPROLOGINTERFACE_H

//boost
#include <boost/algorithm/string.hpp>

//ros packages
#include <json_prolog/prolog.h>
#include <designators/Designator.h>
#include <rs/utils/output.h>

#include <rs_kbreasoning/KRDefinitions.h>


//functonality related to calling json_prolog from roboSherlock
class JsonPrologInterface
{

public:
  JsonPrologInterface()
  {
  }
  ~JsonPrologInterface()
  {
  }

  /*brief
   * in desig: query as designator
   * out prologQuery: the Prolog Query as a string
   * returns true or false /success or fail
   */
  bool buildPrologQueryFromDesignator(designator_integration::Designator *desig,
                                      std::string &prologQuery);

  /*brief
   * Create a vector of Annotator Names from the result of the knowrob_rs library.
   * This vector can be used as input for RSControledAnalysisEngine::setNextPipelineOrder
   */
  std::vector<std::string> createPipelineFromPrologResult(std::string result);

};

#endif //JSONPROLOGINTERFACE_H

