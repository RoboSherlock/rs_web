#include <rs_queryanswering/JsonPrologInterface.h>


bool JsonPrologInterface::buildPrologQueryFromDesignator(designator_integration::Designator *desig,
    std::string &prologQuery)
{
  if(!desig)
  {
    outError("NULL POINTER PASSED TO buildPrologQueryFromDesignator");
    return false;
  }

  if(desig->childForKey("OBJECT"))
  {
    // If the designator contains a "type" key, the highlevel is looking for a specific object of Type XY.
    // Use the corresponding Prolog Rule for object pipeline generation
    prologQuery = "build_pipeline_for_object('";
    // Fetch the accepted predicates from the Designator
    prologQuery += desig->childForKey("OBJECT")->stringValue();
    prologQuery += "', A)";
  }
  else
  {
    prologQuery = "build_single_pipeline_from_predicates([";
    std::vector<std::string> queriedKeys;

    // Fetch the keys from the Designator
    std::list<std::string> allKeys = desig->keys();

    //add the ones that are interpretable to the queriedKeys;
    for(const auto key : allKeys)
    {
      if(std::find(rs_queryanswering::rsQueryTerms.begin(), rs_queryanswering::rsQueryTerms.end(), boost::to_lower_copy(key)) != std::end(rs_queryanswering::rsQueryTerms))
      {
        queriedKeys.push_back(boost::to_lower_copy(key));
      }
      else
      {
        outWarn(key << " is not a valid query-language term");
      }
    }

    //leave this for now: TODO: need to handle also values
    if(desig->childForKey("DETECTION"))
    {
      designator_integration::KeyValuePair *kvp = desig->childForKey("DETECTION");
      if(kvp->stringValue() == "PANCAKE"||kvp->stringValue() == "pancake")
      {
        queriedKeys.push_back("pancakedetector");
      }
    }
    for(int i = 0; i < queriedKeys.size(); i++)
    {
      prologQuery += queriedKeys.at(i);
      if(i < queriedKeys.size() - 1)
      {
        prologQuery += ",";
      }
    }
    prologQuery += "], A)";
  }
  return true;
}
std::vector< std::string > JsonPrologInterface::createPipelineFromPrologResult(std::string queryResult)
{
  std::vector<std::string> new_pipeline;

  // Strip the braces from the result
  queryResult.erase(queryResult.end() - 1);
  queryResult.erase(queryResult.begin());

  std::stringstream resultstream(queryResult);

  std::string token;
  while(std::getline(resultstream, token, ','))
  {
    // erase leading whitespaces
    token.erase(token.begin(), std::find_if(token.begin(), token.end(), std::bind1st(std::not_equal_to<char>(), ' ')));
    outDebug("Planned Annotator by Prolog Planner " << token);

    // From the extracted tokens, remove the prefix
    std::string prefix("http://knowrob.org/kb/rs_components.owl#");
    int prefix_length = prefix.length();

    // Erase by length, to avoid string comparison
    token.erase(0, prefix_length);
    // outInfo("Annotator name sanitized: " << token );

    new_pipeline.push_back(token);
  }
  return new_pipeline;
}
