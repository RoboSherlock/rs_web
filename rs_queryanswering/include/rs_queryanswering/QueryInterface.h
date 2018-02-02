//json
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include <vector>
#include <rs/utils/output.h>

#include <rs_queryanswering/RSControledAnalysisEngine.h>
#include <rs_queryanswering/PrologInterface.h>

#include <json_prolog/prolog.h>


#include <std_srvs/Trigger.h>


class QueryInterface
{
private:
    std::map<std::string,std::string> thorinObjects_;
    PrologInterface* prologInterface;


public:

  enum QueryType{NONE, INSPECT, DETECT};

  RSControledAnalysisEngine* engine_;
  rapidjson::Document query;

  QueryInterface(bool withJsonProlog){
      query = rapidjson::Document(rapidjson::kObjectType);
      prologInterface = new PrologInterface(withJsonProlog);
  }
  ~QueryInterface()
  {
  }

  bool parseQuery(std::string query);

  QueryType processQuery(std::vector<std::string> &newPipelineOrder);

  bool handleDetect(std::vector<std::string> &newPipelineOrder);
  bool handleInspect(std::vector<std::string> &newPipelineOrder);

//  bool designatorCallbackLogic(std::string &req, std::vector<std::string> &res);

  //bool handleQuery(std::string *req,
  //                 std::vector<std::string> &resp);

  void filterResults(std::string &requestDesignator,
                     std::vector<std::string> &resultDesignators,
                     std::vector<std::string> &filteredResponse,
                     std::vector<bool> &designatorsToKeep,
                     const std::string superclass);

  std::string toJson(const tf::StampedTransform &pose, std::string OID, std::string type);
  std::string getObjectByID(std::string OID, std::string type);

};
