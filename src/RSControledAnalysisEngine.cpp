#include <rs_kbreasoning/RSControledAnalysisEngine.h>


void RSControledAnalysisEngine::setNextPipeline(std::vector<std::string> l)
{
  next_pipeline_order = l;
}

std::vector<std::string> &RSControledAnalysisEngine::getNextPipeline()
{
  return next_pipeline_order;
}

void RSControledAnalysisEngine::applyNextPipeline()
{
  if(rspm)
  {
    rspm->setPipelineOrdering(next_pipeline_order);
  }
}

RSControledAnalysisEngine::RSControledAnalysisEngine() : RSAnalysisEngine(),
  rspm(NULL),currentAEName("")
{
  process_mutex = boost::shared_ptr<std::mutex>(new std::mutex);
}

RSControledAnalysisEngine::~RSControledAnalysisEngine()
{
  if(cas)
  {
    delete cas;
    cas = NULL;
  }
  if(engine)
  {
    delete engine;
    engine = NULL;
  }

  if(rspm)
  {
    delete rspm;
    rspm = NULL;
  }
}

void RSControledAnalysisEngine::init(const std::string &file)
{
  uima::ErrorInfo errorInfo;

  size_t pos = file.rfind('/');
  outInfo("Creating analysis engine: " FG_BLUE << (pos == file.npos ? file : file.substr(pos)));

  engine = uima::Framework::createAnalysisEngine(file.c_str(), errorInfo);

  if(errorInfo.getErrorId() != UIMA_ERR_NONE)
  {
    outError("createAnalysisEngine failed." << errorInfo.asString());
    throw uima::Exception(errorInfo);
  }
  // RSPipelineManager rspm(engine);
  rspm = new RSPipelineManager(engine);
  std::vector<icu::UnicodeString> &non_const_nodes = rspm->getFlowConstraintNodes();

  outInfo("*** Fetch the FlowConstraint nodes. Size is: "  << non_const_nodes.size());
  for(int i = 0; i < non_const_nodes.size(); i++)
  {
    std::string tempString;
    non_const_nodes.at(i).toUTF8String(tempString);
    outInfo(tempString);
  }

  rspm->aengine->getNbrOfAnnotators();
  outInfo("*** Number of Annotators in AnnotatorManager: " << rspm->aengine->getNbrOfAnnotators());

  // After all annotators have been initialized, pick the default pipeline
  std::vector<std::string> default_pipeline;
  default_pipeline.push_back("CollectionReader");
  default_pipeline.push_back("ImagePreprocessor");
  default_pipeline.push_back("RegionFilter");
  default_pipeline.push_back("NormalEstimator");
  default_pipeline.push_back("PlaneAnnotator");
  default_pipeline.push_back("PointCloudClusterExtractor");
  default_pipeline.push_back("Cluster3DGeometryAnnotator");
  default_pipeline.push_back("ClusterTFLocationAnnotator");
  default_pipeline.push_back("SacModelAnnotator");
  default_pipeline.push_back("PrimitiveShapeAnnotator");
  default_pipeline.push_back("KBResultAdvertiser");;

  // removed color histogram for tests
  rspm->setDefaultPipelineOrdering(default_pipeline);

  // Get a new CAS
  outInfo("Creating a new CAS");
  cas = engine->newCAS();

  if(cas == NULL)
  {
    outError("Creating new CAS failed.");
    engine->destroy();
    delete engine;
    engine = NULL;
    throw uima::Exception(uima::ErrorMessage(UIMA_ERR_ENGINE_NO_CAS), UIMA_ERR_ENGINE_NO_CAS, uima::ErrorInfo::unrecoverable);
  }

  outInfo("initialization done: " << name << std::endl
          << std::endl << FG_YELLOW << "********************************************************************************" << std::endl);
  currentAEName = file;
}

void RSControledAnalysisEngine::process()
{
  designator_integration_msgs::DesignatorResponse d;
  process(d);
}

void RSControledAnalysisEngine::process(
  designator_integration_msgs::DesignatorResponse &designator_response,
  RSQuery *q)
{
  outInfo("executing analisys engine: " << name);
  try
  {
    UnicodeString ustrInputText;
    ustrInputText.fromUTF8(name);
    cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
    if(q != NULL)
    {
      rs::SceneCas sceneCas(*cas);
      rs::Query ts = rs::create<rs::Query>(*cas);
      if(q->timestamp != 0)
      {
        ts.timestamp.set(q->timestamp);
      }
      if(q->location != "")
      {
        ts.location.set(q->location);
      }
      if(q->objToInspect != "")
      {
        ts.inspect.set(q->objToInspect);
      }
      outInfo("setting in CAS: ts:" << q->timestamp << " location: " << q->location);
      sceneCas.set("QUERY", ts);
    }


    outInfo("processing CAS");
    uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);

    for(int i = 0; casIter.hasNext(); ++i)
    {
      uima::CAS &outCas = casIter.next();

      // release CAS
      outInfo("release CAS " << i);
      engine->getAnnotatorContext().releaseCAS(outCas);
    }

  }
  catch(const rs::Exception &e)
  {
    outError("Exception: " << std::endl << e.what());
  }
  catch(const uima::Exception &e)
  {
    outError("Exception: " << std::endl << e);
  }
  catch(const std::exception &e)
  {
    outError("Exception: " << std::endl << e.what());
  }
  catch(...)
  {
    outError("Unknown exception!");
  }
  // Make a designator from the result
  rs::DesignatorWrapper dw;
  dw.setMode(rs::DesignatorWrapper::CLUSTER);
  dw.setCAS(cas);

  designator_response = dw.getDesignatorResponseMsg();
  cas->reset();
  outInfo("processing finished");
}

// Call process() and decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process( bool reset_pipeline_after_process, designator_integration_msgs::DesignatorResponse &designator_response)
{
  process_mutex->lock();
  outInfo(FG_CYAN << "process(bool,desig) - LOCK OBTAINED");
  process(designator_response, 0);
  if(reset_pipeline_after_process)
  {
    resetPipelineOrdering();  // reset pipeline to default
  }
  process_mutex->unlock();
  outInfo(FG_CYAN << "process(bool,desig) - LOCK RELEASED");
}

// Call process() and decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(bool reset_pipeline_after_process)
{
  designator_integration_msgs::DesignatorResponse d;
  process(reset_pipeline_after_process, d);
}

// Define a pipeline that should be executed,
// process(reset_pipeline_after_process) everything and
// decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(std::vector<std::string> annotators,
                                        bool reset_pipeline_after_process,
                                        designator_integration_msgs::DesignatorResponse &designator_response,
                                        RSQuery *query)
{
  process_mutex->lock();
  outInfo(FG_CYAN << "process(std::vector, bool) - LOCK OBTAINED");
  setNextPipeline(annotators);
  applyNextPipeline();

  // Process the modified pipeline
  process(designator_response, query);
  if(reset_pipeline_after_process)
  {
    resetPipelineOrdering();  // reset pipeline to default
  }
  process_mutex->unlock();
  outInfo(FG_CYAN << "process(std::vector, bool) - LOCK RELEASED");
}

// Define a pipeline that should be executed,
// process(reset_pipeline_after_process) everything and
// decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(std::vector<std::string> annotators, bool reset_pipeline_after_process)
{
  designator_integration_msgs::DesignatorResponse d;
  process(annotators, reset_pipeline_after_process, d);
}
