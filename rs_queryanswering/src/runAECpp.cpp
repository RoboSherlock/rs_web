/*------------------------------------------------------------------------

 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.

 --------------------------------------------------------------------------

 Test driver that reads text files or XCASs or XMIs and calls the annotator

 -------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <chrono>
#include <condition_variable>
#include <sstream>

#include <uima/api.hpp>
#include "uima/internal_aggregate_engine.hpp"
#include "uima/annotator_mgr.hpp"

#include <rs/utils/RSPipelineManager.h>
#include <rs_queryanswering/KRDefinitions.h>
#include <rs_queryanswering/RSProcessManager.h>

#include <ros/ros.h>
#include <ros/package.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG


/**
 * Error output if program is called with wrong parameter.
 */

void help()
{
  std::cout << "Usage : rosrun rs_queryanswering run [options] [analysis_engine]" << std::endl
            << "Options : " << std::endl
            << "              _ae : = engine1   analysis_engine to run" << std::endl
            << "              _vis : = true | false     use robosherlock visualization" << std::endl
            << "              _wait : = true | false    enable or disable pervasiveness" << std::endl
            << "              _cwa : = true | false    enable or disable pervasiveness" << std::endl
            << std::endl;
}


/* ----------------------------------------------------------------------- */
/*       Main                                                              */
/* ----------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    help();
    return 1;
  }

  ros::init(argc, argv, std::string("RoboSherlock_") + getenv("USER"));
  ros::NodeHandle nh("~");

  std::string analysisEnginesName, analysisEngineFile;
  bool useVisualizer, waitForServiceCall, useCWAssumption, useObjIDRes=true,
	withJsonProlog=true;

  nh.param("ae", analysisEnginesName, std::string(""));
  nh.param("wait",waitForServiceCall, false);
  nh.param("vis", useVisualizer, false);
  nh.param("withJsonProlog", withJsonProlog, false);
  nh.param("cwa", useCWAssumption, false);

  if(withJsonProlog) {ros::service::waitForService("/json_prolog/simple_query");}

  rs::common::getAEPaths(analysisEnginesName, analysisEngineFile);

  if(analysisEngineFile.empty())
  {
    outError("analysis   engine \"" << analysisEngineFile << "\" not found.");
    return -1;
  }
  else
  {
    outInfo(analysisEngineFile);
  }

  std::string configFile = ros::package::getPath("rs_queryanswering") + "/config/config.yaml";

  try
  {
    RSProcessManager manager(useVisualizer, waitForServiceCall, useCWAssumption, nh);
    manager.setUseIdentityResolution(useObjIDRes);
    manager.setUseJsonPrologInterface(withJsonProlog);
    manager.pause();
    manager.init(analysisEngineFile, configFile);
    manager.run();
    manager.stop();
  }
  catch(const rs::Exception &e)
  {
    outError("Exception: " << std::endl << e.what());
    return -1;
  }
  catch(const uima::Exception &e)
  {
    outError("Exception: " << std::endl << e);
    return -1;
  }
  catch(const std::exception &e)
  {
    outError("Exception: " << std::endl << e.what());
    return -1;
  }
  catch(...)
  {
    outError("Unknown exception!");
    return -1;
  }
  return 0;
}
