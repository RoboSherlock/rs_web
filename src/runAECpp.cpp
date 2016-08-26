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
#include <rs_kbreasoning/KRDefinitions.h>
#include <rs_kbreasoning/RSControledAnalysisEngine.h>
#include <rs_kbreasoning/RSControledAEManager.h>

#include <ros/ros.h>
#include <ros/package.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG


/**
 * Error output if program is called with wrong parameter.
 */
void help()
{
  std::cout << "Usage: runAECpp [options] analysisEngine.xml [...]" << std::endl
            << "Options:" << std::endl
            << "  -wait If using piepline set this to wait for a service call" << std::endl
            << "  -cwa use the list of objects from [pkg_path]/config/config.yaml to set a closed world assumption"
            << "  -visualizer  Enable visualization" << std::endl
            << "  -save PATH   Path for storing images" << std::endl;
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

  std::vector<std::string> args;
  args.resize(argc - 1);
  for(int argI = 1; argI < argc; ++argI)
  {
    args[argI - 1] = argv[argI];
  }
  bool useVisualizer = false;
  bool waitForServiceCall = false;
  bool useCWAssumption = false;
  bool useObjIDRes = false;
  std::string savePath = getenv("HOME");

  size_t argO = 0;
  for(size_t argI = 0; argI < args.size(); ++argI)
  {
    const std::string &arg = args[argI];

    if(arg == "-visualizer")
    {
      useVisualizer = true;
    }
    else if(arg == "-wait")
    {
      waitForServiceCall = true;
    }
    else if(arg == "-cwa")
    {
      useCWAssumption = true;
    }
    else if(arg == "-idres")
    {
      useObjIDRes = true;
    }
    else if(arg == "-save")
    {
      if(++argI < args.size())
      {
        savePath = args[argI];
      }
      else
      {
        outError("No save path defined!");
        return -1;
      }
    }
    else
    {
      args[argO] = args[argI];
      ++argO;
    }
  }
  args.resize(argO);

  struct stat fileStat;
  if(stat(savePath.c_str(), &fileStat) || !S_ISDIR(fileStat.st_mode))
  {
    outError("Save path \"" << savePath << "\" does not exist.");
    return -1;
  }

  std::string analysisEngineFile;
  for(int argI = 0; argI < args.size(); ++argI)
  {
    const std::string &arg = args[0];

    if(!rs::common::getAEPaths(arg, analysisEngineFile))
    {
      outError("analysis engine \"" << arg << "\" not found.");
      return -1;
    }
  }
  outInfo(analysisEngineFile);
  std::string configFile = ros::package::getPath("rs_kbreasoning") +"/config/config.yaml";
  ros::NodeHandle n("~");
  try
  {
    RSControledAEManager manager(useVisualizer, savePath, waitForServiceCall,useCWAssumption, n);
    manager.setUseIdentityResolution(useObjIDRes);
    manager.init(analysisEngineFile,configFile);
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
