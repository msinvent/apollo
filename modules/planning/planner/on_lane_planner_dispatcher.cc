/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planner/on_lane_planner_dispatcher.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

std::unique_ptr<Planner> OnLanePlannerDispatcher::DispatchPlanner() {
  PlanningConfig planning_config;
  ADEBUG << "DEBUG_MS : DispatchPlanner <<"<<FLAGS_planning_config_file<<","
  		<<FLAGS_open_space_planner_switchable;
  apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                         &planning_config);
  // DEBUG_MS : if this is set to true true std planning switch to open space planner
  // DEBUG_MS : when close enough to target parking spot

  // We are accessing planning_config.pb.txt
	// DEBUG_MS : planner_type(1) == PUBLIC_ROAD
	// DEBUG_MS : planner_type(2) == OPEN_SPACE
	//  standard_planning_config {
	//    planner_type: PUBLIC_ROAD
	//    planner_type: OPEN_SPACE
	//    planner_public_road_config {
	//       scenario_type: LANE_FOLLOW
	//       scenario_type: SIDE_PASS
	//       scenario_type: STOP_SIGN_UNPROTECTED
	//    }
	//  }

  // DEBUG_MS
  if(FLAGS_play_copied_public_road_planner){
  	return planner_factory_.CreateObject(
  	        planning_config.standard_planning_config().planner_type(3));
  }
  if (FLAGS_open_space_planner_switchable) {
    return planner_factory_.CreateObject(
        planning_config.standard_planning_config().planner_type(1));
  }
  return planner_factory_.CreateObject(
      planning_config.standard_planning_config().planner_type(0));
}

}  // namespace planning
}  // namespace apollo
