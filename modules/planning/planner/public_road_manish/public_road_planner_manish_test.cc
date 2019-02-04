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

#include "modules/planning/planner/public_road_manish/public_road_planner_manish.h"

#include "gtest/gtest.h"

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

TEST(PublicRoadPlannerManishTest, Simple) {
  PublicRoadPlannerManish public_road_planner_manish;
  PlanningConfig config;
  EXPECT_EQ(public_road_planner_manish.Name(), "PUBLIC_ROAD_MANISH");
  EXPECT_EQ(public_road_planner_manish.Init(config), common::Status::OK());
}

}  // namespace planning
}  // namespace apollo
