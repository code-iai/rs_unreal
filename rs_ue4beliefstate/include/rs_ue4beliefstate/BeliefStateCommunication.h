/**
 * Copyright 2019 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Patrick Mania <pmania@cs.uni-bremen.de>
 *            Franklin Kenghagho Kenfack <fkenghag@uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __BELIEF_STATE_COMMUNICATION_H
#define __BELIEF_STATE_COMMUNICATION_H

#include "ros/ros.h"
#include "world_control_msgs/SpawnModel.h"
#include "world_control_msgs/SetModelPose.h"
#include "world_control_msgs/DeleteModel.h"
#include <geometry_msgs/Pose.h>

// Interface between RoboSherlock and UE4 to setup belief states 
// with the UROSWorldControl Plugin provided by robcog-iai
class BeliefStateCommunication
{

private:
    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::ServiceClient delete_client;
    std::vector<std::string> episodic_memory={};
public:
  BeliefStateCommunication(std::string domain="pie_rwc/spawn_model");
  BeliefStateCommunication(ros::NodeHandle &nh);
  ~BeliefStateCommunication();

  // TODO:
  // - Shall we set the ID for spawning objects?
  //   In the end we have to keep track of objects anyway with Identity Resolution etc. and might even have an ID for knowrob
  // - Where can we get the Mesh names for the 3d models in the BS?
  //   In the Ontology we do have cad model links, but we have to ensure that the naming matches the UE4 content folder
  
  // Sets the virtual robot camera to a new pose
  // This requires a Semlog ID Tag on the camera with the ID
  // urobovision_camera.
  // The full tag should therefore look like this:
  //  SemLog;id,urobovision_camera; 
  bool SetCameraPose(world_control_msgs::SetModelPose pose);
  bool SetCameraPose(geometry_msgs::Pose p);
  bool DeleteObject(world_control_msgs::DeleteModel object_id);
  bool SetObjectPose(world_control_msgs::SetModelPose pose);
  bool SpawnObject(world_control_msgs::SpawnModel model);
  void rsToUE4ModelMap(world_control_msgs::SpawnModel& model);
  bool deleteEpisodicMemory();
  bool updateEpisodicMemory(std::string object_id);
};

#endif // __BELIEF_STATE_COMMUNICATION_H
