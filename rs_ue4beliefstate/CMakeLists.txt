cmake_minimum_required(VERSION 2.8.3)
project(rs_ue4beliefstate)
find_package(catkin REQUIRED robosherlock world_control_msgs roscpp)

################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs_ue4beliefstate)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/rs_ue4beliefstate/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)

################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)

################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)
catkin_package(
   CFG_EXTRAS ${PROJECT_NAME}_config.cmake
   )

################################################################################
## Includes                                                                   ##
################################################################################
include_directories(
        include
        ${RS_INCLUDE_DIRS_LIST}
        ${catkin_INCLUDE_DIRS}
        )

################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################
## generate classes from the typesystem xml files
generate_type_system(robosherlock)
#find all relevant files
find_additional_files()


################################################################################
## Objects                                                                    ##
################################################################################
add_executable(Spawner src/Spawner.cpp)
target_link_libraries(Spawner ${catkin_LIBRARIES} rs_beliefstatecomm)

add_executable(CameraUpdater src/CameraUpdater.cpp)
target_link_libraries(CameraUpdater ${catkin_LIBRARIES} rs_beliefstatecomm)

add_executable(PR2BaseUpdater src/PR2BaseUpdater.cpp)
target_link_libraries(PR2BaseUpdater ${catkin_LIBRARIES})

add_library(rs_beliefstatecomm SHARED
  src/BeliefStateCommunication.cpp
)
target_link_libraries(rs_beliefstatecomm ${catkin_LIBRARIES})

rs_add_library(rs_UE4Spawner src/annotators/UE4Spawner.cpp)
target_link_libraries(rs_UE4Spawner ${catkin_LIBRARIES} rs_beliefstatecomm)
rs_add_library(rs_GetBeliefStateData src/GetBeliefStateData.cpp)
target_link_libraries(rs_GetBeliefStateData ${catkin_LIBRARIES})
rs_add_library(rs_ClusterColorFromBeliefState src/ClusterColorFromBeliefState.cpp)
target_link_libraries(rs_ClusterColorFromBeliefState ${catkin_LIBRARIES})
