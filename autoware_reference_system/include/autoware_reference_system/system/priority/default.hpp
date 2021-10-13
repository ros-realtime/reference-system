#ifndef AUTOWARE_REFERENCE_SYSTEM__SYSTEM__PRIORITY__DEFAULT_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__SYSTEM__PRIORITY__DEFAULT_HPP_

namespace callback
{
namespace priority
{
  struct Default
  {
    // The higher number, more critical callback
    static constexpr int FRONT_LIDAR_DRIVER_CALLBACK = 51;
    static constexpr int REAR_LIDAR_DRIVER_CALLBACK = 50;
    static constexpr int POINT_CLOUD_MAP_CALLBACK = 22;
    static constexpr int LANELET_2_MAP_CALLBACK = 30;
    static constexpr int VISUALIZER_CALLBACK = 27;
    static constexpr int POINTS_TRANSFORMER_REAR_CALLBACK = 52;
    static constexpr int POINTS_TRANSFORMER_FRONT_CALLBACK = 53;
    static constexpr int POINT_CLOUD_FUSION_CALLBACK_1 = 55;
    static constexpr int POINT_CLOUD_FUSION_CALLBACK_2 = 54;
    static constexpr int POINT_CLOUD_MAP_LOADER_CALLBACK = 24;
    static constexpr int VOXEL_GRID_DOWNSAMPLER_CALLBACK = 23;
    static constexpr int RAY_GROUND_FILTER_CALLBACK = 56;
    static constexpr int NDT_LOCALIZER_CALLBACK_1 = 26;
    static constexpr int NDT_LOCALIZER_CALLBACK_2 = 25;
    static constexpr int EUCLIDEAN_CLUSTER_SETTINGS_CALLBACK = 47;
    static constexpr int INTERSECTION_OUTPUT_CALLBACK = 49;
    static constexpr int EUCLIDEAN_CLUSTER_DETECTOR_CALLBACK = 57;
    static constexpr int EUCLIDEAN_INTERSECTION_CALLBACK = 48;
    static constexpr int LANELET_2_GLOBAL_PLANNER_CALLBACK_1 = 29;
    static constexpr int LANELET_2_GLOBAL_PLANNER_CALLBACK_2 = 28;
    static constexpr int LANELET_2_MAP_LOADER_CALLBACK_1 = 31;
    static constexpr int LANELET_2_MAP_LOADER_CALLBACK_2 = 32;
    static constexpr int PARKING_PLANNER_CALLBACK = 33;
    static constexpr int LANE_PLANNER_CALLBACK = 34;
    static constexpr int OBJECT_COLLISION_ESTIMATOR_CALLBACK = 58;
    static constexpr int BEHAVIOR_PLANNER_CALLBACK_1 = 59;
    static constexpr int BEHAVIOR_PLANNER_CALLBACK_2 = 39;
    static constexpr int BEHAVIOR_PLANNER_CALLBACK_3 = 35;
    static constexpr int BEHAVIOR_PLANNER_CALLBACK_4 = 36;
    static constexpr int BEHAVIOR_PLANNER_CALLBACK_5 = 38;
    static constexpr int BEHAVIOR_PLANNER_CALLBACK_6 = 37;
    static constexpr int BEHAVIOR_PLANNER_CALLBACK_7 = 65;
    static constexpr int MPC_CONTROLLER_CALLBACK = 68;
    static constexpr int VEHICLE_INTERFACE_CALLBACK_1 = 67;
    static constexpr int VEHICLE_INTERFACE_CALLBACK_2 = 66;
    static constexpr int VEHICLE_DBW_SYSTEM_CALLBACK = 69;
  };
  constexpr int Default::FRONT_LIDAR_DRIVER_CALLBACK;
  constexpr int Default::REAR_LIDAR_DRIVER_CALLBACK;
  constexpr int Default::POINT_CLOUD_MAP_CALLBACK;
  constexpr int Default::LANELET_2_MAP_CALLBACK;
  constexpr int Default::VISUALIZER_CALLBACK;
  constexpr int Default::POINTS_TRANSFORMER_REAR_CALLBACK;
  constexpr int Default::POINTS_TRANSFORMER_FRONT_CALLBACK;
  constexpr int Default::POINT_CLOUD_FUSION_CALLBACK_1;
  constexpr int Default::POINT_CLOUD_FUSION_CALLBACK_2;
  constexpr int Default::POINT_CLOUD_MAP_LOADER_CALLBACK;
  constexpr int Default::VOXEL_GRID_DOWNSAMPLER_CALLBACK;
  constexpr int Default::RAY_GROUND_FILTER_CALLBACK;
  constexpr int Default::NDT_LOCALIZER_CALLBACK_1;
  constexpr int Default::NDT_LOCALIZER_CALLBACK_2;
  constexpr int Default::EUCLIDEAN_CLUSTER_DETECTOR_CALLBACK;
  constexpr int Default::INTERSECTION_OUTPUT_CALLBACK;
  constexpr int Default::EUCLIDEAN_INTERSECTION_CALLBACK;
  constexpr int Default::EUCLIDEAN_CLUSTER_SETTINGS_CALLBACK;
  constexpr int Default::LANELET_2_GLOBAL_PLANNER_CALLBACK_1;
  constexpr int Default::LANELET_2_GLOBAL_PLANNER_CALLBACK_2;
  constexpr int Default::LANELET_2_MAP_LOADER_CALLBACK_1;
  constexpr int Default::LANELET_2_MAP_LOADER_CALLBACK_2;
  constexpr int Default::PARKING_PLANNER_CALLBACK;
  constexpr int Default::LANE_PLANNER_CALLBACK;
  constexpr int Default::OBJECT_COLLISION_ESTIMATOR_CALLBACK;
  constexpr int Default::BEHAVIOR_PLANNER_CALLBACK_1;
  constexpr int Default::BEHAVIOR_PLANNER_CALLBACK_2;
  constexpr int Default::BEHAVIOR_PLANNER_CALLBACK_3;
  constexpr int Default::BEHAVIOR_PLANNER_CALLBACK_4;
  constexpr int Default::BEHAVIOR_PLANNER_CALLBACK_5;
  constexpr int Default::BEHAVIOR_PLANNER_CALLBACK_6;
  constexpr int Default::BEHAVIOR_PLANNER_CALLBACK_7;
  constexpr int Default::MPC_CONTROLLER_CALLBACK;
  constexpr int Default::VEHICLE_INTERFACE_CALLBACK_1;
  constexpr int Default::VEHICLE_INTERFACE_CALLBACK_2;
  constexpr int Default::VEHICLE_DBW_SYSTEM_CALLBACK;
  
}   // namespace priority
}   // namespace callback
#endif  // AUTOWARE_REFERENCE_SYSTEM__SYSTEM__PRIORITY__DEFAULT_HPP_