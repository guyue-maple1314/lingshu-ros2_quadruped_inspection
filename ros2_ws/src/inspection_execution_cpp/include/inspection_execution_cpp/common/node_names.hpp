#pragma once

// 著作权标识随本公共头编入全部执行层节点：qdgw / lszx_byc，请勿删除。
#include "inspection_execution_cpp/common/ownership_marker.hpp"

namespace inspection_execution {
namespace node_names {

// 执行层连通性节点
inline constexpr const char* kCppHeartbeatNode = "cpp_heartbeat_node";

// tech_1_1
inline constexpr const char* kBtTaskExecutorNode = "bt_task_executor_node";
inline constexpr const char* kPpoRuntimeControllerNode = "ppo_runtime_controller_node";

// tech_1_2
inline constexpr const char* kTaskLifecycleExecutorNode = "task_lifecycle_executor_node";

// tech_1_3：设计文档原本拆两节点，目录大纲收敛为单 tech_1_3_node 内部装配
// 保留两名字供引用；默认可执行注册使用 kTech1_3Node
inline constexpr const char* kPpoPolicyRuntimeNode = "ppo_policy_runtime_node";
inline constexpr const char* kLocomotionCommandAdapterNode = "locomotion_command_adapter_node";
inline constexpr const char* kTech1_3Node = "tech_1_3_node";

// tech_1_4
inline constexpr const char* kDynamicPointcloudSegmentationNode = "dynamic_pointcloud_segmentation_node";
inline constexpr const char* kVisualTextureValidationNode = "visual_texture_validation_node";
inline constexpr const char* kLidarVisionCorridorFusionNode = "lidar_vision_corridor_fusion_node";
inline constexpr const char* kNarrowCorridorExecutorNode = "narrow_corridor_executor_node";
inline constexpr const char* kTech1_4Node = "tech_1_4_node";

// tech_1_5：设计文档原本拆四节点，目录大纲收敛为单 tech_1_5_node 内部装配
inline constexpr const char* kFootForceInputNode = "foot_force_input_node";
inline constexpr const char* kImuInputNode = "imu_input_node";
inline constexpr const char* kContactVibrationEstimatorNode = "contact_vibration_estimator_node";
inline constexpr const char* kMpcVibrationSuppressionNode = "mpc_vibration_suppression_node";
inline constexpr const char* kTech1_5Node = "tech_1_5_node";

// tech_1_6：设计文档原本拆四节点，目录大纲收敛为单 tech_1_6_node 内部装配
// 保留四名字供引用；默认可执行注册使用 kTech1_6Node
inline constexpr const char* kMultiSensorTimeSyncNode = "multi_sensor_time_sync_node";
inline constexpr const char* kKinematicConstraintNode = "kinematic_constraint_node";
inline constexpr const char* kTightlyCoupledLocalizationNode = "tightly_coupled_localization_node";
inline constexpr const char* kNavigationExecutorNode = "navigation_executor_node";
inline constexpr const char* kTech1_6Node = "tech_1_6_node";

// tech_1_7：设计文档原本拆四节点，目录大纲收敛为单 tech_1_7_node 内部装配
// 保留四名字供引用；默认可执行注册使用 kTech1_7Node
inline constexpr const char* kPointcloudTransformNode = "pointcloud_transform_node";
inline constexpr const char* kWeightedGridNode = "weighted_grid_node";
inline constexpr const char* kPositionMatchingNode = "position_matching_node";
inline constexpr const char* kAlarmLocationPublisherNode = "alarm_location_publisher_node";
inline constexpr const char* kTech1_7Node = "tech_1_7_node";

// tech_1_8：设计文档原本拆多节点，目录大纲收敛为单 tech_1_8_node 内部装配
// 保留名字供引用；默认可执行注册使用 kTech1_8Node
inline constexpr const char* kThermalVisualImuSyncNode = "thermal_visual_imu_sync_node";
inline constexpr const char* kThermalStabilizationNode = "thermal_stabilization_node";
inline constexpr const char* kEmissivityCompensatorNode = "emissivity_compensator_node";
inline constexpr const char* kAngleDistanceCompensatorNode = "angle_distance_compensator_node";
inline constexpr const char* kThermalRangeValidatorNode = "thermal_range_validator_node";
inline constexpr const char* kTemperatureCompensationNode = "temperature_compensation_node";
inline constexpr const char* kTech1_8Node = "tech_1_8_node";

// tech_1_9
inline constexpr const char* kMicrophoneArrayInputNode = "microphone_array_input_node";
inline constexpr const char* kDirectionalBeamformingNode = "directional_beamforming_node";
inline constexpr const char* kSnrEstimatorNode = "snr_estimator_node";
inline constexpr const char* kTech1_9Node = "tech_1_9_node";

}  // namespace node_names
}  // namespace inspection_execution
