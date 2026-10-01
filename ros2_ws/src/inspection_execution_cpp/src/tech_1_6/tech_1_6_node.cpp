#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "inspection_execution_cpp/common/node_names.hpp"
#include "inspection_execution_cpp/common/topic_names.hpp"
#include "inspection_execution_cpp/adapters/imu_adapter.hpp"
#include "inspection_execution_cpp/adapters/foot_force_adapter.hpp"
#include "inspection_execution_cpp/adapters/robot_sdk_adapter.hpp"
#include "inspection_execution_cpp/tech_1_6/multi_sensor_synchronizer.hpp"
#include "inspection_execution_cpp/tech_1_6/kinematic_constraint_builder.hpp"
#include "inspection_execution_cpp/tech_1_6/tightly_coupled_localizer.hpp"
#include "inspection_execution_cpp/tech_1_6/localization_degradation_monitor.hpp"
#include "inspection_execution_cpp/tech_1_6/navigation_executor.hpp"
#include "inspection_interfaces/msg/fusion_pose.hpp"
#include "inspection_interfaces/action/navigate_goal.hpp"

namespace inspection_execution {
namespace tech_1_6 {

/// 弱纹理环境高精度融合导航节点（技术 1.6）
///
/// 装配五组件：
///   MultiSensorSynchronizer → KinematicConstraintBuilder
///   → TightlyCoupledLocalizer → LocalizationDegradationMonitor
///   → NavigationExecutor
///
/// 适配器注入式设计（与 1.3/1.5 同理，不直调厂商 SDK）：
///   ImuAdapter / FootForceAdapter / RobotSdkAdapter 由上层注入
///   AbstractLocalizerBackend 由上层注入（融合/SLAM 库未批准时不虚构）
///
/// 激光/相机数据源尚未接入，同步器按无效标记处理，
/// 与 tech_1_4_node 示例同理，不虚构感知结果。
class Tech16Node : public rclcpp::Node {
 public:
  using NavigateGoal = inspection_interfaces::action::NavigateGoal;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateGoal>;

  Tech16Node() : rclcpp::Node(node_names::kTech1_6Node) {
    this->declare_parameter("sync_tolerance_ms", 5.0);
    this->declare_parameter("control_period_ms", 100);
    this->declare_parameter("nav_timeout_sec", 60.0);

    // 注入默认 FakeLocalizerBackend（不虚构 SLAM，确定性输出）
    auto backend = std::make_shared<FakeLocalizerBackend>();
    backend->Initialize();
    localizer_ = std::make_unique<TightlyCoupledLocalizer>(backend);
    RCLCPP_WARN(this->get_logger(),
                "default localizer backend is a placeholder (FakeLocalizerBackend): "
                "/fusion_pose carries deterministic zero-motion poses and a coarse "
                "covariance, NOT a measured localization result. Inject a real backend "
                "via SetLocalizerBackend before citing accuracy figures.");

    // 预设跨楼层示例路径（真实路径由上层规划器下发）
    std::vector<NavWaypoint> demo_path;
    {
      NavWaypoint wp;
      wp.x = 1.0; wp.floor_id = 1; demo_path.push_back(wp);
      wp.x = 2.0; wp.floor_id = 1; wp.is_stairs = true; demo_path.push_back(wp);
      wp.x = 3.0; wp.z = 3.0; wp.floor_id = 2; wp.is_stairs = false;
      demo_path.push_back(wp);
    }
    nav_executor_.LoadPath(demo_path);
    nav_executor_.Start();

    fusion_pub_ = this->create_publisher<inspection_interfaces::msg::FusionPose>(
        topic_names::kFusionPose, 10);

    // /navigate_goal：动态目标与重规划路径的导航（1.1/1.6）
    nav_action_server_ = rclcpp_action::create_server<NavigateGoal>(
        this, topic_names::kNavigateGoalAction,
        std::bind(&Tech16Node::HandleNavGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&Tech16Node::HandleNavCancel, this, std::placeholders::_1),
        std::bind(&Tech16Node::HandleNavAccepted, this, std::placeholders::_1));

    const int period_ms = this->get_parameter("control_period_ms").as_int();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        [this]() { Tick(); });

    RCLCPP_INFO(this->get_logger(),
                "%s started: 5-component fusion navigation (adapter injection)",
                node_names::kTech1_6Node);
  }

  // ---- 适配器注入接口 ----
  void SetImuAdapter(std::shared_ptr<ImuAdapter> adapter) {
    imu_adapter_ = std::move(adapter);
  }
  void SetFootForceAdapter(std::shared_ptr<FootForceAdapter> adapter) {
    foot_force_adapter_ = std::move(adapter);
  }
  void SetRobotSdkAdapter(std::shared_ptr<RobotSdkAdapter> adapter) {
    robot_sdk_adapter_ = std::move(adapter);
  }
  void SetLocalizerBackend(std::shared_ptr<AbstractLocalizerBackend> backend) {
    if (backend && backend->Initialize()) {
      localizer_ = std::make_unique<TightlyCoupledLocalizer>(backend);
    }
  }

 private:
  void Tick() {
    // 1. 读取传感器（通过抽象适配器，不直调 SDK）
    ImuSample imu_sample{};
    FootForceSample foot_sample{};
    if (imu_adapter_) imu_adapter_->ReadSample(&imu_sample);
    if (foot_force_adapter_) foot_force_adapter_->ReadSample(&foot_sample);

    // 激光/相机未接入：按无效标记推入时间戳
    // 当前示例：标记为无效（不伪造有效状态，红线）
    const std::uint64_t now_ns = static_cast<std::uint64_t>(
        this->now().nanoseconds());
    synchronizer_.UpdateLidar(now_ns, false);       // 激光未接入
    synchronizer_.UpdateCamera(0, now_ns, false);   // 左相机未接入
    synchronizer_.UpdateCamera(1, now_ns, false);   // 右相机未接入
    synchronizer_.UpdateImu(imu_sample);
    synchronizer_.UpdateFootForce(foot_sample);

    // 2. 时间同步
    const auto packet = synchronizer_.TrySync();
    if (!packet) return;

    // 3. 运动学约束构建（50N 足力 + 关节限位）
    RobotStateRaw robot_state{};
    if (robot_sdk_adapter_) robot_sdk_adapter_->ReadState(&robot_state);
    auto constraint_robot = constraint_builder_.BuildFromRobotState(
        robot_state, packet->synced_timestamp_ns);
    auto constraint_foot = constraint_builder_.BuildFromFootForce(
        packet->foot_force);
    const auto constraints = KinematicConstraintBuilder::Merge(
        constraint_robot, constraint_foot);

    // 4. 紧耦合定位
    LocalizationSolveInput solve_input;
    solve_input.sensors = *packet;
    solve_input.constraints = constraints;
    solve_input.prev_pose = localizer_->LatestPose();
    solve_input.timestamp_ns = packet->synced_timestamp_ns;

    FusionPoseOutput pose_out{};
    localizer_->Localize(solve_input, &pose_out);

    // 5. 退化监控
    const auto degradation = degradation_monitor_.Evaluate(*packet);
    // 退化时通知导航执行器
    nav_executor_.OnDegradation(
        degradation.level == DegradationLevel::kDegraded ||
        degradation.level == DegradationLevel::kLost);

    // 6. 导航执行
    nav_executor_.Update(pose_out);

    // 7. 发布 FusionPose
    auto msg = inspection_interfaces::msg::FusionPose();
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = pose_out.x;
    msg.pose.pose.position.y = pose_out.y;
    msg.pose.pose.position.z = pose_out.z;
    msg.pose.pose.orientation.w = pose_out.qw;
    msg.pose.pose.orientation.x = pose_out.qx;
    msg.pose.pose.orientation.y = pose_out.qy;
    msg.pose.pose.orientation.z = pose_out.qz;
    // 协方差（6 维对角：x,y,z,roll,pitch,yaw）
    for (std::size_t i = 0; i < 6 && i < pose_out.covariance.size(); ++i) {
      msg.pose.covariance[i * 6 + i] = pose_out.covariance[i];
    }
    msg.valid_sources = pose_out.valid_sources;
    msg.localization_state = pose_out.localization_state;
    fusion_pub_->publish(msg);

    // 导航 Action 反馈与终态（与 Tick 同线程驱动，无需额外加锁）
    UpdateNavigationAction(pose_out);
  }

  // ---- /navigate_goal Action 服务端 ----
  rclcpp_action::GoalResponse HandleNavGoal(
      const rclcpp_action::GoalUUID& /*uuid*/,
      std::shared_ptr<const NavigateGoal::Goal> goal) {
    const auto& p = goal->target_pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      RCLCPP_WARN(get_logger(), "rejecting navigate_goal: non-finite target pose");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleNavCancel(
      const std::shared_ptr<GoalHandleNavigate> /*goal_handle*/) {
    RCLCPP_INFO(get_logger(), "navigate_goal cancel accepted");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleNavAccepted(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
    const auto& p = goal_handle->get_goal()->target_pose.pose.position;
    NavWaypoint wp;
    wp.x = p.x;
    wp.y = p.y;
    wp.z = p.z;
    wp.floor_id = 0;

    if (!nav_executor_.LoadPath({wp}) || !nav_executor_.Start()) {
      auto result = std::make_shared<NavigateGoal::Result>();
      result->success = false;
      result->message = "failed to load navigation target";
      goal_handle->abort(result);
      return;
    }

    nav_goal_handle_ = goal_handle;
    nav_goal_start_ = this->now();
    RCLCPP_INFO(get_logger(), "navigate_goal accepted: (%.2f, %.2f, %.2f)", p.x,
                p.y, p.z);
  }

  void UpdateNavigationAction(const FusionPoseOutput& pose) {
    if (!nav_goal_handle_) {
      return;
    }
    const auto goal_handle = nav_goal_handle_;

    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<NavigateGoal::Result>();
      result->success = false;
      result->message = "canceled";
      goal_handle->canceled(result);
      nav_goal_handle_.reset();
      return;
    }

    const auto snapshot = nav_executor_.Update(pose);

    auto feedback = std::make_shared<NavigateGoal::Feedback>();
    feedback->current_pose.pose.position.x = pose.x;
    feedback->current_pose.pose.position.y = pose.y;
    feedback->current_pose.pose.position.z = pose.z;
    feedback->current_pose.pose.orientation.w = pose.qw;
    feedback->current_pose.pose.orientation.x = pose.qx;
    feedback->current_pose.pose.orientation.y = pose.qy;
    feedback->current_pose.pose.orientation.z = pose.qz;
    feedback->state = NavigationStateName(snapshot.state);
    goal_handle->publish_feedback(feedback);

    if (snapshot.state == NavigationState::kCompleted) {
      auto result = std::make_shared<NavigateGoal::Result>();
      result->success = true;
      result->message = "target reached";
      goal_handle->succeed(result);
      nav_goal_handle_.reset();
      return;
    }
    if (snapshot.state == NavigationState::kFailed) {
      auto result = std::make_shared<NavigateGoal::Result>();
      result->success = false;
      result->message = "navigation failed";
      goal_handle->abort(result);
      nav_goal_handle_.reset();
      return;
    }

    const double elapsed = (this->now() - nav_goal_start_).seconds();
    if (elapsed > this->get_parameter("nav_timeout_sec").as_double()) {
      auto result = std::make_shared<NavigateGoal::Result>();
      result->success = false;
      result->message = "navigation timeout";
      goal_handle->abort(result);
      nav_goal_handle_.reset();
      RCLCPP_WARN(get_logger(), "navigate_goal aborted: timeout after %.1fs",
                  elapsed);
    }
  }

  static const char* NavigationStateName(NavigationState state) {
    switch (state) {
      case NavigationState::kIdle: return "idle";
      case NavigationState::kNavigating: return "navigating";
      case NavigationState::kOnStairs: return "on_stairs";
      case NavigationState::kFloorTransition: return "floor_transition";
      case NavigationState::kDegradedPause: return "degraded_pause";
      case NavigationState::kCompleted: return "completed";
      case NavigationState::kFailed: return "failed";
    }
    return "unknown";
  }

  MultiSensorSynchronizer synchronizer_{MultiSensorSynchronizer::kDefaultToleranceMs};
  KinematicConstraintBuilder constraint_builder_{};
  std::unique_ptr<TightlyCoupledLocalizer> localizer_{};
  LocalizationDegradationMonitor degradation_monitor_{};
  NavigationExecutor nav_executor_{};

  std::shared_ptr<ImuAdapter> imu_adapter_{};
  std::shared_ptr<FootForceAdapter> foot_force_adapter_{};
  std::shared_ptr<RobotSdkAdapter> robot_sdk_adapter_{};

  rclcpp::Publisher<inspection_interfaces::msg::FusionPose>::SharedPtr fusion_pub_{};
  rclcpp::TimerBase::SharedPtr timer_{};
  rclcpp_action::Server<NavigateGoal>::SharedPtr nav_action_server_{};
  std::shared_ptr<GoalHandleNavigate> nav_goal_handle_{};
  rclcpp::Time nav_goal_start_{0, 0, RCL_ROS_TIME};
};

}  // namespace tech_1_6
}  // namespace inspection_execution

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<inspection_execution::tech_1_6::Tech16Node>());
  rclcpp::shutdown();
  return 0;
}
