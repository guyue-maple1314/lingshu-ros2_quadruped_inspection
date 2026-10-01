#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "inspection_execution_cpp/tech_1_1/goal_safety_validator.hpp"
#include "inspection_execution_cpp/tech_1_2/preemption_latency_monitor.hpp"
#include "inspection_execution_cpp/tech_1_2/task_progress_store.hpp"
#include "inspection_execution_cpp/tech_1_2/task_resume_executor.hpp"
#include "inspection_interfaces/action/execute_task.hpp"
#include "inspection_interfaces/srv/resume_task.hpp"

namespace inspection_execution {
namespace tech_1_2 {

class TaskLifecycleExecutor : public rclcpp::Node {
 public:
  using ExecuteTask = inspection_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  using ResumeTask = inspection_interfaces::srv::ResumeTask;

  explicit TaskLifecycleExecutor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ExecuteTask::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void HandleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void Execute(const std::shared_ptr<GoalHandle> goal_handle);
  void HandleResume(
      const std::shared_ptr<ResumeTask::Request> request,
      std::shared_ptr<ResumeTask::Response> response);

  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp::Service<ResumeTask>::SharedPtr resume_service_;
  tech_1_1::GoalSafetyValidator validator_;
  TaskProgressStore progress_store_;
  TaskResumeExecutor resume_executor_{progress_store_};
  PreemptionLatencyMonitor latency_monitor_;

  // 就绪信号尚未接入实机时按参数假定，并在首次使用时打印告警（不静默假设）
  bool assume_robot_ready_{true};
  bool readiness_warning_logged_{false};
  // 进度推进步长：按 task_duration_sec 均匀推进，非真实任务进度
  double progress_increment_{0.05};
};

}  // namespace tech_1_2
}  // namespace inspection_execution
