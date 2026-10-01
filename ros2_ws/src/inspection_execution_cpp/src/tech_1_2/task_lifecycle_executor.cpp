#include "inspection_execution_cpp/tech_1_2/task_lifecycle_executor.hpp"

#include <chrono>
#include <functional>
#include <thread>

#include "inspection_execution_cpp/common/node_names.hpp"
#include "inspection_execution_cpp/common/topic_names.hpp"

namespace inspection_execution {
namespace tech_1_2 {

TaskLifecycleExecutor::TaskLifecycleExecutor(const rclcpp::NodeOptions& options)
    : Node(node_names::kTaskLifecycleExecutorNode, options) {
  using namespace std::placeholders;

  // 实机就绪信号与真实任务进度尚未接入：用参数显式配置，默认值不等于真实结论
  declare_parameter("assume_robot_ready", true);
  declare_parameter("task_duration_sec", 2.0);
  assume_robot_ready_ = get_parameter("assume_robot_ready").as_bool();
  const double duration = get_parameter("task_duration_sec").as_double();
  // 反馈周期固定 10 Hz（与 Execute() 里的 rclcpp::Rate 一致）
  progress_increment_ = duration > 0.0 ? (0.1 / duration) : 0.05;

  action_server_ = rclcpp_action::create_server<ExecuteTask>(
      this, topic_names::kExecuteTaskAction,
      std::bind(&TaskLifecycleExecutor::HandleGoal, this, _1, _2),
      std::bind(&TaskLifecycleExecutor::HandleCancel, this, _1),
      std::bind(&TaskLifecycleExecutor::HandleAccepted, this, _1));

  resume_service_ = create_service<ResumeTask>(
      topic_names::kResumeTaskService,
      std::bind(&TaskLifecycleExecutor::HandleResume, this, _1, _2));

  RCLCPP_INFO(get_logger(), "task lifecycle executor ready (execute_task / resume_task)");
}

rclcpp_action::GoalResponse TaskLifecycleExecutor::HandleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteTask::Goal> goal) {
  latency_monitor_.MarkArrival(std::chrono::steady_clock::now());

  tech_1_1::GoalSafetyInput input;
  input.goal_id = goal->goal.goal_id;
  input.task_type = goal->goal.task_type;
  input.valid_until_sec = static_cast<double>(goal->goal.valid_until.sec) +
                          static_cast<double>(goal->goal.valid_until.nanosec) * 1e-9;
  input.now_sec = static_cast<double>(now().seconds()) +
                  static_cast<double>(now().nanoseconds()) * 1e-9;
  input.robot_ready = assume_robot_ready_;
  if (assume_robot_ready_ && !readiness_warning_logged_) {
    readiness_warning_logged_ = true;
    RCLCPP_WARN(get_logger(),
                "robot readiness signal is not wired yet; goal validation uses "
                "assume_robot_ready=true (placeholder, set it false to require a "
                "real readiness signal)");
  }
  input.constraints = goal->goal.constraints;

  const tech_1_1::GoalSafetyResult result = validator_.Validate(input);
  if (!result.allowed) {
    RCLCPP_WARN(get_logger(), "rejecting goal %s: %s",
                goal->goal.goal_id.c_str(), result.reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskLifecycleExecutor::HandleCancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/) {
  RCLCPP_INFO(get_logger(), "cancel request accepted");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskLifecycleExecutor::HandleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
  latency_monitor_.MarkSwitchComplete(std::chrono::steady_clock::now());
  std::thread{std::bind(&TaskLifecycleExecutor::Execute, this, goal_handle)}.detach();
}

void TaskLifecycleExecutor::Execute(const std::shared_ptr<GoalHandle> goal_handle) {
  const auto goal_msg = goal_handle->get_goal()->goal;
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  auto result = std::make_shared<ExecuteTask::Result>();
  double progress = 0.0;

  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      progress_store_.Save(goal_msg.goal_id, TaskProgress{progress, "preempted"});
      result->success = false;
      result->message = "preempted";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "goal %s preempted at %.2f",
                  goal_msg.goal_id.c_str(), progress);
      return;
    }

    // 按 task_duration_sec 均匀推进，代表"执行中"而非真实完成度
    progress += progress_increment_;
    feedback->progress = progress;
    feedback->current_state = "running";
    goal_handle->publish_feedback(feedback);

    if (progress >= 1.0) {
      progress_store_.Clear(goal_msg.goal_id);
      result->success = true;
      result->message = "completed";
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "goal %s completed", goal_msg.goal_id.c_str());
      return;
    }
    rate.sleep();
  }
}

void TaskLifecycleExecutor::HandleResume(
    const std::shared_ptr<ResumeTask::Request> request,
    std::shared_ptr<ResumeTask::Response> response) {
  TaskProgress progress;
  if (!resume_executor_.LoadResumePoint(request->task_id, &progress)) {
    response->accepted = false;
    response->resume_state = "no saved progress";
    return;
  }
  response->accepted = true;
  response->resume_state = progress.state;
  RCLCPP_INFO(get_logger(), "resume request for %s at %.2f",
              request->task_id.c_str(), progress.progress);
}

}  // namespace tech_1_2
}  // namespace inspection_execution
