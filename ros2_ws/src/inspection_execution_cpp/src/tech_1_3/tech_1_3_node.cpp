// tech_1_3_node：单可执行文件装配 PPO 在线运行时四个组件
//   - PpoPolicyLoaderImpl   （实现 tech_1_1::PpoPolicyRuntime）
//   - ObservationBuilder    （imu + foot_force + terrain → observation）
//   - PolicyOutputValidator（action 维度/范围/时效）
//   - LocomotionCommandAdapter（action → RobotSdkAdapter 的 JointCommand）

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "inspection_execution_cpp/common/node_names.hpp"
#include "inspection_execution_cpp/common/topic_names.hpp"
#include "inspection_execution_cpp/adapters/imu_adapter.hpp"
#include "inspection_execution_cpp/adapters/foot_force_adapter.hpp"
#include "inspection_execution_cpp/adapters/robot_sdk_adapter.hpp"
#include "inspection_execution_cpp/tech_1_3/ppo_policy_loader.hpp"
#include "inspection_execution_cpp/tech_1_3/observation_builder.hpp"
#include "inspection_execution_cpp/tech_1_3/policy_output_validator.hpp"
#include "inspection_execution_cpp/tech_1_3/locomotion_command_adapter.hpp"

// 复用 inspection_interfaces/TerrainObservation
#include "inspection_interfaces/msg/terrain_observation.hpp"

using namespace std::chrono_literals;

namespace inspection_execution {
namespace tech_1_3 {

class Tech1_3Node : public rclcpp::Node {
 public:
  Tech1_3Node()
      : Node(node_names::kTech1_3Node),
        loader_(std::make_unique<PpoPolicyLoaderImpl>()),
        builder_(std::make_unique<ObservationBuilder>()),
        validator_(std::make_unique<PolicyOutputValidator>()) {
    // 声明参数：策略加载目录（来自 Python policy_exporter 导出目录）
    this->declare_parameter<std::string>("policy_dir", "");

    terrain_obs_pub_ = this->create_publisher<inspection_interfaces::msg::TerrainObservation>(
        topic_names::kTerrainObservation, 10);

    // 运行时线程：IMU 1000Hz / 足力 500Hz 由 adapter 线程推动；
    // 本节点 100Hz 轮询组装观测并推策略。
    timer_ = this->create_wall_timer(10ms, std::bind(&Tech1_3Node::Tick, this));

    RCLCPP_INFO(this->get_logger(), "%s started", node_names::kTech1_3Node);
  }

  // 依赖注入（便于 unittest / launcher 传入 fake adapter）
  void SetRobotSdk(std::shared_ptr<RobotSdkAdapter> robot) {
    robot_ = std::move(robot);
    adapter_ = std::make_unique<LocomotionCommandAdapter>(robot_);
  }
  void SetImu(std::shared_ptr<ImuAdapter> imu) { imu_ = std::move(imu); }
  void SetFootForce(std::shared_ptr<FootForceAdapter> ff) { foot_force_ = std::move(ff); }

 private:
  void Tick() {
    const auto now = this->now();
    // 1. 加载策略（一次性）
    if (!policy_configured_) {
      const auto dir = this->get_parameter("policy_dir").as_string();
      if (!dir.empty()) {
        if (loader_->Load(dir)) {
          const auto& meta = loader_->Metadata();
          builder_->SetExpectedDimension(meta.observation_dim);
          validator_->SetExpectedDimension(meta.action_dim);
          RCLCPP_INFO(this->get_logger(), "policy loaded: v=%s obs=%u act=%u loaded=%d",
                      meta.version.c_str(), meta.observation_dim, meta.action_dim,
                      loader_->IsLoaded() ? 1 : 0);
        } else {
          RCLCPP_WARN(this->get_logger(), "policy load failed from %s", dir.c_str());
        }
        policy_configured_ = true;
      }
    }

    // 2. 组装观测（无 IMU/足力 时跳过，不阻塞）
    if (imu_ && foot_force_) {
      ImuSample imu;
      FootForceSample ff;
      if (imu_->ReadSample(&imu) && foot_force_->ReadSample(&ff)) {
        ObservationInput in;
        in.imu = imu;
        in.foot_force = ff;
        in.timestamp_ns = static_cast<std::uint64_t>(now.nanoseconds());
        std::vector<float> obs;
        if (builder_->Build(in, &obs)) {
          PublishTerrainObservation(in, now);
          TryRunPolicy(obs, now);
        }
      }
    }
  }

  void PublishTerrainObservation(const ObservationInput& in, const rclcpp::Time& now) {
    inspection_interfaces::msg::TerrainObservation m;
    m.header.stamp = now;
    m.header.frame_id = "base_link";
    m.terrain_type = in.terrain_type;
    m.foot_contact.resize(kFootCount);
    for (std::size_t i = 0; i < kFootCount; ++i) {
      m.foot_contact[i] = in.foot_force.contact_flags[i] > 0.0;
    }
    m.confidence = 1.0;
    terrain_obs_pub_->publish(m);
  }

  void TryRunPolicy(const std::vector<float>& obs, const rclcpp::Time& now) {
    if (!loader_->IsLoaded() || !adapter_) return;
    std::vector<float> action;
    if (!loader_->Infer(obs, &action)) return;  // 不虚构 action
    const auto ns_now = static_cast<std::uint64_t>(now.nanoseconds());
    auto ec = validator_->Clamp(&action);
    if (ec != ErrorCode::kOk) return;
    const auto valid = validator_->Validate(action, ns_now, ns_now);
    if (!valid.success) return;
    (void)adapter_->ConvertAndSend(action);
  }

  std::unique_ptr<PpoPolicyLoaderImpl> loader_;
  std::unique_ptr<ObservationBuilder> builder_;
  std::unique_ptr<PolicyOutputValidator> validator_;
  std::unique_ptr<LocomotionCommandAdapter> adapter_;
  std::shared_ptr<RobotSdkAdapter> robot_;
  std::shared_ptr<ImuAdapter> imu_;
  std::shared_ptr<FootForceAdapter> foot_force_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<inspection_interfaces::msg::TerrainObservation>::SharedPtr terrain_obs_pub_;
  bool policy_configured_{false};
};

}  // namespace tech_1_3
}  // namespace inspection_execution

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<inspection_execution::tech_1_3::Tech1_3Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
