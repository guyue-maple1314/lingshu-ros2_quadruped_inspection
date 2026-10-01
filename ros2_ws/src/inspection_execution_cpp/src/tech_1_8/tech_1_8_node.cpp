#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "inspection_execution_cpp/adapters/imu_adapter.hpp"
#include "inspection_execution_cpp/common/node_names.hpp"
#include "inspection_execution_cpp/common/topic_names.hpp"
#include "inspection_execution_cpp/tech_1_8/thermal_visual_imu_synchronizer.hpp"
#include "inspection_execution_cpp/tech_1_8/thermal_image_stabilizer.hpp"
#include "inspection_execution_cpp/tech_1_8/emissivity_compensator.hpp"
#include "inspection_execution_cpp/tech_1_8/angle_distance_compensator.hpp"
#include "inspection_execution_cpp/tech_1_8/thermal_range_validator.hpp"
#include "inspection_interfaces/msg/thermal_measurement.hpp"

namespace inspection_execution {
namespace tech_1_8 {

/// 动态红外测温节点（技术 1.8）
///
/// 装配五组件：
///   ThermalVisualImuSynchronizer → ThermalImageStabilizer
///   → EmissivityCompensator → AngleDistanceCompensator
///   → ThermalRangeValidator
///
/// 数据流：
///   ImuAdapter（注入）→ 1000Hz IMU 样本
///   红外帧由 SetThermalFrame 注入
///   发布 /thermal_measurement（ThermalMeasurement）
///
/// 红线：
///   - IMU 无效（适配器未注入/ReadSample 失败）时不发布（不虚构）
///   - 角度/距离/修正系数超 PPT 范围（±60°、1–5m、0.95–1.05）时
///     error_state="xxx_out_of_range"，不按范围内精度发布
///   - 稳像与温度补偿在 C++ 执行，Python 只做异常高温判定
class Tech18Node : public rclcpp::Node {
 public:
  Tech18Node() : rclcpp::Node(node_names::kTech1_8Node) {
    this->declare_parameter("control_period_ms", 33);  // ≈30fps
    this->declare_parameter("sync_tolerance_ms", 5.0);
    this->declare_parameter("exposure_ms", 8.0);
    this->declare_parameter("focal_factor", 400.0);

    const double tol = this->get_parameter("sync_tolerance_ms").as_double();
    const double exp_ms = this->get_parameter("exposure_ms").as_double();
    const double focal = this->get_parameter("focal_factor").as_double();

    synchronizer_ = std::make_unique<ThermalVisualImuSynchronizer>(tol);
    stabilizer_ = std::make_unique<ThermalImageStabilizer>(exp_ms, focal);

    // 发布 /thermal_measurement
    thermal_pub_ = this->create_publisher<inspection_interfaces::msg::ThermalMeasurement>(
        topic_names::kThermalMeasurement, 10);

    const int period_ms = this->get_parameter("control_period_ms").as_int();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        [this]() { Tick(); });

    RCLCPP_INFO(this->get_logger(),
                "%s started: 5-component dynamic thermal measurement",
                node_names::kTech1_8Node);
  }

  // ---- 适配器 / 数据注入接口 ----
  void SetImuAdapter(std::shared_ptr<ImuAdapter> adapter) {
    imu_adapter_ = std::move(adapter);
  }

  /// 红外帧注入
  void SetThermalFrame(const ThermalFrame& frame) {
    pending_thermal_ = frame;
  }

 private:
  void Tick() {
    // 1. 读取 IMU（通过抽象适配器，不直调 SDK）
    ImuSample imu_sample{};
    const bool imu_ok = imu_adapter_ && imu_adapter_->ReadSample(&imu_sample);
    if (imu_ok) {
      synchronizer_->UpdateImu(imu_sample);
    }

    // 2. 红外帧注入（未注入时 valid=false，不虚构）
    synchronizer_->UpdateThermal(pending_thermal_);
    pending_thermal_ = ThermalFrame{};  // 用后清空

    // 3. 时间同步（IMU 无效时返回 nullopt，不虚构）
    const auto packet = synchronizer_->TrySync();
    if (!packet) return;

    // 4. 稳像（IMU/红外无效时 valid=false）
    const auto stabilized = stabilizer_->Stabilize(*packet);
    if (!stabilized.valid) {
      PublishInvalid(stabilized.error_state, packet->thermal);
      return;
    }

    // 5. 辐射率补偿
    const auto emissivity_out = emissivity_compensator_.Compensate(stabilized);
    if (!emissivity_out.valid) {
      PublishInvalid(emissivity_out.error_state, packet->thermal);
      return;
    }

    // 6. 角度 / 距离补偿
    const auto angle_out = angle_distance_compensator_.Compensate(emissivity_out);
    if (!angle_out.valid) {
      PublishInvalid(angle_out.error_state, packet->thermal);
      return;
    }

    // 7. 范围校验（红线：超范围标 out_of_range，不按范围内精度发布）
    const auto validated = range_validator_.Validate(
        angle_out, stabilized.emissivity);

    // 8. 发布 ThermalMeasurement
    inspection_interfaces::msg::ThermalMeasurement msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "thermal_link";  // 对齐 frames.yaml: thermal: thermal_link
    msg.raw_temperature = packet->thermal.raw_temperature;
    msg.compensated_temperature = validated.final_temperature;
    msg.angle = validated.angle;
    msg.distance = validated.distance;
    msg.emissivity = validated.emissivity;
    msg.correction_factor = validated.correction_factor;
    // 范围内 → "ok"；超范围 → "xxx_out_of_range"（不按范围内精度发布）
    msg.error_state = validated.in_range ? "ok" : validated.error_state;
    thermal_pub_->publish(msg);

    if (!validated.in_range) {
      RCLCPP_WARN(this->get_logger(),
                  "thermal out of range: %s (angle=%.2f° dist=%.2fm cf=%.3f)",
                  validated.error_state.c_str(), validated.angle,
                  validated.distance, validated.correction_factor);
    }
  }

  void PublishInvalid(const std::string& error_state,
                      const ThermalFrame& raw) {
    inspection_interfaces::msg::ThermalMeasurement msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "thermal_link";  // 对齐 frames.yaml: thermal: thermal_link
    msg.raw_temperature = raw.raw_temperature;
    msg.compensated_temperature = 0.0;  // 无效补偿不虚构
    msg.angle = raw.angle;
    msg.distance = raw.distance;
    msg.emissivity = raw.emissivity;
    msg.correction_factor = raw.correction_factor;
    msg.error_state = error_state.empty() ? "invalid" : error_state;
    thermal_pub_->publish(msg);
  }

  std::shared_ptr<ImuAdapter> imu_adapter_{};
  std::unique_ptr<ThermalVisualImuSynchronizer> synchronizer_{};
  std::unique_ptr<ThermalImageStabilizer> stabilizer_{};
  EmissivityCompensator emissivity_compensator_{};
  AngleDistanceCompensator angle_distance_compensator_{};
  ThermalRangeValidator range_validator_{};

  ThermalFrame pending_thermal_{};

  rclcpp::Publisher<inspection_interfaces::msg::ThermalMeasurement>::SharedPtr
      thermal_pub_{};
  rclcpp::TimerBase::SharedPtr timer_{};
};

}  // namespace tech_1_8
}  // namespace inspection_execution

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<inspection_execution::tech_1_8::Tech18Node>());
  rclcpp::shutdown();
  return 0;
}
