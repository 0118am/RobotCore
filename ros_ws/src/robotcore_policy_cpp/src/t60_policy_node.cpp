#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robotcore_interfaces/msg/body_state.hpp"
#include "robotcore_interfaces/msg/policy_status.hpp"
#include "robotcore_interfaces/msg/thruster_command.hpp"
#include "robotcore_interfaces/msg/trajectory_target.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace robotcore_policy_cpp
{
constexpr std::size_t kCurrentSize = 33;
constexpr std::size_t kHistoryFrames = 8;
constexpr std::size_t kHistoryFrameSize = 21;
constexpr std::size_t kObservationSize = kCurrentSize + kHistoryFrames * kHistoryFrameSize;
constexpr std::size_t kActionSize = 8;
static_assert(kObservationSize == 201);

constexpr std::array<float, kCurrentSize> kObservationScale{
  0.25F, 0.25F, 0.25F,
  0.40F, 0.40F, 0.40F,
  0.20F, 0.20F, 0.20F,
  1.00F, 1.00F, 1.00F, 1.00F,
  1.00F, 1.00F, 1.00F,
  0.80F, 0.80F, 0.80F,
  0.80F, 0.80F, 0.80F,
  0.45F, 0.45F, 0.45F,
  1.00F, 1.00F, 1.00F, 1.00F, 1.00F, 1.00F, 1.00F, 1.00F};

constexpr std::array<std::size_t, kHistoryFrameSize> kHistoryIndices{
  0, 1, 2, 6, 7, 8, 9, 10, 11, 12, 16, 17, 18,
  25, 26, 27, 28, 29, 30, 31, 32};

struct Vec3
{
  double x{};
  double y{};
  double z{};
};

struct Quaternion
{
  double w{1.0};
  double x{};
  double y{};
  double z{};
};

Vec3 operator-(const Vec3 & left, const Vec3 & right)
{
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vec3 cross(const Vec3 & left, const Vec3 & right)
{
  return {
    left.y * right.z - left.z * right.y,
    left.z * right.x - left.x * right.z,
    left.x * right.y - left.y * right.x};
}

Vec3 operator+(const Vec3 & left, const Vec3 & right)
{
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3 operator*(double scale, const Vec3 & value)
{
  return {scale * value.x, scale * value.y, scale * value.z};
}

Quaternion normalized(Quaternion value)
{
  const double norm = std::sqrt(
    value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    throw std::runtime_error("zero or non-finite quaternion");
  }
  value.w /= norm;
  value.x /= norm;
  value.y /= norm;
  value.z /= norm;
  return value;
}

Quaternion conjugate(const Quaternion & value)
{
  return {value.w, -value.x, -value.y, -value.z};
}

Quaternion multiply(const Quaternion & left, const Quaternion & right)
{
  return {
    left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
    left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
    left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w};
}

Vec3 quaternion_to_rpy(const Quaternion & value)
{
  return {
    std::atan2(
      2.0 * (value.w * value.x + value.y * value.z),
      1.0 - 2.0 * (value.x * value.x + value.y * value.y)),
    std::asin(std::clamp(
      2.0 * (value.w * value.y - value.z * value.x), -1.0, 1.0)),
    std::atan2(
      2.0 * (value.w * value.z + value.x * value.y),
      1.0 - 2.0 * (value.y * value.y + value.z * value.z))};
}

Quaternion quaternion_from_rpy(const Vec3 & value)
{
  const double cr = std::cos(0.5 * value.x);
  const double sr = std::sin(0.5 * value.x);
  const double cp = std::cos(0.5 * value.y);
  const double sp = std::sin(0.5 * value.y);
  const double cy = std::cos(0.5 * value.z);
  const double sy = std::sin(0.5 * value.z);
  return normalized({
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy});
}

Vec3 rotate(const Quaternion & quaternion, const Vec3 & vector)
{
  const Vec3 xyz{quaternion.x, quaternion.y, quaternion.z};
  const Vec3 twice_cross = 2.0 * cross(xyz, vector);
  return vector + quaternion.w * twice_cross + cross(xyz, twice_cross);
}

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "TensorRT: " << message << '\n';
    }
  }
};

template<typename T>
using TrtPtr = std::unique_ptr<T>;

void cuda_check(cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(
            std::string(operation) + ": " + cudaGetErrorString(result));
  }
}

class TensorRtActor
{
public:
  explicit TensorRtActor(const std::string & model_path)
  {
    try {
      TrtPtr<nvinfer1::IBuilder> builder{nvinfer1::createInferBuilder(logger_)};
      if (!builder) {throw std::runtime_error("TensorRT builder creation failed");}
      TrtPtr<nvinfer1::INetworkDefinition> network{
        builder->createNetworkV2(
          1U << static_cast<unsigned>(
            nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH))};
      if (!network) {throw std::runtime_error("TensorRT network creation failed");}
      TrtPtr<nvonnxparser::IParser> parser{
        nvonnxparser::createParser(*network, logger_)};
      if (!parser || !parser->parseFromFile(
          model_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
      {
        throw std::runtime_error("TensorRT failed to parse " + model_path);
      }
      TrtPtr<nvinfer1::IBuilderConfig> config{builder->createBuilderConfig()};
      if (!config) {throw std::runtime_error("TensorRT config creation failed");}
      TrtPtr<nvinfer1::IHostMemory> plan{
        builder->buildSerializedNetwork(*network, *config)};
      if (!plan) {throw std::runtime_error("TensorRT engine build failed");}

      runtime_.reset(nvinfer1::createInferRuntime(logger_));
      if (!runtime_) {throw std::runtime_error("TensorRT runtime creation failed");}
      engine_.reset(runtime_->deserializeCudaEngine(plan->data(), plan->size()));
      if (!engine_) {throw std::runtime_error("TensorRT engine deserialization failed");}
      context_.reset(engine_->createExecutionContext());
      if (!context_) {throw std::runtime_error("TensorRT context creation failed");}
      validate_contract();

      cuda_check(cudaMalloc(&device_input_, sizeof(float) * kObservationSize), "cudaMalloc input");
      cuda_check(cudaMalloc(&device_output_, sizeof(float) * kActionSize), "cudaMalloc output");
      cuda_check(cudaStreamCreate(&stream_), "cudaStreamCreate");
      if (!context_->setTensorAddress("obs", device_input_) ||
        !context_->setTensorAddress("actions", device_output_))
      {
        throw std::runtime_error("TensorRT rejected tensor addresses");
      }
    } catch (...) {
      cleanup();
      throw;
    }
  }

  ~TensorRtActor()
  {
    cleanup();
  }

  std::array<float, kActionSize> infer(
    const std::array<float, kObservationSize> & observation)
  {
    std::array<float, kActionSize> action{};
    cuda_check(
      cudaMemcpyAsync(
        device_input_, observation.data(), sizeof(float) * observation.size(),
        cudaMemcpyHostToDevice, stream_), "copy observation");
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("TensorRT enqueueV3 failed");
    }
    cuda_check(
      cudaMemcpyAsync(
        action.data(), device_output_, sizeof(float) * action.size(),
        cudaMemcpyDeviceToHost, stream_), "copy action");
    cuda_check(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
    for (float & value : action) {
      value = std::clamp(value, -1.0F, 1.0F);
    }
    return action;
  }

private:
  void validate_contract()
  {
    if (engine_->getNbIOTensors() != 2) {
      throw std::runtime_error("t60 TensorRT engine must have two IO tensors");
    }
    const auto input = engine_->getTensorShape("obs");
    const auto output = engine_->getTensorShape("actions");
    if (input.nbDims != 2 || input.d[0] != 1 || input.d[1] != kObservationSize ||
      output.nbDims != 2 || output.d[0] != 1 || output.d[1] != kActionSize ||
      engine_->getTensorDataType("obs") != nvinfer1::DataType::kFLOAT ||
      engine_->getTensorDataType("actions") != nvinfer1::DataType::kFLOAT)
    {
      throw std::runtime_error("t60 TensorRT tensor contract mismatch");
    }
  }

  void cleanup() noexcept
  {
    if (stream_ != nullptr) {cudaStreamDestroy(stream_);}
    if (device_output_ != nullptr) {cudaFree(device_output_);}
    if (device_input_ != nullptr) {cudaFree(device_input_);}
    stream_ = nullptr;
    device_output_ = nullptr;
    device_input_ = nullptr;
  }

  TensorRtLogger logger_;
  TrtPtr<nvinfer1::IRuntime> runtime_;
  TrtPtr<nvinfer1::ICudaEngine> engine_;
  TrtPtr<nvinfer1::IExecutionContext> context_;
  void * device_input_{nullptr};
  void * device_output_{nullptr};
  cudaStream_t stream_{nullptr};
};

template<typename Message>
struct StampedMessage
{
  std::int64_t stamp_ns{};
  std::shared_ptr<const Message> message;
};

template<typename Message>
void append_message(
  std::deque<StampedMessage<Message>> & history,
  const std::shared_ptr<const Message> & message)
{
  const auto & stamp = message->header.stamp;
  const std::int64_t stamp_ns =
    static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  history.push_back({stamp_ns, message});
  while (history.size() > 256) {history.pop_front();}
}

template<typename Message>
const StampedMessage<Message> * latest_at(
  const std::deque<StampedMessage<Message>> & history, std::int64_t tick_ns)
{
  for (auto item = history.rbegin(); item != history.rend(); ++item) {
    if (item->stamp_ns <= tick_ns) {return &*item;}
  }
  return nullptr;
}

class T60PolicyNode final : public rclcpp::Node
{
  using BodyState = robotcore_interfaces::msg::BodyState;
  using ThrusterCommand = robotcore_interfaces::msg::ThrusterCommand;
  using TrajectoryTarget = robotcore_interfaces::msg::TrajectoryTarget;
  using PolicyStatus = robotcore_interfaces::msg::PolicyStatus;
  using Imu = sensor_msgs::msg::Imu;

public:
  T60PolicyNode()
  : Node("t60_policy")
  {
    policy_name_ = declare_parameter<std::string>(
      "policy_name", "t60_precision_v7_model_499");
    model_path_ = declare_parameter<std::string>("model_path", "");
    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 25.0);
    max_input_age_ns_ = static_cast<std::int64_t>(
      declare_parameter<double>("max_input_age_s", 0.25) * 1.0e9);
    if (model_path_.empty()) {throw std::runtime_error("model_path is required");}
    actor_ = std::make_unique<TensorRtActor>(model_path_);

    action_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/policy/body/action", rclcpp::QoS(10));
    status_pub_ = create_publisher<PolicyStatus>(
      "/policy/body/status", rclcpp::QoS(10));
    body_sub_ = create_subscription<BodyState>(
      "/robot/body_state", rclcpp::QoS(4).reliable(),
      [this](BodyState::ConstSharedPtr message) {append_message(body_, message);});
    imu_sub_ = create_subscription<Imu>(
      "/sensors/external_imu", rclcpp::SensorDataQoS().keep_last(16),
      [this](Imu::ConstSharedPtr message) {append_message(imu_, message);});
    command_sub_ = create_subscription<ThrusterCommand>(
      "/control/thruster_cmd", rclcpp::QoS(4).reliable(),
      [this](ThrusterCommand::ConstSharedPtr message) {append_message(command_, message);});
    target_sub_ = create_subscription<TrajectoryTarget>(
      "/runtime/trajectory_target", rclcpp::QoS(4).reliable(),
      [this](TrajectoryTarget::ConstSharedPtr message) {append_message(target_, message);});
    run_dir_sub_ = create_subscription<std_msgs::msg::String>(
      "/runtime/run_dir", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {set_run_dir(message->data);});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz),
      std::bind(&T60PolicyNode::tick, this));
    RCLCPP_INFO(
      get_logger(), "C++ t60 policy started at %.1f Hz using %s",
      control_rate_hz, model_path_.c_str());
  }

private:
  static Vec3 vector(const geometry_msgs::msg::Vector3 & value)
  {
    return {value.x, value.y, value.z};
  }

  static Vec3 point(const geometry_msgs::msg::Point & value)
  {
    return {value.x, value.y, value.z};
  }

  static Quaternion quaternion(const geometry_msgs::msg::Quaternion & value)
  {
    return normalized({value.w, value.x, value.y, value.z});
  }

  static void write_vec(
    std::array<float, kCurrentSize> & output, std::size_t offset, const Vec3 & value)
  {
    output[offset] = static_cast<float>(value.x);
    output[offset + 1] = static_cast<float>(value.y);
    output[offset + 2] = static_cast<float>(value.z);
  }

  std::array<float, kCurrentSize> build_current(
    const BodyState & body, const Imu & imu, const ThrusterCommand & command,
    const TrajectoryTarget & target) const
  {
    const Vec3 measured_position_w = point(body.pose.position);
    const Quaternion localization_q_w = quaternion(body.pose.orientation);
    const Quaternion imu_q_w = quaternion(imu.orientation);
    const Vec3 localization_rpy = quaternion_to_rpy(localization_q_w);
    const Vec3 imu_rpy = quaternion_to_rpy(imu_q_w);
    // Match the control/UI attitude contract: low-latency external-IMU
    // roll/pitch plus the absolute map yaw from BodyState localization.
    const Quaternion measured_q_w = quaternion_from_rpy(
      {imu_rpy.x, imu_rpy.y, localization_rpy.z});
    const Quaternion q_bw = conjugate(measured_q_w);
    const Vec3 measured_linear_velocity_b = vector(body.twist.linear);
    const Vec3 measured_angular_velocity_b = vector(imu.angular_velocity);
    const Vec3 target_position_w = point(target.target_pose.position);
    const Quaternion target_q_w = quaternion(target.target_pose.orientation);
    const Vec3 target_linear_velocity_w = vector(target.target_twist.linear);
    const Vec3 target_angular_velocity_w = vector(target.target_twist.angular);
    const Vec3 target_linear_acceleration_w = vector(target.target_accel.linear);

    const Vec3 position_error_b = rotate(q_bw, target_position_w - measured_position_w);
    const Vec3 target_linear_velocity_b = rotate(q_bw, target_linear_velocity_w);
    const Vec3 linear_velocity_error_b =
      target_linear_velocity_b - measured_linear_velocity_b;
    Quaternion attitude_error = normalized(multiply(q_bw, target_q_w));
    if (attitude_error.w < 0.0) {
      attitude_error.w = -attitude_error.w;
      attitude_error.x = -attitude_error.x;
      attitude_error.y = -attitude_error.y;
      attitude_error.z = -attitude_error.z;
    }
    const Vec3 projected_gravity_b = rotate(q_bw, {0.0, 0.0, -1.0});
    const Vec3 target_angular_velocity_b = rotate(q_bw, target_angular_velocity_w);
    const Vec3 target_linear_acceleration_b = rotate(q_bw, target_linear_acceleration_w);

    std::array<float, kCurrentSize> current{};
    write_vec(current, 0, position_error_b);
    write_vec(current, 3, target_linear_velocity_b);
    write_vec(current, 6, linear_velocity_error_b);
    current[9] = static_cast<float>(attitude_error.w);
    current[10] = static_cast<float>(attitude_error.x);
    current[11] = static_cast<float>(attitude_error.y);
    current[12] = static_cast<float>(attitude_error.z);
    write_vec(current, 13, projected_gravity_b);
    write_vec(current, 16, measured_angular_velocity_b);
    write_vec(current, 19, target_angular_velocity_b);
    write_vec(current, 22, target_linear_acceleration_b);
    for (std::size_t channel = 0; channel < kActionSize; ++channel) {
      current[25 + channel] = command.action[channel];
    }
    for (std::size_t index = 0; index < current.size(); ++index) {
      current[index] /= kObservationScale[index];
    }
    return current;
  }

  std::array<float, kObservationSize> pack_observation(
    const std::array<float, kCurrentSize> & current) const
  {
    std::array<float, kObservationSize> packed{};
    std::copy(current.begin(), current.end(), packed.begin());
    std::size_t offset = kCurrentSize;
    for (std::size_t frame = 0; frame < kHistoryFrames; ++frame) {
      if (frame < history_.size()) {
        std::copy(
          history_[frame].begin(), history_[frame].end(), packed.begin() + offset);
      }
      offset += kHistoryFrameSize;
    }
    return packed;
  }

  static std::array<float, kHistoryFrameSize> history_frame(
    const std::array<float, kCurrentSize> & current)
  {
    std::array<float, kHistoryFrameSize> frame{};
    for (std::size_t index = 0; index < frame.size(); ++index) {
      frame[index] = current[kHistoryIndices[index]];
    }
    return frame;
  }

  template<typename Message>
  bool valid_age(
    const StampedMessage<Message> * sample, std::int64_t tick_ns,
    const std::string & name, std::vector<std::string> & missing) const
  {
    if (sample == nullptr || tick_ns - sample->stamp_ns > max_input_age_ns_) {
      missing.push_back(name);
      return false;
    }
    return true;
  }

  void tick()
  {
    const auto tick_time = now();
    const std::int64_t tick_ns = tick_time.nanoseconds();
    const auto * body = latest_at(body_, tick_ns);
    const auto * imu = latest_at(imu_, tick_ns);
    const auto * command = latest_at(command_, tick_ns);
    const auto * target = latest_at(target_, tick_ns);
    std::vector<std::string> missing;
    valid_age(body, tick_ns, "/robot/body_state", missing);
    valid_age(imu, tick_ns, "/sensors/external_imu", missing);
    valid_age(command, tick_ns, "/control/thruster_cmd", missing);
    valid_age(target, tick_ns, "/runtime/trajectory_target", missing);
    if (body != nullptr &&
      !(body->message->state_valid || body->message->position_estimated))
    {
      missing.emplace_back("/robot/body_state(localization_invalid)");
    }
    if (body != nullptr && !body->message->linear_velocity_valid) {
      missing.emplace_back("/robot/body_state(linear_velocity_invalid)");
    }
    if (target != nullptr && !target->message->valid) {
      missing.emplace_back("/runtime/trajectory_target(invalid)");
    }
    if (!missing.empty()) {
      history_.clear();
      publish_status(tick_time, false, missing, 0.0F);
      return;
    }

    const auto started = std::chrono::steady_clock::now();
    try {
      const auto current = build_current(
        *body->message, *imu->message, *command->message, *target->message);
      const auto action = actor_->infer(pack_observation(current));
      const float latency_ms = static_cast<float>(
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count());
      std_msgs::msg::Float32MultiArray message;
      message.data.assign(action.begin(), action.end());
      action_pub_->publish(message);
      history_.push_front(history_frame(current));
      if (history_.size() > kHistoryFrames) {history_.pop_back();}
      ++inference_count_;
      write_policy_io(tick_time.nanoseconds(), action);
      publish_status(tick_time, true, {}, latency_ms);
    } catch (const std::exception & error) {
      history_.clear();
      publish_status(tick_time, false, {std::string("inference: ") + error.what()}, 0.0F);
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", error.what());
    }
  }

  void publish_status(
    const rclcpp::Time & stamp, bool ready,
    const std::vector<std::string> & missing, float latency_ms)
  {
    PolicyStatus status;
    status.header.stamp = stamp;
    status.policy_role = "body";
    status.policy_name = policy_name_;
    status.runner = "tensorrt_cpp";
    status.model_path = model_path_;
    status.loaded = actor_ != nullptr;
    status.input_ready = ready;
    status.missing_inputs = missing;
    status.inference_latency_ms = latency_ms;
    status.inference_count = inference_count_;
    status_pub_->publish(status);
  }

  void set_run_dir(const std::string & run_dir)
  {
    const auto directory = std::filesystem::path(run_dir) / "policy_io";
    std::filesystem::create_directories(directory);
    policy_io_.close();
    policy_io_.open(directory / "body_policy_io.jsonl", std::ios::app);
  }

  void write_policy_io(
    std::int64_t stamp_ns, const std::array<float, kActionSize> & action)
  {
    if (!policy_io_.is_open()) {return;}
    policy_io_ << "{\"action\":[";
    for (std::size_t index = 0; index < action.size(); ++index) {
      if (index > 0) {policy_io_ << ',';}
      policy_io_ << action[index];
    }
    policy_io_ << "],\"observation_order\":\"latest_first\",\"policy\":\""
               << policy_name_ << "\",\"time\":" << stamp_ns << "}\n";
    policy_io_.flush();
  }

  std::string policy_name_;
  std::string model_path_;
  std::int64_t max_input_age_ns_{};
  std::uint32_t inference_count_{};
  std::unique_ptr<TensorRtActor> actor_;
  std::deque<StampedMessage<BodyState>> body_;
  std::deque<StampedMessage<Imu>> imu_;
  std::deque<StampedMessage<ThrusterCommand>> command_;
  std::deque<StampedMessage<TrajectoryTarget>> target_;
  std::deque<std::array<float, kHistoryFrameSize>> history_;
  std::ofstream policy_io_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;
  rclcpp::Publisher<PolicyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<BodyState>::SharedPtr body_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<ThrusterCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<TrajectoryTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr run_dir_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace robotcore_policy_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotcore_policy_cpp::T60PolicyNode>());
  rclcpp::shutdown();
  return 0;
}
