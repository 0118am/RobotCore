#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <pthread.h>
#include <sched.h>

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
constexpr std::size_t kCurrentSize = 30;
constexpr std::size_t kHistoryFrames = 8;
constexpr std::size_t kHistoryFrameSize = 21;
constexpr std::size_t kObservationSize = kCurrentSize + kHistoryFrames * kHistoryFrameSize;
constexpr std::size_t kActionSize = 8;
static_assert(kObservationSize == 198);

void configure_fifo_thread(
  const rclcpp::Logger & logger, const char * thread_name, int priority)
{
  if (priority <= 0) {return;}
  sched_param parameters{};
  parameters.sched_priority = priority;
  const int error = pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters);
  if (error != 0) {
    RCLCPP_ERROR(
      logger, "Failed to set %s to SCHED_FIFO/%d: %s",
      thread_name, priority, std::strerror(error));
    return;
  }
  RCLCPP_INFO(logger, "%s uses SCHED_FIFO/%d", thread_name, priority);
}

constexpr std::array<float, kCurrentSize> kObservationScale{
  0.25F, 0.25F, 0.25F,
  0.40F, 0.40F, 0.40F,
  0.20F, 0.20F, 0.20F,
  1.00F, 1.00F, 1.00F, 1.00F,
  0.80F, 0.80F, 0.80F,
  0.80F, 0.80F, 0.80F,
  0.45F, 0.45F, 0.45F,
  1.00F, 1.00F, 1.00F, 1.00F, 1.00F, 1.00F, 1.00F, 1.00F};

constexpr std::array<float, kHistoryFrameSize> kHistoryScale{
  0.25F, 0.25F, 0.25F,
  0.20F, 0.20F, 0.20F,
  1.00F, 1.00F, 1.00F, 1.00F,
  0.80F, 0.80F, 0.80F,
  1.00F, 1.00F, 1.00F, 1.00F, 1.00F, 1.00F, 1.00F, 1.00F};

struct Vec3
{
  double x{};
  double y{};
  double z{};
};

struct ObservationFrame
{
  std::array<float, kCurrentSize> current{};
  std::array<float, kHistoryFrameSize> history{};
  // Raw target-minus-measured body rate, before observation normalization.
  Vec3 angular_velocity_error_b{};
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

void apply_vertical_rate_damping(
  std::array<float, kActionSize> & action,
  const Vec3 & target_minus_measured_rate_b,
  double roll_gain, double pitch_gain, double correction_limit)
{
  // T1/T2 are starboard, T3/T4 port; T1/T3 are forward, T2/T4 aft.
  // Positive vertical action produces -Z thrust. These signs therefore make
  // r x F oppose measured roll/pitch rate relative to the trajectory target.
  const double measured_minus_target_roll = -target_minus_measured_rate_b.x;
  const double measured_minus_target_pitch = -target_minus_measured_rate_b.y;
  const double roll = roll_gain * measured_minus_target_roll;
  const double pitch = pitch_gain * measured_minus_target_pitch;
  const std::array<double, 4> correction{
    -roll - pitch,
    -roll + pitch,
    roll - pitch,
    roll + pitch};
  for (std::size_t channel = 0; channel < correction.size(); ++channel) {
    const float bounded = static_cast<float>(
      std::clamp(correction[channel], -correction_limit, correction_limit));
    action[channel] = std::clamp(action[channel] + bounded, -1.0F, 1.0F);
  }
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
        builder->createNetworkV2(0U)};
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
      int least_priority = 0;
      int greatest_priority = 0;
      cuda_check(
        cudaDeviceGetStreamPriorityRange(&least_priority, &greatest_priority),
        "cudaDeviceGetStreamPriorityRange");
      cuda_check(
        cudaStreamCreateWithPriority(
          &stream_, cudaStreamNonBlocking, greatest_priority),
        "cudaStreamCreateWithPriority");
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

// Policy logging is diagnostic-only. Never let filesystem latency block the
// SCHED_FIFO inference executor: a bounded queue transfers records to a normal
// worker, and contention drops a log record instead of delaying control.
class AsyncPolicyIoWriter
{
public:
  struct Record
  {
    std::int64_t stamp_ns{};
    std::array<float, kActionSize> action{};
  };

  AsyncPolicyIoWriter()
  : thread_([this]() {run();})
  {
  }

  ~AsyncPolicyIoWriter()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    condition_.notify_one();
    if (thread_.joinable()) {thread_.join();}
  }

  AsyncPolicyIoWriter(const AsyncPolicyIoWriter &) = delete;
  AsyncPolicyIoWriter & operator=(const AsyncPolicyIoWriter &) = delete;

  void set_run_dir(const std::string & run_dir, const std::string & policy_name)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_path_ = run_dir.empty() ? std::filesystem::path{} :
        std::filesystem::path(run_dir) / "policy_io" / "body_policy_io.jsonl";
      pending_policy_name_ = policy_name;
      path_changed_ = true;
      active_ = !run_dir.empty();
      head_ = 0U;
      size_ = 0U;
    }
    condition_.notify_one();
  }

  void enqueue(
    std::int64_t stamp_ns,
    const std::array<float, kActionSize> & action) noexcept
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !active_) {return;}
    if (size_ == records_.size()) {
      head_ = (head_ + 1U) % records_.size();
      --size_;
    }
    const std::size_t tail = (head_ + size_) % records_.size();
    records_[tail] = Record{stamp_ns, action};
    ++size_;
  }

private:
  static constexpr std::size_t kCapacity = 512U;

  void run()
  {
    std::ofstream output;
    std::string policy_name;
    while (true) {
      std::vector<Record> pending;
      std::filesystem::path path;
      std::string next_policy_name;
      bool reopen = false;
      bool stop = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(
          lock, 250ms,
          [this]() {return stop_ || path_changed_ || size_ > 0U;});
        if (path_changed_) {
          path = pending_path_;
          next_policy_name = pending_policy_name_;
          path_changed_ = false;
          reopen = true;
        }
        pending.reserve(size_);
        while (size_ > 0U) {
          pending.push_back(records_[head_]);
          head_ = (head_ + 1U) % records_.size();
          --size_;
        }
        stop = stop_;
      }

      if (reopen) {
        output.close();
        policy_name = std::move(next_policy_name);
        if (!path.empty()) {
          std::error_code error;
          std::filesystem::create_directories(path.parent_path(), error);
          if (!error) {output.open(path, std::ios::app);}
        }
      }
      if (output.is_open()) {
        for (const auto & record : pending) {
          output << "{\"action\":[";
          for (std::size_t index = 0; index < record.action.size(); ++index) {
            if (index > 0U) {output << ',';}
            output << record.action[index];
          }
          output << "],\"observation_order\":\"latest_first\",\"policy\":\""
                 << policy_name << "\",\"time\":" << record.stamp_ns << "}\n";
        }
        if (!pending.empty() || stop) {output.flush();}
      }
      if (stop) {return;}
    }
  }

  std::array<Record, kCapacity> records_{};
  std::size_t head_{0U};
  std::size_t size_{0U};
  std::mutex mutex_;
  std::condition_variable condition_;
  std::filesystem::path pending_path_;
  std::string pending_policy_name_;
  bool path_changed_{false};
  bool active_{false};
  bool stop_{false};
  std::thread thread_;
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
    const auto realtime_priority = declare_parameter<std::int64_t>(
      "executor_realtime_priority", 0);
    if (realtime_priority < 0 || realtime_priority > 99) {
      throw std::invalid_argument("executor_realtime_priority must be in [0, 99]");
    }
    executor_realtime_priority_ = static_cast<int>(realtime_priority);
    policy_name_ = declare_parameter<std::string>(
      "policy_name", "t60_precision_v17_model_400");
    model_path_ = declare_parameter<std::string>("model_path", "");
    const double control_rate_hz = std::max(
      declare_parameter<double>("control_rate_hz", 25.0), 1.0);
    control_period_s_ = 1.0 / std::max(control_rate_hz, 1.0);
    max_input_age_ns_ = static_cast<std::int64_t>(
      declare_parameter<double>("max_input_age_s", 0.25) * 1.0e9);
    rate_damping_enabled_ = declare_parameter<bool>("rate_damping_enabled", true);
    roll_rate_damping_gain_ = declare_parameter<double>(
      "roll_rate_damping_gain_action_per_rps", 0.10);
    pitch_rate_damping_gain_ = declare_parameter<double>(
      "pitch_rate_damping_gain_action_per_rps", 0.10);
    rate_damping_action_limit_ = declare_parameter<double>(
      "rate_damping_action_limit", 0.08);
    if (!std::isfinite(roll_rate_damping_gain_) || roll_rate_damping_gain_ < 0.0 ||
      !std::isfinite(pitch_rate_damping_gain_) || pitch_rate_damping_gain_ < 0.0 ||
      !std::isfinite(rate_damping_action_limit_) ||
      rate_damping_action_limit_ <= 0.0 || rate_damping_action_limit_ > 1.0)
    {
      throw std::invalid_argument("rate damping parameters are invalid");
    }
    if (model_path_.empty()) {throw std::runtime_error("model_path is required");}
    actor_ = std::make_unique<TensorRtActor>(model_path_);

    action_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/policy/body/action", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    status_pub_ = create_publisher<PolicyStatus>(
      "/policy/body/status", rclcpp::QoS(1));
    body_sub_ = create_subscription<BodyState>(
      "/robot/body_state", rclcpp::QoS(1).reliable(),
      [this](BodyState::ConstSharedPtr message) {append_message(body_, message);});
    imu_sub_ = create_subscription<Imu>(
      "/sensors/external_imu", rclcpp::SensorDataQoS().keep_last(1),
      [this](Imu::ConstSharedPtr message) {append_message(imu_, message);});
    command_sub_ = create_subscription<ThrusterCommand>(
      "/control/thruster_cmd", rclcpp::QoS(1).reliable(),
      [this](ThrusterCommand::ConstSharedPtr message) {append_message(command_, message);});
    target_sub_ = create_subscription<TrajectoryTarget>(
      "/runtime/trajectory_target", rclcpp::QoS(1).reliable(),
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

  void configure_executor_thread() const
  {
    configure_fifo_thread(
      get_logger(), "T60 policy executor", executor_realtime_priority_);
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

  template<std::size_t Size>
  static void write_vec(
    std::array<float, Size> & output, std::size_t offset, const Vec3 & value)
  {
    output[offset] = static_cast<float>(value.x);
    output[offset + 1] = static_cast<float>(value.y);
    output[offset + 2] = static_cast<float>(value.z);
  }

  ObservationFrame build_frame(
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
    const Vec3 target_angular_velocity_b = rotate(q_bw, target_angular_velocity_w);
    const Vec3 angular_velocity_error_b =
      target_angular_velocity_b - measured_angular_velocity_b;
    const Vec3 target_linear_acceleration_b = rotate(q_bw, target_linear_acceleration_w);

    ObservationFrame frame;
    write_vec(frame.current, 0, position_error_b);
    write_vec(frame.current, 3, target_linear_velocity_b);
    write_vec(frame.current, 6, linear_velocity_error_b);
    frame.current[9] = static_cast<float>(attitude_error.w);
    frame.current[10] = static_cast<float>(attitude_error.x);
    frame.current[11] = static_cast<float>(attitude_error.y);
    frame.current[12] = static_cast<float>(attitude_error.z);
    write_vec(frame.current, 13, measured_angular_velocity_b);
    write_vec(frame.current, 16, target_angular_velocity_b);
    write_vec(frame.current, 19, target_linear_acceleration_b);
    for (std::size_t channel = 0; channel < kActionSize; ++channel) {
      frame.current[22 + channel] = command.action[channel];
    }

    write_vec(frame.history, 0, position_error_b);
    write_vec(frame.history, 3, linear_velocity_error_b);
    frame.history[6] = static_cast<float>(attitude_error.w);
    frame.history[7] = static_cast<float>(attitude_error.x);
    frame.history[8] = static_cast<float>(attitude_error.y);
    frame.history[9] = static_cast<float>(attitude_error.z);
    write_vec(frame.history, 10, angular_velocity_error_b);
    frame.angular_velocity_error_b = angular_velocity_error_b;
    for (std::size_t channel = 0; channel < kActionSize; ++channel) {
      frame.history[13 + channel] = command.action[channel];
    }
    for (std::size_t index = 0; index < frame.current.size(); ++index) {
      frame.current[index] /= kObservationScale[index];
    }
    for (std::size_t index = 0; index < frame.history.size(); ++index) {
      frame.history[index] /= kHistoryScale[index];
    }
    return frame;
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
    const auto steady_tick = std::chrono::steady_clock::now();
    if (last_tick_) {
      tick_interval_ms_ = static_cast<float>(
        std::chrono::duration<double, std::milli>(steady_tick - *last_tick_).count());
      if (tick_interval_ms_ > control_period_s_ * 1500.0) {
        ++deadline_miss_count_;
      }
    }
    last_tick_ = steady_tick;
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
      const auto frame = build_frame(
        *body->message, *imu->message, *command->message, *target->message);
      auto action = actor_->infer(pack_observation(frame.current));
      if (rate_damping_enabled_) {
        apply_vertical_rate_damping(
          action, frame.angular_velocity_error_b,
          roll_rate_damping_gain_, pitch_rate_damping_gain_,
          rate_damping_action_limit_);
      }
      const float latency_ms = static_cast<float>(
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count());
      std_msgs::msg::Float32MultiArray message;
      message.data.assign(action.begin(), action.end());
      action_pub_->publish(message);
      history_.push_front(frame.history);
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
    status.tick_interval_ms = tick_interval_ms_;
    status.deadline_miss_count = deadline_miss_count_;
    status_pub_->publish(status);
  }

  void set_run_dir(const std::string & run_dir)
  {
    policy_io_writer_.set_run_dir(run_dir, policy_name_);
  }

  void write_policy_io(
    std::int64_t stamp_ns, const std::array<float, kActionSize> & action)
  {
    policy_io_writer_.enqueue(stamp_ns, action);
  }

  std::string policy_name_;
  std::string model_path_;
  std::int64_t max_input_age_ns_{};
  int executor_realtime_priority_{};
  std::uint32_t inference_count_{};
  std::uint32_t deadline_miss_count_{};
  double control_period_s_{};
  bool rate_damping_enabled_{true};
  double roll_rate_damping_gain_{};
  double pitch_rate_damping_gain_{};
  double rate_damping_action_limit_{};
  float tick_interval_ms_{};
  std::optional<std::chrono::steady_clock::time_point> last_tick_;
  std::unique_ptr<TensorRtActor> actor_;
  std::deque<StampedMessage<BodyState>> body_;
  std::deque<StampedMessage<Imu>> imu_;
  std::deque<StampedMessage<ThrusterCommand>> command_;
  std::deque<StampedMessage<TrajectoryTarget>> target_;
  std::deque<std::array<float, kHistoryFrameSize>> history_;
  AsyncPolicyIoWriter policy_io_writer_;

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
  auto node = std::make_shared<robotcore_policy_cpp::T60PolicyNode>();
  node->configure_executor_thread();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
