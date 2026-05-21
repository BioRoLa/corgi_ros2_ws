#ifndef IMU_RAW_HPP
#define IMU_RAW_HPP

#ifndef SIMULATION
#ifndef DEBUG
#define DEBUG
#endif
#include <mip/mip_all.hpp>
#include <mip/mip_device.hpp>
#include <mip/platform/serial_connection.hpp>
#include <mip/mip_logging.h>
#include <array>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cmath>

using namespace mip;

// ── Shared utilities (same as cx5.hpp) ───────────────────────────────────────

static inline Timestamp getCurrentTimestamp_raw(){
    using namespace std::chrono;
    return duration_cast<milliseconds>( steady_clock::now().time_since_epoch() ).count();
}

struct RawUtils{
    std::unique_ptr<Connection> connection;
    std::unique_ptr<DeviceInterface> device;
    uint8_t buffer[1024];
};

static inline std::unique_ptr<RawUtils> assign_serial_raw(const std::string& port, uint32_t baud){
    auto utils = std::unique_ptr<RawUtils>(new RawUtils());
    if( baud == 0 )
        throw std::runtime_error("Serial baud rate must be a decimal integer greater than 0.");

    using SerialConnection = mip::platform::SerialConnection;
    utils->connection = std::unique_ptr<SerialConnection>(new SerialConnection(port, baud));
    utils->device = std::unique_ptr<mip::DeviceInterface>(
        new mip::DeviceInterface(utils->connection.get(), utils->buffer,
                                  sizeof(utils->buffer),
                                  mip::C::mip_timeout_from_baudrate(baud), 500));

    if( !utils->connection->connect() )
        throw std::runtime_error("Failed to open the connection");

    return utils;
}

// ── CX5_RAW: Scaled sensor data at 1000 Hz ──────────────────────────────────

class CX5_RAW {
public:
    CX5_RAW(const std::string& port, uint32_t baud, uint16_t sample_rate)
        : sample_rate_(sample_rate), running_(false)
    {
        (void)baud; // actual baud is always 921600
        utils_ = assign_serial_raw(port, 921600);

        if(commands_base::ping(*utils_->device) != CmdResult::ACK_OK)
            throw std::runtime_error("ERROR: Could not ping the device!");
    }

    void start() {
        running_ = true;
        auto& device = utils_->device;

        if(commands_base::ping(*device) != CmdResult::ACK_OK)
            throw std::runtime_error("ERROR: Could not ping the device!");

        // ── Sensor base rate (typically 1000 Hz on CX5-AHRS) ────────────
        uint16_t sensor_base_rate;
        if(commands_3dm::imuGetBaseRate(*device, &sensor_base_rate) != CmdResult::ACK_OK)
            throw std::runtime_error("ERROR: Could not get sensor base rate!");

        const uint16_t decimation = sensor_base_rate / sample_rate_;

        // Request scaled accel + scaled gyro from the sensor descriptor set
        std::array<DescriptorRate, 2> descriptors = {{
            { data_sensor::DATA_ACCEL_SCALED, decimation },
            { data_sensor::DATA_GYRO_SCALED,  decimation },
        }};

        if(commands_3dm::writeImuMessageFormat(*device, descriptors.size(), descriptors.data()) != CmdResult::ACK_OK)
            throw std::runtime_error("ERROR: Could not set sensor message format!");

        // Physical mounting: the sensor is installed rotated pi around X relative to
        // the robot body, so configure sensor->vehicle rotation as [pi, 0, 0].
        float euler[3] = { static_cast<float>(M_PI), 0.0f, 0.0f };
        if(commands_filter::writeSensorToVehicleRotationEuler(*device, euler[0], euler[1], euler[2]) != CmdResult::ACK_OK)
            throw std::runtime_error("ERROR: Could not set sensor-to-vehicle transformation!");

        // Register member-function callbacks
        device->registerDataCallback<data_sensor::ScaledAccel, CX5_RAW, &CX5_RAW::on_accel>(handlers_[0], this);
        device->registerDataCallback<data_sensor::ScaledGyro,  CX5_RAW, &CX5_RAW::on_gyro>(handlers_[1], this);

        {
            std::lock_guard<std::mutex> lk(mutex_);
            last_data_time_ = std::chrono::steady_clock::now();
        }

        if(commands_base::resume(*device) != CmdResult::ACK_OK)
            throw std::runtime_error("ERROR: Could not resume the device!");

        while(running_) {
            device->update();
        }
    }

    /// Block until the next sensor sample is ready, then copy it out.
    /// Returns false on timeout (no data within 5 ms).
    bool wait_and_get(Eigen::Vector3f &accel, Eigen::Vector3f &gyro,
                      std::chrono::steady_clock::time_point &data_time) {
        std::unique_lock<std::mutex> lk(mutex_);
        if (!data_cv_.wait_for(lk, std::chrono::milliseconds(5),
                               [this]{ return new_data_; }))
            return false;
        new_data_ = false;
        accel     = acceleration_;
        gyro      = angular_velocity_;
        data_time = last_data_time_;
        return true;
    }

    void stop() { running_ = false; }

private:
    std::unique_ptr<RawUtils> utils_;
    uint16_t sample_rate_;

    Eigen::Vector3f acceleration_      = Eigen::Vector3f::Zero();
    Eigen::Vector3f angular_velocity_  = Eigen::Vector3f::Zero();

    std::mutex mutex_;
    std::condition_variable data_cv_;
    bool new_data_ = false;
    std::atomic<bool> running_;
    std::chrono::steady_clock::time_point last_data_time_;
    DispatchHandler handlers_[2];

    // Frame conversion for published vectors: NED -> NWU.
    // This is equivalent to a pi rotation around X, i.e. diag(1, -1, -1).
    static Eigen::Matrix3f rot() {
        Eigen::Matrix3f r;
        r << 1, 0, 0,
             0,-1, 0,
             0, 0,-1;
        return r;
    }

    void on_accel(const data_sensor::ScaledAccel& data, Timestamp /*timestamp*/) {
        std::lock_guard<std::mutex> lk(mutex_);
        // ScaledAccel is in [g]; convert to [m/s²]
        constexpr float G = 9.80665f;
        acceleration_ = rot() * (Eigen::Vector3f(data.scaled_accel) * G);
        last_data_time_ = std::chrono::steady_clock::now();
    }

    void on_gyro(const data_sensor::ScaledGyro& data, Timestamp /*timestamp*/) {
        {
            std::lock_guard<std::mutex> lk(mutex_);
            // ScaledGyro is already in [rad/s]
            angular_velocity_ = rot() * Eigen::Vector3f(data.scaled_gyro);
            last_data_time_ = std::chrono::steady_clock::now();
            new_data_ = true;
        }
        // Notify AFTER releasing the lock so the waiter can acquire it
        data_cv_.notify_one();
    }
};

#endif // SIMULATION
#endif // IMU_RAW_HPP
