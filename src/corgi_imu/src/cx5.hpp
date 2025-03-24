#ifndef IMU_HPP
#define IMU_HPP

#ifndef SIMULATION
#include <mip/mip_all.hpp>
#include <mip/mip_device.hpp>
#include <mip/platform/serial_connection.hpp>
#include <mip/mip_logging.h>
#include <array>
#include <vector>
#include <chrono>
#include <thread>
#include <stdexcept>
// #include <stdarg.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <iostream>
#include <mutex>
#include "math.h"

using namespace mip;  
using namespace std::chrono;

Timestamp getCurrentTimestamp(){
    return duration_cast<milliseconds>( steady_clock::now().time_since_epoch() ).count();
}

struct Utils{
    std::unique_ptr<Connection> connection;
    std::unique_ptr<DeviceInterface> device;
    uint8_t buffer[1024];
};

// connect the device by serial port
std::unique_ptr<Utils> assign_serial(std::string port, uint32_t baud){
    if( baud == 0 )
        throw std::runtime_error("Serial baud rate must be a decimal integer greater than 0.");
    
    auto utils = std::unique_ptr<Utils>(new Utils());
    using SerialConnection = mip::platform::SerialConnection;
    utils->connection = std::unique_ptr<SerialConnection>(new SerialConnection(port, baud));
    utils->device = std::unique_ptr<mip::DeviceInterface>(new mip::DeviceInterface(utils->connection.get(), utils->buffer, sizeof(utils->buffer), mip::C::mip_timeout_from_baudrate(baud), 500));

    if( !utils->connection->connect() )
        throw std::runtime_error("Failed to open the connection");

    return utils;
}

class CX5_AHRS {
    public:
        CX5_AHRS(std::string port, uint32_t baud, uint16_t _sensor_sample_rate, uint16_t _filter_sample_rate) {
            utils = assign_serial(port, baud);
            sensor_sample_rate = _sensor_sample_rate;
            filter_sample_rate = _filter_sample_rate;
            twist_bias = Eigen::Vector3f(0,0,0);
        }

        void start() {
            std::unique_ptr<DeviceInterface>& device = utils->device;
            
            while (commands_base::ping(*device) != CmdResult::ACK_OK){
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                std::cout << "Waiting for connection ...\n";
            }

            uint16_t sensor_base_rate;
            if(commands_3dm::imuGetBaseRate(*device, &sensor_base_rate) != CmdResult::ACK_OK)
                throw std::runtime_error("ERROR: Could not get sensor base rate format!");

            const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;
            std::array<DescriptorRate, 1> sensor_descriptors = {{
                { data_sensor::DATA_COMP_QUATERNION, sensor_decimation },
            }};

            if(commands_3dm::writeImuMessageFormat(*device, sensor_descriptors.size(), sensor_descriptors.data()) != CmdResult::ACK_OK)
                throw std::runtime_error("ERROR: Could not set sensor message format!");

            float sensor_to_vehicle_transformation_euler[3] = {M_PI, 0, 0};
            if(commands_filter::writeSensorToVehicleRotationEuler(*device, sensor_to_vehicle_transformation_euler[0], sensor_to_vehicle_transformation_euler[1], sensor_to_vehicle_transformation_euler[2]) != CmdResult::ACK_OK)
                throw std::runtime_error("ERROR: Could not set sensor-to-vehicle transformation!");
            
            uint16_t filter_base_rate;
            if(commands_3dm::filterGetBaseRate(*device, &filter_base_rate) != CmdResult::ACK_OK)
                throw std::runtime_error("ERROR: Could not get filter base rate format!");

            const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;
            std::array<DescriptorRate, 3> filter_descriptors = {{
                { data_filter::DATA_COMPENSATED_ACCELERATION, filter_decimation },
                { data_filter::DATA_COMPENSATED_ANGULAR_RATE, filter_decimation },
                { data_filter::DATA_GRAVITY_VECTOR,           filter_decimation },
            }};

            if(commands_3dm::writeFilterMessageFormat(*device, filter_descriptors.size(), filter_descriptors.data()) != CmdResult::ACK_OK)
                throw std::runtime_error("ERROR: Could not set filter message format!");

            if(commands_3dm::writeComplementaryFilter(*device, true, false, 10., 1.) != CmdResult::ACK_OK)
                throw std::runtime_error("ERROR: Could not set north complement off!");

            if(commands_filter::writeHeadingSource(*device, commands_filter::HeadingSource::Source::NONE) != CmdResult::ACK_OK)
                throw std::runtime_error("ERROR: Could not set filter heading update control!");

            if(commands_filter::writeAutoInitControl(*device, 1) != CmdResult::ACK_OK)
                throw std::runtime_error("ERROR: Could not set filter autoinit control!");

            // if(commands_filter::reset(*device) != CmdResult::ACK_OK)
            //     throw std::runtime_error("ERROR: Could not reset the filter!");

            DispatchHandler sensor_data_handlers[4];
            device->registerExtractor(sensor_data_handlers[0], &raw_attitude);
            device->registerExtractor(sensor_data_handlers[1], &raw_gyro);
            device->registerExtractor(sensor_data_handlers[2], &raw_accel);
            device->registerExtractor(sensor_data_handlers[3], &g);

            if(commands_base::resume(*device) != CmdResult::ACK_OK)
                throw std::runtime_error("ERROR: Could not resume the device!");

            // bool filter_state_ahrs = false;
            Eigen::Matrix3f rot;
            rot << 1, 0, 0, 0, -1, 0, 0, 0, -1;
            
            while(true) {
                device->update();
                // if((!filter_state_ahrs)){
                //     if (filter_status.filter_state == data_filter::FilterMode::AHRS) filter_state_ahrs = true;
                //     else continue;
                // }
                _imu_mutex.lock();
                acceleration = rot * (Eigen::Vector3f(raw_accel.accel) - attitude.toRotationMatrix().transpose() * Eigen::Vector3f(g.gravity));
                twist = rot * (Eigen::Vector3f(raw_gyro.gyro)) - twist_bias;
                attitude = Eigen::Quaternionf(raw_attitude.q[0], raw_attitude.q[1], raw_attitude.q[2], raw_attitude.q[3]);
                _imu_mutex.unlock();
            }
        }

        void calibrate(int block_time_ms, bool calibrate_gravity = false){
            Eigen::Vector3f acceleration_;
            Eigen::Vector3f twist_;
            Eigen::Quaternionf attitude_;
            Eigen::Vector3f t_sum(0, 0, 0);
            for (int i = 0; i < 1000; i++) {
                this->get(acceleration_, twist_, attitude_);
                std::this_thread::sleep_for(std::chrono::microseconds(900));
                t_sum += twist_;
            }
            twist_bias = t_sum / 1000.0;
        }

        void get(Eigen::Vector3f &acceleration_, Eigen::Vector3f &twist_, Eigen::Quaternionf &attitude_){
            _imu_mutex.lock();
            acceleration_ = acceleration;
            twist_ = twist;
            attitude_ = attitude;
            _imu_mutex.unlock();
        }

    private:
        data_filter::CompAngularRate raw_gyro;
        data_filter::CompAccel raw_accel;
        data_sensor::CompQuaternion raw_attitude;
        data_filter::GravityVector g;
        uint16_t sensor_sample_rate;
        uint16_t filter_sample_rate;
        std::unique_ptr<Utils> utils;
        Eigen::Vector3f acceleration;
        Eigen::Vector3f twist;
        Eigen::Quaternionf attitude;
        Eigen::Vector3f twist_bias;
        std::mutex _imu_mutex;
};

#endif
#endif