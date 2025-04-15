#include <chrono>
#include <string>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <vector>
#include <fstream>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/transform_stamped.hpp"


#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/u_int16.hpp>
#include "rosgraph_msgs/msg/clock.hpp"

#include "autonoma_msgs/msg/ground_truth_array.hpp"
#include "autonoma_msgs/msg/powertrain_data.hpp"
#include "autonoma_msgs/msg/race_control.hpp"
#include "autonoma_msgs/msg/to_raptor.hpp"
#include "autonoma_msgs/msg/vehicle_data.hpp"
#include "autonoma_msgs/msg/vehicle_inputs.hpp"

#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"

#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"

#include "foxglove_msgs/msg/scene_update.hpp"

#include "iac_qos.h"

// #include "VESIAPI.h"
// #include "VESIResultData.h"

#include "ASMBus_renamed.h"
#include "RaceControlInterface.h"

#include "nav_msgs/msg/odometry.hpp"
#include "can_bridge/canid_enum.hpp"

#include <can_msgs/msg/frame.hpp>

#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>
#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/UTMUPS.hpp>

namespace bridge
{   
    class CanBridgeNode : public rclcpp::Node
    {

    public:
        CanBridgeNode();
        ~CanBridgeNode() = default;

    private:
       // Publishers
        rclcpp::Publisher<autonoma_msgs::msg::ToRaptor>::SharedPtr toRaptorPublisher_;
        rclcpp::Publisher<autonoma_msgs::msg::VehicleInputs>::SharedPtr vehicleInputsPublisher_;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelImuDataRawPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelImuPublisher;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr novaTelOdomPublisher;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr novaTelFixPublisher;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelImuDataRawPublisher1_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelImuPublisher1_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr novaTelOdomPublisher1_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr novaTelFixPublisher1_;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelImuDataRawPublisher2_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelImuPublisher2_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr novaTelOdomPublisher2_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr novaTelFixPublisher2_;

        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr canPublisher_;

        // Subscibers
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr canSubscriber_;

        rclcpp::Subscription<autonoma_msgs::msg::VehicleInputs>::SharedPtr receiveVehicleCommands_;
        rclcpp::Subscription<autonoma_msgs::msg::RaceControl>::SharedPtr receiveRaceControl_;
        rclcpp::Subscription<autonoma_msgs::msg::VehicleData>::SharedPtr receiveVehicleData_;
        rclcpp::Subscription<autonoma_msgs::msg::PowertrainData>::SharedPtr receivePowertrainData_;

        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr simClockTimePublisher_;

        // Parameter
        bool maneuverStarted = false;
        bool vesiDataAvailabe = false;
        bool feedbackDataAvailabe = false;
        bool useCustomRaceControl = false;
        bool verbosePrinting = false;
        bool simModeEnabled = false;
        bool numberWarningSent = false;
        uint8_t prestart_rolling_counter;


        // Execution duration logging
        std::vector <double> measured_vesi_times;
        int64_t timeStartNanosec = 0;
        int64_t timeEndNanosec = 0;
        int64_t timeStartVESICallBackNanosec = 0;
        int64_t timeEndVESICallBackNanosec = 0;
        std::ofstream myfile;
        std::string pathTimeRecord;
        bool enableTimeRecord;

        // Simulated clock
        uint32_t nsec = 0;
        uint32_t sec = 0;
        uint64_t simTotalMsec = 0;
        rosgraph_msgs::msg::Clock simClockTime;
        rclcpp::TimerBase::SharedPtr updateSimClock_;

        // Callbacks
        void subscribeRaceControlCallback(const autonoma_msgs::msg::RaceControl &msg);
        void subscribeVehicleDataCallback(const autonoma_msgs::msg::VehicleData &msg);
        void subscribePowertrainDataCallback(const autonoma_msgs::msg::PowertrainData &msg);
        void simClockTimeCallback();

        void subscribeBestPosCallback1(const novatel_oem7_msgs::msg::BESTPOS &msg);
        void subscribeBestPosCallback2(const novatel_oem7_msgs::msg::BESTPOS &msg);
        void subscribeRawImuXCallback1(const sensor_msgs::msg::Imu &msg);
        void subscribeRawImuXCallback2(const sensor_msgs::msg::Imu &msg);
        void subscribeHeadingCallback1(const novatel_oem7_msgs::msg::HEADING2 &msg);
        void subscribeHeadingCallback2(const novatel_oem7_msgs::msg::HEADING2 &msg);
        void subscribeInspvaCallback1(const novatel_oem7_msgs::msg::INSPVA &msg);
        void subscribeInspvaCallback2(const novatel_oem7_msgs::msg::INSPVA &msg);

        // Publishing functions

        void publishVehicleInputs();
        void publishToRaptor();

        void canSubscriberCallback(can_msgs::msg::Frame::UniquePtr msg);

        // //Receiving CAN frames
        void recvBrakePressureCmd(const can_msgs::msg::Frame& msg);
        void recvAcceleratorCmd(const can_msgs::msg::Frame& msg);
        void recvSteeringCmd(const can_msgs::msg::Frame& msg);
        void recvGearShiftCmd(const can_msgs::msg::Frame& msg);
        void recvCtReport(const can_msgs::msg::Frame& msg);
        void recvCtReport2(const can_msgs::msg::Frame& msg);

        std::string dbw_dbc_file_;
        NewEagle::Dbc dbwDbc_;
        
        uint8_t prev_veh_num = 99;
        uint8_t prev_ct_state = 99;
        uint8_t prev_track_cond_ack = 99;
        uint8_t prev_veh_sig_ack = 99;
        rclcpp::TimerBase::SharedPtr timer_;
        void timerCallback();

        uint8_t to_raptor_veh_num = 1;
        uint8_t to_raptor_ct_state = 255;
        uint8_t to_raptor_rolling_counter;
        uint8_t to_raptor_track_cond_ack = 1;
        uint8_t to_raptor_veh_sig_ack = 1;

        uint8_t vehicle_inputs_gear_cmd;
        float vehicle_inputs_steering_cmd;
        uint8_t vehicle_inputs_steering_cmd_count;
        float vehicle_inputs_throttle_cmd;
        uint8_t vehicle_inputs_throttle_cmd_count;
        float vehicle_inputs_brake_cmd;
        uint8_t vehicle_inputs_brake_cmd_count;

        uint8_t race_control_base_to_car_heartbeat;
        uint8_t race_control_track_flag;
        uint8_t race_control_veh_flag;
        uint8_t race_control_veh_rank;
        uint8_t race_control_sys_state;
        uint8_t race_control_lap_count;
        float race_control_lap_distance;
        uint8_t race_control_round_target_speed;
        uint8_t race_control_laps;
        float race_control_lap_time;
        float race_control_time_stamp;

        float top_heading2_heading = 999.0;
        float bottom_heading2_heading = 999.0;
        float top_inspva_azimuth = 999.0;
        float bottom_inspva_azimuth = 999.0;

// # Misc data
    float vehicle_data_battery_voltage = 14.0;
    int vehicle_data_safety_switch_state = 4;
    bool vehicle_data_mode_switch_state = false; //true for test mode

    };
}