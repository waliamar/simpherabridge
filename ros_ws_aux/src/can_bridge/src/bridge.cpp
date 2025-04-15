#include "bridge.h"

#include "can_bridge/canid_enum.hpp"
#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>
#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <VESIResultData.h>
#include <GeographicLib/UTMUPS.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using builtin_interfaces::msg::Time;
using sensor_msgs::msg::Imu;

namespace bridge {

  struct DbcFrameRx {
    NewEagle::DbcMessage* message = NULL;

    DbcFrameRx(const can_msgs::msg::Frame& msg, const MessageID& msgType, NewEagle::Dbc& dbc,
              Time& frame_timestamp) {
      message = dbc.GetMessageById(static_cast<uint32_t>(msgType));
      if (msg.dlc < message->GetDlc()) return;
      frame_timestamp = msg.header.stamp;
      message->SetFrame(std::make_shared<can_msgs::msg::Frame>(msg));
    }

    bool valid() const { return message != NULL; }

    template <typename T>
    DbcFrameRx& operator()(const std::string& dbcField, T& msgField) {
      msgField = static_cast<T>(message->GetSignal(dbcField)->GetResult());
      return *this;
    }
  };

  struct DbcFrameTx {
    NewEagle::DbcMessage* message = NULL;

    DbcFrameTx(const MessageID& msgType, NewEagle::Dbc& dbc) {
      message = dbc.GetMessageById(static_cast<uint32_t>(msgType));
        if (!message) {
            std::cerr << "Error: Could not find message ID " << static_cast<uint32_t>(msgType)
                      << " in DBC file." << std::endl;
        }
    }

    bool valid() const { return message != NULL; }

    template <typename T>
    DbcFrameTx& operator()(const std::string& dbcField, const T& msgField) {
      message->GetSignal(dbcField)->SetResult(static_cast<double>(msgField));
      return *this;
    }
  };

  uint8_t rolling_counter_ = 0;  
  void incrementRollingCounter() {
      // Increment and wrap around at 15 (for a 4-bit counter)
      rolling_counter_ = (rolling_counter_ + 1) % 16;
  }


  CanBridgeNode::CanBridgeNode() : Node("can_bridge_node")
  {
    // RCLCPP_INFO(this->get_logger(), "Hello World");

    std::string package_share_directory;
    try {
        package_share_directory = ament_index_cpp::get_package_share_directory("can_bridge");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find package share directory: %s", e.what());
        throw e;
    }
    dbw_dbc_file_ = package_share_directory + "/config/CAN1-INDY-V17.dbc";

    try {
        // Load the DBC file using DbcBuilder
        dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);

        // Verify DBC loading
        if (dbwDbc_.GetMessageCount() == 0) {
            RCLCPP_ERROR(this->get_logger(), 
                         "DBC file loaded, but no messages found: %s", dbw_dbc_file_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), 
                        "Successfully loaded DBC file: %s", dbw_dbc_file_.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load DBC file: %s. Error: %s", 
                     dbw_dbc_file_.c_str(), e.what());
        throw e;
    }
    
    try
    {
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_iac);
      const auto sim_qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sim_clock);

      this->toRaptorPublisher_ = this->create_publisher<autonoma_msgs::msg::ToRaptor>("to_raptor", qos);
      this->vehicleInputsPublisher_ = this->create_publisher<autonoma_msgs::msg::VehicleInputs>("vehicle_inputs", qos);

      this->canPublisher_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 20); 
      this->canSubscriber_ = this->create_subscription<can_msgs::msg::Frame>("to_can_bus", qos, std::bind(&CanBridgeNode::canSubscriberCallback, this, _1)); 
      if (this->canPublisher_ == nullptr || this->canSubscriber_ == nullptr) {
        std::cerr << "Failed to create CAN publisher or subscriber!" << std::endl;
      }
      
      this->receiveRaceControl_ = this->create_subscription<autonoma_msgs::msg::RaceControl>("race_control", qos, std::bind(&CanBridgeNode::subscribeRaceControlCallback, this, _1));
      this->receiveVehicleData_ = this->create_subscription<autonoma_msgs::msg::VehicleData>("vehicle_data", qos, std::bind(&CanBridgeNode::subscribeVehicleDataCallback, this, _1));
      this->receivePowertrainData_ = this->create_subscription<autonoma_msgs::msg::PowertrainData>("powertrain_data", qos, std::bind(&CanBridgeNode::subscribePowertrainDataCallback, this, _1));
      
      timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CanBridgeNode::timerCallback, this));  

      // if(this->simModeEnabled)
      // {
      //   std::cout << "Use Simulated Clock." << '\n';
      //   this->simClockTimePublisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", sim_qos);      
      //   this->simTimeIncrease_ = this->create_subscription<std_msgs::msg::UInt16>("sim_time_increase", sim_qos, std::bind(&CanBridgeNode::simTimeIncreaseCallback, this, _1));
      //   // vesiCallback();
      //   this->simClockTime.clock = rclcpp::Time(this->sec, this->nsec);
      //   this->simClockTimePublisher_->publish(this->simClockTime);
      // }
      // else
      // {
      //   std::cout << "Use Wall Clock (system clock)." << '\n';
      //   this->updateVESIVehicleInputs_ = this->create_wall_timer(10ms, std::bind(&CanBridgeNode::vesiCallback, this));
      // }
    }
    catch(const std::exception& e)
    {
      std::cerr << "Failed to create object for Can Bridge node: " << e.what() << '\n';
    }

    // if (this->enableTimeRecord)
    // {
    //   this->myfile.open(std::string(this->pathTimeRecord) + "/duration_recording.csv");
    //   this->myfile << "sendVehicleFeedbackToSimulation" << ","
    //                << "requestCustomData" << ","
    //                << "castCanbus_raw" << ","
    //                << "publishSimulationState" << ","
    //                << "VESICallBackInterval" << "\n";
    //   this->myfile.close();
    //   std::cout << "Log created under path: " << std::string(this->pathTimeRecord) << std::endl;
    // }

    std::cout << "Setup done." << '\n';
  }

  void CanBridgeNode::timerCallback() {
    // RCLCPP_INFO(this->get_logger(), "Publishing Stack State");
    publishVehicleInputs();
    publishToRaptor();
    // simClockTimeCallback();
  }

  // void CanBridgeNode::simClockTimeCallback()
  // {
  //   // Configure simulated clock
  //   simClockTime.clock = rclcpp::Time(this->sec,this->nsec);
  //   this->simClockTimePublisher_->publish(simClockTime);
  // }

  void CanBridgeNode::subscribeRaceControlCallback(const autonoma_msgs::msg::RaceControl & msg)
  {
    if (this->verbosePrinting)
    {
      std::cout << "subscribeRaceControlCallback" << '\n';
    }

    // race_control_base_to_car_heartbeat = msg.base_to_car_heartbeat;
    // race_control_veh_flag = msg.veh_flag;
    // race_control_veh_rank = msg.veh_rank;
    // race_control_track_flag = msg.track_flag;
    // race_control_sys_state = msg.sys_state;
    // race_control_lap_count = msg.lap_count;
    // race_control_lap_distance = msg.lap_distance;
    // race_control_round_target_speed = msg.round_target_speed;
    // race_control_laps = msg.laps;
    // race_control_lap_time = msg.lap_time;
    // race_control_time_stamp = msg.time_stamp;

   //PUBLISH BASE_TO_CAR_TIMING
    DbcFrameTx dbcFrame = DbcFrameTx{MessageID::BASE_TO_CAR_TIMING, this->dbwDbc_}; 
    if (this->dbwDbc_.GetMessageCount() == 0) {
        std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    }
    dbcFrame("laps", msg.laps )
            ("lap_time", msg.lap_time)
            ("time_stamp", msg.time_stamp);
    auto frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame()); 
    if(this->simModeEnabled)
    {
      frame->header.stamp.sec = this->sec;
      frame->header.stamp.nanosec = this->nsec;
    }
    else 
    {
      frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    this->canPublisher_->publish(std::move(frame)); 

    //PUBLISH BASE_TO_CAR_SUMMARY (mylaps)
    dbcFrame = DbcFrameTx{MessageID::BASE_TO_CAR_SUMMARY, this->dbwDbc_}; 
    if (this->dbwDbc_.GetMessageCount() == 0) {
        std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    }
    dbcFrame("base_to_car_heartbeat", msg.base_to_car_heartbeat)
            ("veh_rank", msg.veh_rank)
            ("lap_count", msg.lap_count)
            ("lap_distance", msg.lap_distance)
            ("round_target_speed", msg.round_target_speed)
            ("track_flag", msg.track_flag) //dspace uses mylaps by default
            ("veh_flag", msg.veh_flag);
    frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame());
    if(this->simModeEnabled)
    {
      frame->header.stamp.sec = this->sec;
      frame->header.stamp.nanosec = this->nsec;
    }
    else 
    {
      frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    this->canPublisher_->publish(std::move(frame)); 

    //PUBLISH MISC_REPORT
    dbcFrame = DbcFrameTx{MessageID::MISC_REPORT, this->dbwDbc_}; 
    if (this->dbwDbc_.GetMessageCount() == 0) {
        std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    }
    dbcFrame("battery_voltage", vehicle_data_battery_voltage) //currently hard-coded val
            ("safety_switch_state", vehicle_data_safety_switch_state)  //currently hard-coded val
            ("mode_switch_state", vehicle_data_mode_switch_state) //false: race mode, true: test mode  (currently hard-coded val)
            ("sys_state", msg.sys_state);
            // ("raptor_rolling_counter", rolling_counter_); //currently unused
    frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame());
    if(this->simModeEnabled)
    {
      frame->header.stamp.sec = this->sec;
      frame->header.stamp.nanosec = this->nsec;
    }
    else 
    {
      frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    this->canPublisher_->publish(std::move(frame));

    // //PUBLISH MARELLI_REPORT_1 (UNCOMMENT IF USINg MARELLI)
    // dbcFrame = DbcFrameTx{MessageID::MARELLI_REPORT_1, this->dbwDbc_}; 
    // if (this->dbwDbc_.GetMessageCount() == 0) {
    //     std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    // }
    // dbcFrame("marelli_track_flag", race_control_track_flag)
    //         ("marelli_vehicle_flag", race_control_veh_flag);
    //         // ("marelli_sector_flag", race_control_sector_flag); //not used
    // frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame());
    // if(this->simModeEnabled)
    // {
    //   frame->header.stamp.sec = this->sec;
    //   frame->header.stamp.nanosec = this->nsec;
    // }
    // else //this is triggered by default
    // {
    //   frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    //   frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    // }

    // this->canPublisher_->publish(std::move(frame)); 
      
  }

  void CanBridgeNode::subscribeVehicleDataCallback(const autonoma_msgs::msg::VehicleData & msg)
  {
    if (this->verbosePrinting)
    {
      std::cout << "subscribeVehicleDataCallback" << '\n';
    }

    //PUBLISH ACCELERATOR_REPORT
    DbcFrameTx dbcFrame = DbcFrameTx{MessageID::ACCELERATOR_REPORT, this->dbwDbc_}; //overwrite dbcFrame
    if (this->dbwDbc_.GetMessageCount() == 0) {
        std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    }
    dbcFrame("acc_pedal_fdbk", msg.accel_pedal_output)
            ("acc_pedal_fdbk_counter", rolling_counter_);
    auto frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame()); //overwrite can frame msg
    if(this->simModeEnabled)
    {
      frame->header.stamp.sec = this->sec;
      frame->header.stamp.nanosec = this->nsec;
    }
    else
    {
      frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    this->canPublisher_->publish(std::move(frame)); 

    //PUBLISH STEERING_REPORT_EXTD
    dbcFrame = DbcFrameTx{MessageID::STEERING_REPORT_EXTD, this->dbwDbc_}; 
    if (this->dbwDbc_.GetMessageCount() == 0) {
        std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    }
    dbcFrame("average_steering_ang_fdbk",msg.steering_wheel_angle) // Angles in degrees
          ("secondary_steering_ang_fdbk", msg.steering_wheel_angle)
          ("primary_steering_angle_fbk", msg.steering_wheel_angle);
    frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame()); 
    if(this->simModeEnabled)
    {
      frame->header.stamp.sec = this->sec;
      frame->header.stamp.nanosec = this->nsec;
    }
    else 
    {
      frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    this->canPublisher_->publish(std::move(frame)); 

    // PUBLISH WHEEL SPEED REPORT
    dbcFrame = DbcFrameTx{MessageID::WHEEL_SPEED_REPORT, this->dbwDbc_}; 
    if (this->dbwDbc_.GetMessageCount() == 0) {
        std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    }
    dbcFrame("wheel_speed_RL", msg.ws_rear_left) // Angles in degrees
          ("wheel_speed_FR", msg.ws_front_right)
          ("wheel_speed_FL", msg.ws_front_left)
          ("wheel_speed_RR", msg.ws_rear_right);
    frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame()); 
    if(this->simModeEnabled)
    {
      frame->header.stamp.sec = this->sec;
      frame->header.stamp.nanosec = this->nsec;
    }
    else 
    {
      frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    this->canPublisher_->publish(std::move(frame)); 

    // PUBLISH BRAKE PRESSURE REPORT
    dbcFrame = DbcFrameTx{MessageID::BRAKE_PRESSURE_REPORT, this->dbwDbc_}; 
    if (this->dbwDbc_.GetMessageCount() == 0) {
        std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    }
    dbcFrame("brake_pressure_fdbk_front", msg.front_brake_pressure)
            ("brake_pressure_fdbk_rear", msg.rear_brake_pressure)
            ("brk_pressure_fdbk_counter", rolling_counter_);
    frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame());
    if(this->simModeEnabled)
    {
      frame->header.stamp.sec = this->sec;
      frame->header.stamp.nanosec = this->nsec;
    }
    else
    {
      frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    this->canPublisher_->publish(std::move(frame)); 

    incrementRollingCounter(); //4 bit counter
  }

  void CanBridgeNode::subscribePowertrainDataCallback(const autonoma_msgs::msg::PowertrainData & msg)
  {
    if (this->verbosePrinting)
    {
      std::cout << "subscribePowertrainDataCallback" << '\n';
    }

    //PUBLISH PT_REPORT_1
    DbcFrameTx dbcFrame = DbcFrameTx{MessageID::PT_REPORT_1, this->dbwDbc_}; 
    if (this->dbwDbc_.GetMessageCount() == 0) {
        std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    }
    dbcFrame("throttle_position", msg.throttle_position)
            ("engine_run_switch", msg.engine_run_switch_status)
            ("current_gear", msg.current_gear)
            ("engine_speed_rpm", msg.engine_rpm)
            ("vehicle_speed_kmph", msg.vehicle_speed_kmph)
            ("engine_state", msg.engine_on_status)
            ("gear_shift_status", msg.gear_shift_status);
    auto frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame());  
    if(this->simModeEnabled)
    {
      frame->header.stamp.sec = this->sec;
      frame->header.stamp.nanosec = this->nsec;
    }
    else
    {
      frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    this->canPublisher_->publish(std::move(frame)); 

    //PUBLISH PT_REPORT_2
    dbcFrame = DbcFrameTx{MessageID::PT_REPORT_2, this->dbwDbc_}; 
    if (this->dbwDbc_.GetMessageCount() == 0) {
        std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    }
    dbcFrame("fuel_pressure_kPa", msg.fuel_pressure)
            ("engine_oil_pressure_kPa", msg.engine_oil_pressure)
            ("coolant_temperature", msg.engine_coolant_temperature)
            ("transmission_temperature", msg.transmission_oil_temperature)
            ("transmission_pressure_kPa", msg.transmission_oil_pressure);
    frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame());
    if(this->simModeEnabled)
    {
      frame->header.stamp.sec = this->sec;
      frame->header.stamp.nanosec = this->nsec;
    }
    else
    {
      frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    this->canPublisher_->publish(std::move(frame)); 

    //PUBLISH PT_REPORT_3
    dbcFrame = DbcFrameTx{MessageID::PT_REPORT_3, this->dbwDbc_}; 
    if (this->dbwDbc_.GetMessageCount() == 0) {
        std::cerr << "DBC file is not loaded or contains no messages." << std::endl;
    }
    dbcFrame("engine_oil_temperature", msg.engine_oil_temperature)
            ("torque_wheels", msg.torque_wheels_nm);
            // ("driver_traction_aim_swicth_fbk", powertrainData.driver_traction_aim_switch)
            // ("driver_traction_range_switch_fbk", powertrainData.driver_traction_range_switch);
    frame = std::make_unique<can_msgs::msg::Frame>(dbcFrame.message->GetFrame());
    if(this->simModeEnabled)
    {
      frame->header.stamp.sec = this->sec;
      frame->header.stamp.nanosec = this->nsec;
    }
    else
    {
      frame->header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      frame->header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    this->canPublisher_->publish(std::move(frame)); 

  }

  void CanBridgeNode::publishVehicleInputs()
  {
    if (this->verbosePrinting)
    {
      std::cout << "publishVehicleInputs" << '\n';
    }

    auto vehicleInputsMsg = autonoma_msgs::msg::VehicleInputs();

    vehicleInputsMsg.throttle_cmd = vehicle_inputs_throttle_cmd;
    vehicleInputsMsg.throttle_cmd_count = vehicle_inputs_throttle_cmd_count;
    vehicleInputsMsg.brake_cmd = vehicle_inputs_brake_cmd;
    vehicleInputsMsg.brake_cmd_count = vehicle_inputs_brake_cmd_count;
    vehicleInputsMsg.steering_cmd = vehicle_inputs_steering_cmd;
    vehicleInputsMsg.steering_cmd_count = vehicle_inputs_steering_cmd_count;
    vehicleInputsMsg.gear_cmd = vehicle_inputs_gear_cmd;

    // RCLCPP_INFO(this->get_logger(), 
    //           "Publishing accelerator cmd To Vehicle_Inputs:  %f", vehicle_inputs_throttle_cmd);
    // Header
    vehicleInputsMsg.header.frame_id = "";

    if(this->simModeEnabled)
    {
      vehicleInputsMsg.header.stamp.sec = this->sec;
      vehicleInputsMsg.header.stamp.nanosec = this->nsec;
    }
    else
    {
      vehicleInputsMsg.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      vehicleInputsMsg.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (vehicleInputsMsg.header.stamp.sec*1000000000);
    }

    this->vehicleInputsPublisher_->publish(vehicleInputsMsg); 
    // if (this->useCustomRaceControl == false)
    // {
    //   this->vehicleInputsPublisher_->publish(vehicleInputsMsg);  
    // }
  }

  void CanBridgeNode::publishToRaptor()
  {
    if (this->verbosePrinting)
    {
      std::cout << "publishToRaptor" << '\n';
    }

    auto toRaptorMsg = autonoma_msgs::msg::ToRaptor();

    toRaptorMsg.track_cond_ack = to_raptor_track_cond_ack;
    toRaptorMsg.veh_sig_ack = to_raptor_veh_sig_ack;
    toRaptorMsg.ct_state = to_raptor_ct_state;
    toRaptorMsg.rolling_counter = to_raptor_rolling_counter;
    toRaptorMsg.veh_num = to_raptor_veh_num;
    
    // Header
    toRaptorMsg.header.frame_id = "";

    if(this->simModeEnabled)
    {
      toRaptorMsg.header.stamp.sec = this->sec;
      toRaptorMsg.header.stamp.nanosec = this->nsec;
    }
    else
    {
      toRaptorMsg.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      toRaptorMsg.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (toRaptorMsg.header.stamp.sec*1000000000);
    }

    //check track flag is valid
    if(to_raptor_track_cond_ack == 1 || to_raptor_track_cond_ack == 2 || to_raptor_track_cond_ack == 3 || to_raptor_track_cond_ack == 1){ 
      this->toRaptorPublisher_->publish(toRaptorMsg); 
    }
    else{
      std::cerr << "incorrect track flag, will not publish to_raptor: " << static_cast<int>(to_raptor_track_cond_ack) << std::endl;
    }
  }

  void CanBridgeNode::canSubscriberCallback(can_msgs::msg::Frame::UniquePtr msg) 
  {
    static constexpr std::string_view fl = "FL";
    static constexpr std::string_view fr = "FR";
    static constexpr std::string_view rl = "RL";
    static constexpr std::string_view rr = "RR";
    static constexpr uint8_t part1 = 1;
    static constexpr uint8_t part2 = 2;
    static constexpr uint8_t part3 = 3;
    static constexpr uint8_t part4 = 4;

    if (msg->is_rtr || msg->is_error) return;

    switch (static_cast<MessageID>(msg->id)) {
      case MessageID::BRAKE_PRESSURE_CMD:
        recvBrakePressureCmd(*msg);
        break;

      case MessageID::ACCELERATOR_CMD:
        recvAcceleratorCmd(*msg);
        break;

      case MessageID::STEERING_CMD:
        recvSteeringCmd(*msg);
        break;

      case MessageID::GEAR_SHIFT_CMD:
        recvGearShiftCmd(*msg);
        break;

      case MessageID::CT_REPORT:
        recvCtReport(*msg);
        break;

      // case MessageID::CT_REPORT_2:
      //   recvCtReport2(*msg);
      //   break;

      default:
        break;
    }
  } // end can subscriber callback

  void CanBridgeNode::recvBrakePressureCmd(const can_msgs::msg::Frame& msg) {
    Time stamp;
    DbcFrameRx dbcFrame{msg, MessageID::BRAKE_PRESSURE_CMD, dbwDbc_, stamp};
    if (!dbcFrame.valid()){
    std::cout << "DBC file LOAD FAILED" << '\n'; 
    return;
    }
    dbcFrame("brake_pressure_cmd", vehicle_inputs_brake_cmd)
            ("brk_pressure_cmd_counter", vehicle_inputs_brake_cmd_count);
  }

  void CanBridgeNode::recvAcceleratorCmd(const can_msgs::msg::Frame& msg) {
    Time stamp;
    DbcFrameRx dbcFrame{msg, MessageID::ACCELERATOR_CMD, dbwDbc_, stamp};
    if (!dbcFrame.valid()){
    std::cout << "DBC file LOAD FAILED" << '\n'; 
    return;
    } 
    dbcFrame("acc_pedal_cmd", vehicle_inputs_throttle_cmd)
            ("acc_pedal_cmd_counter", vehicle_inputs_throttle_cmd_count);

    // RCLCPP_INFO(this->get_logger(), 
    //             "Received accelerator cmd thru CAN:  %f", vehicle_inputs_throttle_cmd);
  }

  void CanBridgeNode::recvSteeringCmd(const can_msgs::msg::Frame& msg) {
    Time stamp;
    DbcFrameRx dbcFrame{msg, MessageID::STEERING_CMD, dbwDbc_, stamp};
    if (!dbcFrame.valid()){
    std::cout << "DBC file LOAD FAILED" << '\n'; 
    return;
    } 
    dbcFrame("steering_motor_ang_cmd", vehicle_inputs_steering_cmd)
            ("steering_motor_cmd_counter", vehicle_inputs_steering_cmd_count);
  }

  void CanBridgeNode::recvGearShiftCmd(const can_msgs::msg::Frame& msg) {
    Time stamp;
    DbcFrameRx dbcFrame{msg, MessageID::GEAR_SHIFT_CMD, dbwDbc_, stamp};
    if (!dbcFrame.valid()){
    std::cout << "DBC file LOAD FAILED" << '\n'; 
    return;
    } 
    dbcFrame("desired_gear", vehicle_inputs_gear_cmd);
  }

  void CanBridgeNode::recvCtReport(const can_msgs::msg::Frame& msg) {
    Time stamp;
    DbcFrameRx dbcFrame{msg, MessageID::CT_REPORT, dbwDbc_, stamp};
    if (!dbcFrame.valid()){
    std::cout << "DBC file LOAD FAILED" << '\n'; 
    return;
    } 
    dbcFrame("veh_num", to_raptor_veh_num )
            ("ct_state", to_raptor_ct_state) 
            ("ct_state_rolling_counter", to_raptor_rolling_counter)
            ("track_cond_ack", to_raptor_track_cond_ack) //Dspace uses mylaps by default
            ("veh_sig_ack", to_raptor_veh_sig_ack);
    prev_veh_num = to_raptor_veh_num;
    prev_ct_state = to_raptor_ct_state;
  }

  // //UNCOMMENT IF UsING MArELLI
  // void CanBridgeNode::recvCtReport2(const can_msgs::msg::Frame& msg) {
  //   Time stamp;
  //   DbcFrameRx dbcFrame{msg, MessageID::CT_REPORT_2, dbwDbc_, stamp};
  //   if (!dbcFrame.valid()){
  //   std::cout << "DBC file LOAD FAILED" << '\n'; 
  //   return;
  //   } 
  //   dbcFrame("marelli_track_flag_ack", to_raptor_track_cond_ack) 
  //           ("marelli_vehicle_flag_ack", to_raptor_veh_sig_ack);
  //           // ("marelli_sector_flag_ack", race_control_cmd_.sector_flag); //not used
  //   prev_track_cond_ack = to_raptor_track_cond_ack;
  //   prev_veh_sig_ack = to_raptor_veh_sig_ack;
  // }

}

int main(int argc, char * argv[])
{
  try
  {
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr CanBridgeNodePtr = std::make_shared<bridge::CanBridgeNode>();
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(CanBridgeNodePtr);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
  catch(const std::exception& e)
  {
    std::cerr << "Failed to initialize Can Bridge node: " << e.what() << '\n';
  }
  
}
