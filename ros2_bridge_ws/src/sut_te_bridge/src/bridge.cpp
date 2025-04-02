#include "bridge.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace bridge {

  SutTeBridgeNode::SutTeBridgeNode() : Node("sut_te_bridge_node")
  {
    std::cout << "Set SimManager Host IP to: "<< std::getenv("VESI_IP") << std::endl;
    if (std::getenv("VESI_IP")){
      this->api.setSimManagerHost(std::getenv("VESI_IP"));
    } else {
      this->api.setSimManagerHost("127.0.0.1");
    }
    std::cout << "Set ASM Host IP to: " << std::getenv("ASM_IP") << std::endl;
    if (std::getenv("ASM_IP")){
      this->api.setASMHost(std::getenv("ASM_IP"));
    } else {
      this->api.setASMHost("127.0.0.1");
    }
    std::cout << "Set Publish interval" << std::endl;
    if (std::getenv("PUB_ITV_RACE_CONTROL_DATA")){
      this->pubIntervalRaceControlData = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_RACE_CONTROL_DATA"))));
    } else {
      this->pubIntervalRaceControlData = 10;
    }
    if (std::getenv("PUB_ITV_VEHICLE_DATA")){
      this->pubIntervalVehicleData = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_VEHICLE_DATA"))));
    } else {
      this->pubIntervalVehicleData = 10;
    }
    if (std::getenv("PUB_ITV_POWER_TRAIN_DATA")){
      this->pubIntervalPowertrainData = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_POWER_TRAIN_DATA"))));
    } else {
      this->pubIntervalPowertrainData = 10;
    }
    if (std::getenv("PUB_ITV_GROUND_TRUTH_ARRAY")){
      this->pubIntervalGroundTruthArray = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_GROUND_TRUTH_ARRAY"))));
    } else {
      this->pubIntervalGroundTruthArray = 10;
    }
    if (std::getenv("PUB_ITV_VECTOR_NAV_DATA")){
      this->pubIntervalVectorNavData = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_VECTOR_NAV_DATA"))));
    } else {
      this->pubIntervalVectorNavData = 10;
    }
    if (std::getenv("PUB_ITV_NOVATEL_DATA")){
      this->pubIntervalNovatelData = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_NOVATEL_DATA"))));
    } else {
      this->pubIntervalNovatelData = 10;
    }
    if (std::getenv("PUB_ITV_FOXGLOVE_MAP")){
      this->pubIntervalFoxgloveMap = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_FOXGLOVE_MAP"))));
    } else {
      this->pubIntervalFoxgloveMap = 10;
    }

    if (std::getenv("PATH_LOG")){
      this->pathTimeRecord = std::string(std::getenv("PATH_LOG"));
    } else {
      this->pathTimeRecord = "/root/record_log";
    }
    if (std::getenv("ENABLE_LOG")){
      if (std::string(std::getenv("ENABLE_LOG")) == "true"){
        this->enableTimeRecord = true;
      } else {
        this->enableTimeRecord = false;
      }
    } else {
      this->enableTimeRecord = false;
    }

    std::cout << "Set SimManager Host Port to: 12345"<< std::endl;
    this->api.setSimManagerPort(12345);
    std::cout << "Set Custom Data required to: true"<< std::endl;
    this->api.setCustomDataRequired(true);

    std::cout << "Trying to connect to V-ESI at: " << std::getenv("VESI_IP") << " : 12345" << std::endl;
    std::cout << "Trying to connect to ASM at: " << std::getenv("ASM_IP") << " with CustomDataInterface set to: true" << std::endl;
    bool vesiConnection = false;
    int16_t retries = 1;
    int16_t max_retries = 1;

    if (std::getenv("VERBOSE_PRINT") && (std::string(std::getenv("VERBOSE_PRINT")) == "true"))
    {
      this->verbosePrinting = true;
    }

    if (std::getenv("SIM_CLOCK_MODE") && (std::string(std::getenv("SIM_CLOCK_MODE")) == "true"))
    {
        this->simModeEnabled = true;
    }

    while (vesiConnection == false)
    {
      try
      {
        std::list<uint16_t> providedControlDataIDs{22222};
        api.setProvidedControlDataIDs(providedControlDataIDs, true);

        this->api.connect();
        std::cout << "V-ESI connection configured" << std::endl;
        vesiConnection = true;
      }
      catch(const std::exception& e)
      {
        std::cerr << "Failed to configure V-ESI: " << e.what() << '\n';
        if (retries < max_retries)
        {
          std::cout << "Failed for the " << retries << " time. Try again." << '\n';
          retries++;
          rclcpp::sleep_for(1s);
        }
        else
        {
          std::cout << "Failed too often. Exit." << '\n';
          throw e;
        }
      }
    }
    
    try
    {
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_iac);
      const auto sim_qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sim_clock);
      this->raceControlDataPublisher_ = this->create_publisher<autonoma_msgs::msg::RaceControl>("race_control", qos);
      this->vehicleDataPublisher_ = this->create_publisher<autonoma_msgs::msg::VehicleData>("vehicle_data", qos);
      this->powertrainDataPublisher_ = this->create_publisher<autonoma_msgs::msg::PowertrainData>("powertrain_data", qos);
      this->groundTruthArrayPublisher_ = this->create_publisher<autonoma_msgs::msg::GroundTruthArray>("ground_truth_array", qos);

      this->verctorNavCommonGroupPublisher_ = this->create_publisher<vectornav_msgs::msg::CommonGroup>("vectornav/raw/common", qos);
      this->verctorNavAttitudeGroupPublisher_ = this->create_publisher<vectornav_msgs::msg::AttitudeGroup>("vectornav/raw/attitude", qos);
      this->verctorNavImuGroupPublisher_ = this->create_publisher<vectornav_msgs::msg::ImuGroup>("vectornav/raw/imu", qos);
      this->verctorNavInsGroupPublisher_ = this->create_publisher<vectornav_msgs::msg::InsGroup>("vectornav/raw/ins", qos);
      this->verctorNavGpsGroupLeftPublisher_ = this->create_publisher<vectornav_msgs::msg::GpsGroup>("vectornav/raw/gps_left", qos);
      this->verctorNavGpsGroupRightPublisher_ = this->create_publisher<vectornav_msgs::msg::GpsGroup>("vectornav/raw/gps_right", qos);
      this->verctorNavTimeGroupPublisher_ = this->create_publisher<vectornav_msgs::msg::TimeGroup>("vectornav/raw/time", qos);

      this->novaTelBestPosPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_top/bestpos", qos);
      this->novaTelBestGNSSPosPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_top/bestgnsspos", qos);
      this->novaTelBestVelPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_top/bestvel", qos);
      this->novaTelBestGNSSVelPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_top/bestgnssvel", qos);
      // this->novaTelInspvaPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::INSPVA>("novatel_top/inspva", qos);
      // this->novaTelHeading2Publisher1_ = this->create_publisher<novatel_oem7_msgs::msg::HEADING2>("novatel_top/heading2", qos);
      this->novaTelRawImuXPublisher1_ = this->create_publisher<sensor_msgs::msg::Imu>("novatel_top/imu/data_raw", qos);
      this->novaTelImuPublisher1_ = this->create_publisher<sensor_msgs::msg::Imu>("novatel_top/imu/data", qos);
      this->novaTelOdomPublisher1_ = this->create_publisher<nav_msgs::msg::Odometry>("novatel_top/odom", qos);
      this->novaTelFixPublisher1_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("novatel_top/fix", qos);


      this->novaTelBestPosPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_bottom/bestpos", qos);
      this->novaTelBestGNSSPosPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_bottom/bestgnsspos", qos);
      this->novaTelBestVelPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_bottom/bestvel", qos);
      this->novaTelBestGNSSVelPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_bottom/bestgnssvel", qos);
      // this->novaTelInspvaPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::INSPVA>("novatel_bottom/inspva", qos);
      // this->novaTelHeading2Publisher2_ = this->create_publisher<novatel_oem7_msgs::msg::HEADING2>("novatel_bottom/heading2", qos);
      this->novaTelRawImuXPublisher2_ = this->create_publisher<sensor_msgs::msg::Imu>("novatel_bottom/imu/data_raw", qos);
      this->novaTelImuPublisher2_ = this->create_publisher<sensor_msgs::msg::Imu>("novatel_bottom/imu/data", qos);
      this->novaTelOdomPublisher2_ = this->create_publisher<nav_msgs::msg::Odometry>("novatel_bottom/odom", qos);
      this->novaTelFixPublisher2_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("novatel_bottom/fix", qos);

      this->foxgloveMapPublisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("foxglove_map", qos);
      this->foxgloveScenePublisher_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>("foxglove_scene", qos);
      this->egoPositionPublisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("ego_position", qos);

      this->resetCommandPublisher_ = this->create_publisher<std_msgs::msg::Bool>("maneuver_reset", qos);

      this->receiveVehicleCommands_ = this->create_subscription<autonoma_msgs::msg::VehicleInputs>("vehicle_inputs", qos, std::bind(&SutTeBridgeNode::subscribeVehicleCommandsCallback, this, _1));
      this->receiveRaptorCommands_ = this->create_subscription<autonoma_msgs::msg::ToRaptor>("to_raptor", qos, std::bind(&SutTeBridgeNode::subscribeRaptorCommandsCallback, this, _1));

      this->useCustomRaceControlSource_ = this->create_subscription<std_msgs::msg::Bool>("use_custom_race_control", qos, std::bind(&SutTeBridgeNode::switchRaceControlSourceCallback, this, _1));
      initializeFeedback();

      if(this->simModeEnabled)
      {
        std::cout << "Use Simulated Clock." << '\n';
        this->simClockTimePublisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", sim_qos);      
        this->simTimeIncrease_ = this->create_subscription<std_msgs::msg::UInt16>("sim_time_increase", sim_qos, std::bind(&SutTeBridgeNode::simTimeIncreaseCallback, this, _1));
        vesiCallback();
        this->simClockTime.clock = rclcpp::Time(this->sec, this->nsec);
        this->simClockTimePublisher_->publish(this->simClockTime);
      }
      else
      {
        std::cout << "Use Wall Clock (system clock)." << '\n';
        this->updateVESIVehicleInputs_ = this->create_wall_timer(10ms, std::bind(&SutTeBridgeNode::vesiCallback, this));
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << "Failed to create object for SUT-TE-Bridge node: " << e.what() << '\n';
    }

    if (this->enableTimeRecord)
    {
      this->myfile.open(std::string(this->pathTimeRecord) + "/duration_recording.csv");
      this->myfile << "sendVehicleFeedbackToSimulation" << ","
                   << "requestCustomData" << ","
                   << "castCanbus_raw" << ","
                   << "publishSimulationState" << ","
                   << "VESICallBackInterval" << "\n";
      this->myfile.close();
      std::cout << "Log created under path: " << std::string(this->pathTimeRecord) << std::endl;
    }

    std::cout << "Setup done." << '\n';
  }

  void SutTeBridgeNode::initializeFeedback()
  {
    if (this->verbosePrinting)
    {
      std::cout << "initializeFeedback" << '\n';
    }

    this->maneuverStarted = false;

    // Throttle command (%)
    this->feedbackCmd.vehicle_inputs.throttle_cmd = 0.0;
    this->feedbackCmd.vehicle_inputs.throttle_cmd_count = 0;

    // # Brake pressure command (kPa)
    this->feedbackCmd.vehicle_inputs.brake_cmd = 0.0;
    this->feedbackCmd.vehicle_inputs.brake_cmd_count = 0;

    // # Steering motor angle command (degrees)
    this->feedbackCmd.vehicle_inputs.steering_cmd = 0.0;
    this->feedbackCmd.vehicle_inputs.steering_cmd_count = 0;

    // # Gear command
    this->feedbackCmd.vehicle_inputs.gear_cmd = 1;
    
    // # Race Control command
    this->feedbackCmd.to_raptor.track_cond_ack = 0;
    this->feedbackCmd.to_raptor.veh_sig_ack = 0;
    this->feedbackCmd.to_raptor.ct_state = 0;
    this->feedbackCmd.to_raptor.rolling_counter = 0;
    this->feedbackCmd.to_raptor.veh_num = 255;
  }

  void SutTeBridgeNode::simClockTimeCallback()
  {
    // Configure simulated clock
    simClockTime.clock = rclcpp::Time(this->sec,this->nsec);
    this->simClockTimePublisher_->publish(simClockTime);
  }

  void SutTeBridgeNode::simTimeIncreaseCallback(const std_msgs::msg::UInt16 & msg)
  {
      for (uint16_t timeIncreaseStep = 0; timeIncreaseStep <  msg.data; timeIncreaseStep++)
      {
        vesiCallback();
      }
      simClockTimeCallback();
  }

  void SutTeBridgeNode::vesiCallback()
  {
    if (this->verbosePrinting)
    {
      std::cout << "vesiCallback" << '\n';
    }

    if (this->enableTimeRecord)
    {
      this->timeEndVESICallBackNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      this->measured_vesi_times.push_back((int64_t(this->timeEndVESICallBackNanosec) - int64_t(this->timeStartVESICallBackNanosec)) / 1000000.0);

      if (this->timeStartNanosec != 0 && this->timeStartVESICallBackNanosec != 0 && this->measured_vesi_times.size() == 5)
      {
        // write measurments to csv log file
        this->myfile.open(std::string(this->pathTimeRecord) + "/duration_recording.csv", std::ios_base::app);
        this->myfile << std::to_string(measured_vesi_times[0]) << ","
                     << std::to_string(measured_vesi_times[1]) << ","
                     << std::to_string(measured_vesi_times[2]) << ","
                     << std::to_string(measured_vesi_times[3]) << ","
                     << std::to_string(measured_vesi_times[4]) << "\n";
        this->myfile.close();
      }

      this->measured_vesi_times.clear();
      this->timeStartVESICallBackNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }

    try
    {
      std::vector<uint8_t> canbus_raw;

      if (this->maneuverStarted == true)
      {
        if (this->enableTimeRecord){
          this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        }
        SutTeBridgeNode::sendVehicleFeedbackToSimulation();
        if (this->enableTimeRecord){
          this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
          this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
        }

        if (this->enableTimeRecord){
          this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        }
        this->api.requestCustomData(&canbus_raw);
        if (this->enableTimeRecord){
          this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
          this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
        }

        if(this->simModeEnabled)
        {
          this->simTotalMsec += 1;
          this->sec = this->simTotalMsec / 1000;
          this->nsec = (this->simTotalMsec % 1000) * 1000000;
        }
        else
        {
          this->simTotalMsec += 10;
        }
      }
      else
      {
        this->api.requestCustomData(&canbus_raw);
      }

      if (this->enableTimeRecord){
        this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      }
      if (canbus_raw.empty()==false)
      {
        this->canBus = reinterpret_cast<ASMBus*>(canbus_raw.data());
        if (this->canBus->asm_bus_var.environment.maneuver.maneuverScheduler.info.maneuverState == 3 && this->maneuverStarted == false)
        {
          this->maneuverStarted = true;
          std::cout << "Maneuver started. Data will be published" << std::endl;
        } else if (this->canBus->asm_bus_var.environment.maneuver.maneuverScheduler.info.maneuverState != 3 && this->maneuverStarted == true)
        {
          std::cout << "Maneuver stopped. System will be reset" << std::endl;
          initializeFeedback();
          std_msgs::msg::Bool resetMsg;
          resetMsg.data = true;
          this->resetCommandPublisher_->publish(resetMsg);
        }
        this->vesiDataAvailabe = true;
      }
      else {std::cout << "No Custom Data available" << std::endl;}
      if (this->enableTimeRecord){
        this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
      }

    }
    catch(const std::exception& e)
    {
      std::cerr << "Failed to request data from ASM: " << e.what() << '\n';
    }

    if (this->enableTimeRecord){
      this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    SutTeBridgeNode::publishSimulationState();
    if (this->enableTimeRecord){
      this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
    }
  }

  void SutTeBridgeNode::publishSimulationState()
  {
    if (this->verbosePrinting)
    {
      std::cout << "publishSimulationState" << '\n';
    }

    try
    {
      if (this->vesiDataAvailabe == true && this->simTotalMsec != 0)
      {
        if(this->simTotalMsec % (this->pubIntervalRaceControlData) == 0)
            SutTeBridgeNode::publishRaceControlData();

        if(this->simTotalMsec % (this->pubIntervalVehicleData) == 0)
            SutTeBridgeNode::publishVehicleData();

        if(this->simTotalMsec % (this->pubIntervalPowertrainData) == 0)
            SutTeBridgeNode::publishPowertrainData();

        if(this->simTotalMsec % (this->pubIntervalGroundTruthArray) == 0)
            SutTeBridgeNode::publishGroundTruthArray();
        
        if(this->simTotalMsec % (this->pubIntervalVectorNavData) == 0)
            SutTeBridgeNode::publishVectorNavData();

        if(this->simTotalMsec % (this->pubIntervalNovatelData) == 0)
        {            
            SutTeBridgeNode::publishNovatelData(1);
            SutTeBridgeNode::publishNovatelData(2);
        }

        if(this->simTotalMsec % (this->pubIntervalFoxgloveMap) == 0)
    	  {
          SutTeBridgeNode::publishFoxgloveMap();
          // SutTeBridgeNode::publishFoxgloveSceneUpdate();
        }

        this->vesiDataAvailabe = false;
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << "Publishing of data failed" << e.what() << '\n';
    }
  }

  void SutTeBridgeNode::publishFoxgloveMap()
  {
    if (this->verbosePrinting)
    {
      std::cout << "publishFoxgloveMap" << '\n';
    }

    // Best Pos
    auto foxgloveMap = sensor_msgs::msg::NavSatFix();

    foxgloveMap.status.status = -1;
    foxgloveMap.status.service = 1;
    
    foxgloveMap.latitude = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lat;
    foxgloveMap.longitude = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lon;
    foxgloveMap.altitude = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.hgt;

    foxgloveMap.position_covariance_type = 0;
    // Header
    foxgloveMap.header.frame_id = "world";
    if(this->simModeEnabled)
    {
      foxgloveMap.header.stamp.sec = this->sec;
      foxgloveMap.header.stamp.nanosec = this->nsec;
    }
    else
    {
      foxgloveMap.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      foxgloveMap.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (foxgloveMap.header.stamp.sec*1000000000);
    }
  
    this->foxgloveMapPublisher_->publish(foxgloveMap);
  }

  void SutTeBridgeNode::publishFoxgloveSceneUpdate()
  {
    if (this->verbosePrinting)
    {
      std::cout << "publishFoxgloveSceneUpdate" << '\n';
    }

    auto foxgloveScene = foxglove_msgs::msg::SceneUpdate();

    auto fellows = foxglove_msgs::msg::SceneEntity();

    auto trackPosition = geometry_msgs::msg::TransformStamped();
    auto egoPosition = geometry_msgs::msg::TransformStamped();

    if(this->simModeEnabled)
    {
      egoPosition.header.stamp.sec = this->sec;
      egoPosition.header.stamp.nanosec = this->nsec;
    }
    else
    {
      egoPosition.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      egoPosition.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (egoPosition.header.stamp.sec*1000000000);
    }

    egoPosition.header.frame_id = "world";
    egoPosition.child_frame_id = "ego";

    egoPosition.transform.translation.x = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lat;
    egoPosition.transform.translation.y = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lon;
    egoPosition.transform.translation.z = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.hgt;

    tf2::Quaternion q_ego;

    q_ego.setRPY(0, this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.heading_2_var.pitch, this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.heading_2_var.heading);

    egoPosition.transform.rotation.x = q_ego.x();
    egoPosition.transform.rotation.y = q_ego.y();
    egoPosition.transform.rotation.z = q_ego.z();
    egoPosition.transform.rotation.w = q_ego.w();

    if (this->verbosePrinting)
    {
      std::cout << "publish ego position" << '\n';
    }
    this->egoPositionPublisher_->publish(egoPosition);

    // fellows = foxgloveScene.entities.add();
    foxgloveScene.entities.push_back(fellows);

    if(this->simModeEnabled)
    {
      fellows.timestamp.sec = this->sec;
      fellows.timestamp.nanosec = this->nsec;
    }
    else
    {
      fellows.timestamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      fellows.timestamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (fellows.timestamp.sec*1000000000);
    }

    fellows.frame_id = "world";

    fellows.frame_locked = true;

    fellows.cubes.resize(this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count+1);

    if (this->verbosePrinting)
    {
      std::cout << "add ego position " << this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count << '\n';
    }

    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].pose.position.x = egoPosition.transform.translation.x;
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].pose.position.y = egoPosition.transform.translation.y;
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].pose.position.z = egoPosition.transform.translation.z;
    
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].pose.orientation.x = q_ego.x();
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].pose.orientation.y = q_ego.y();
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].pose.orientation.z = q_ego.z();
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].pose.orientation.w = q_ego.w();
    
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].size.x = 3.5;
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].size.x = 2;
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].size.x = 1;
    
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].color.r = 0.6;
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].color.g = 0.2;
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].color.b = 1;
    fellows.cubes[this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count].color.a = 1;

    for (size_t vehicleID = 0; vehicleID < this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count; vehicleID++)
    {
      if (this->verbosePrinting)
      {
        std::cout << "add fellow position " << vehicleID << '\n';
      }
      fellows.cubes[vehicleID].pose.position.x = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[vehicleID];
      fellows.cubes[vehicleID].pose.position.y = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[vehicleID];
      fellows.cubes[vehicleID].pose.position.z = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[vehicleID];
      
      tf2::Quaternion q_fellow;

      q_fellow.setRPY(this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.roll[vehicleID], this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.pitch[vehicleID], this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.yaw[vehicleID]);
      
      fellows.cubes[vehicleID].pose.orientation.x = q_fellow.x();
      fellows.cubes[vehicleID].pose.orientation.y = q_fellow.y();
      fellows.cubes[vehicleID].pose.orientation.z = q_fellow.z();
      fellows.cubes[vehicleID].pose.orientation.w = q_fellow.w();
      
      fellows.cubes[vehicleID].size.x = 3.5;
      fellows.cubes[vehicleID].size.x = 2;
      fellows.cubes[vehicleID].size.x = 1;
      
      fellows.cubes[vehicleID].color.r = 0.8;
      fellows.cubes[vehicleID].color.g = 0.8;
      fellows.cubes[vehicleID].color.b = 0.2;
      fellows.cubes[vehicleID].color.a = 1;
    }

    if (this->verbosePrinting)
    {
      std::cout << "publish scene update" << '\n';
    }
    this->foxgloveScenePublisher_->publish(foxgloveScene);
  }

  void SutTeBridgeNode::sendVehicleFeedbackToSimulation()
  {
    if (this->verbosePrinting)
    {
      std::cout << "sendVehicleFeedbackToSimulation" << '\n';
    }

    this->api.sendControlData(22222,std::addressof(this->feedbackCmd),sizeof(this->feedbackCmd));
    if(this->simModeEnabled)
      this->api.increaseSimulationTime(0.001);
    else
      this->api.increaseSimulationTime(0.01);
  }

  void SutTeBridgeNode::subscribeVehicleCommandsCallback(const autonoma_msgs::msg::VehicleInputs & msg)
  {
    if (this->verbosePrinting)
    {
      std::cout << "subscribeVehicleCommandsCallback" << '\n';
    }

    // Throttle command (%)
    this->feedbackCmd.vehicle_inputs.throttle_cmd = msg.throttle_cmd;

    this->feedbackCmd.vehicle_inputs.throttle_cmd_count = msg.throttle_cmd_count;
    this->feedbackCmd.vehicle_inputs.enable_throttle_cmd = 1;

    // # Brake pressure command (kPa)
    this->feedbackCmd.vehicle_inputs.brake_cmd = msg.brake_cmd;
    this->feedbackCmd.vehicle_inputs.brake_cmd_count = msg.brake_cmd_count;
    this->feedbackCmd.vehicle_inputs.enable_brake_cmd = 1;

    // # Steering motor angle command (degrees)
    this->feedbackCmd.vehicle_inputs.steering_cmd = msg.steering_cmd;
    this->feedbackCmd.vehicle_inputs.steering_cmd_count = msg.steering_cmd_count;
    this->feedbackCmd.vehicle_inputs.enable_steering_cmd = 1;

    // # Gear command
    this->feedbackCmd.vehicle_inputs.gear_cmd = msg.gear_cmd;
    this->feedbackCmd.vehicle_inputs.enable_gear_cmd = 1;

    this->feedbackDataAvailabe = true;
  }

  void SutTeBridgeNode::subscribeRaptorCommandsCallback(const autonoma_msgs::msg::ToRaptor & msg)
  {
    if (this->verbosePrinting)
    {
      std::cout << "subscribeRaptorCommandsCallback" << '\n';
    }

    // time 5 end "stack_answer_time_toraptor"

    this->feedbackCmd.to_raptor.track_cond_ack = msg.track_cond_ack;
    this->feedbackCmd.to_raptor.veh_sig_ack = msg.veh_sig_ack;
    this->feedbackCmd.to_raptor.ct_state = msg.ct_state;
    this->feedbackCmd.to_raptor.rolling_counter = msg.rolling_counter;
    if (msg.veh_num != 0)
    {
      this->feedbackCmd.to_raptor.veh_num = msg.veh_num;
    }
    else
    {
      if (this->numberWarningSent == false)
      {
        std::cerr << "Vehicle number should not be zero. Will be replaced by 255. Change this configuration to enable racing" << '\n';
        this->numberWarningSent = true;
      }
      this->feedbackCmd.to_raptor.veh_num = 255;
    }
    
    this->raptorDataAvailabe = true;
  }

  void SutTeBridgeNode::switchRaceControlSourceCallback(const std_msgs::msg::Bool & msg)
  {
    if (this->verbosePrinting)
    {
      std::cout << "switchRaceControlSourceCallback" << '\n';
    }

    this->useCustomRaceControl = msg.data;
  }

  void SutTeBridgeNode::publishRaceControlData()
  {
    if (this->verbosePrinting)
    {
      std::cout << "publishRaceControlData" << '\n';
    }

    auto raceControlData = autonoma_msgs::msg::RaceControl();

    // RaceControl Message
    if (this->maneuverStarted)
    {
      raceControlData.base_to_car_heartbeat = this->canBus->asm_bus_var.race_control_var.base_to_car_heartbeat;
      // raceControlData.track_flag = this->canBus->asm_bus_var.race_control_var.track_flag;
      if (this->canBus->asm_bus_var.race_control_var.track_flag == 4)
      {
        raceControlData.track_flag = 4;
      }
      else
      {
        raceControlData.track_flag = this->canBus->asm_bus_var.race_control_var.track_flag;
      }
      raceControlData.veh_flag = this->canBus->asm_bus_var.race_control_var.veh_flag;
      raceControlData.sys_state = this->canBus->asm_bus_var.race_control_var.sys_state;
      raceControlData.veh_rank = this->canBus->asm_bus_var.race_control_var.veh_rank;
      raceControlData.lap_count = this->canBus->asm_bus_var.race_control_var.lap_count;
      raceControlData.lap_distance = this->canBus->asm_bus_var.race_control_var.lap_distance;
      raceControlData.round_target_speed = this->canBus->asm_bus_var.race_control_var.round_target_speed;

      raceControlData.laps = this->canBus->asm_bus_var.race_control_var.laps;
      raceControlData.lap_time = this->canBus->asm_bus_var.race_control_var.lap_time;
      raceControlData.time_stamp = this->canBus->asm_bus_var.race_control_var.time_stamp;
    }
    else
    {
      raceControlData.base_to_car_heartbeat = this->canBus->asm_bus_var.race_control_var.base_to_car_heartbeat;
      raceControlData.track_flag = 255;
      raceControlData.veh_flag = 1;
      raceControlData.sys_state = 255;
      raceControlData.veh_rank = 1;
      raceControlData.lap_count = 0;
      raceControlData.lap_distance = 0;
      raceControlData.round_target_speed = 0;

      raceControlData.laps = 0;
      raceControlData.lap_time = 0;
      raceControlData.time_stamp = this->canBus->asm_bus_var.race_control_var.time_stamp;
    }
    
    

    // Header
    raceControlData.header.frame_id = "";

    if(this->simModeEnabled)
    {
      raceControlData.header.stamp.sec = this->sec;
      raceControlData.header.stamp.nanosec = this->nsec;
    }
    else
    {
      raceControlData.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      raceControlData.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (raceControlData.header.stamp.sec*1000000000);
    }

    if (this->useCustomRaceControl == false)
    {
      this->raceControlDataPublisher_->publish(raceControlData);
    }
  }

  void SutTeBridgeNode::publishVehicleData()
  {
    if (this->verbosePrinting)
    {
      std::cout << "publishVehicleData" << '\n';
    }

    auto vehicleData = autonoma_msgs::msg::VehicleData();

    // Vehicle Data Message
    // Tire data
    vehicleData.fl_tire_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature;
    vehicleData.fl_damper_linear_potentiometer = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_damper_linear_potentiometer;
    vehicleData.fl_tire_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_pressure;
    vehicleData.fl_tire_pressure_gauge = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_pressure_gauge;
    vehicleData.fl_wheel_load = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_wheel_load;

    vehicleData.fr_tire_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature;
    vehicleData.fr_damper_linear_potentiometer = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_damper_linear_potentiometer;
    vehicleData.fr_tire_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_pressure;
    vehicleData.fr_tire_pressure_gauge = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_pressure_gauge;
    vehicleData.fr_wheel_load = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_wheel_load;

    vehicleData.rl_tire_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature;
    vehicleData.rl_damper_linear_potentiometer = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_damper_linear_potentiometer;
    vehicleData.rl_tire_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_pressure;
    vehicleData.rl_tire_pressure_gauge = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_pressure_gauge;
    vehicleData.rl_wheel_load = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_wheel_load;

    vehicleData.rr_tire_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature;
    vehicleData.rr_damper_linear_potentiometer = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_damper_linear_potentiometer;
    vehicleData.rr_tire_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_pressure;
    vehicleData.rr_tire_pressure_gauge = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_pressure_gauge;
    vehicleData.rr_wheel_load = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_wheel_load;

    // Brake temps
    vehicleData.fl_brake_temp = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_brake_temp;
    vehicleData.fr_brake_temp = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_brake_temp;
    vehicleData.rl_brake_temp = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_brake_temp;
    vehicleData.rr_brake_temp = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_brake_temp;

    // Misc data
    vehicleData.battery_voltage = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.battery_voltage;
    vehicleData.safety_switch_state = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.safety_switch_state;
    vehicleData.mode_switch_state = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.mode_switch_state;
    vehicleData.sys_state = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.sys_state;

    // Accel pedal report 
    vehicleData.accel_pedal_input = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.accel_pedal_input;
    vehicleData.accel_pedal_output = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.accel_pedal_output;

    // Brake report
    vehicleData.front_brake_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.front_brake_pressure;
    vehicleData.rear_brake_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rear_brake_pressure;

    // Steering report
    vehicleData.steering_wheel_angle = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_angle;
    vehicleData.steering_wheel_angle_cmd = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_angle_cmd;
    vehicleData.steering_wheel_torque = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_torque;

    // Wheel speeds (kph)
    vehicleData.ws_front_left = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_front_left;
    vehicleData.ws_front_right = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_front_right;
    vehicleData.ws_rear_left = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_rear_left;
    vehicleData.ws_rear_right = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_rear_right;

    // Header
    vehicleData.header.frame_id = "ego";

    if(this->simModeEnabled)
    {
      vehicleData.header.stamp.sec = this->sec;
      vehicleData.header.stamp.nanosec = this->nsec;
    }
    else
    {
      vehicleData.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      vehicleData.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (vehicleData.header.stamp.sec*1000000000);
    }

    this->vehicleDataPublisher_->publish(vehicleData);
  }

  void SutTeBridgeNode::publishPowertrainData()
  {
    if (this->verbosePrinting)
    {
      std::cout << "publishPowertrainData" << '\n';
    }
    
    auto powertrainData = autonoma_msgs::msg::PowertrainData();

    // Powertrain Data Message
    // Data
    powertrainData.map_sensor = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.map_sensor;
    powertrainData.lambda_sensor = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.lambda_sensor;
    
    powertrainData.fuel_level = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.fuel_level;
    powertrainData.fuel_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.fuel_pressure;
    
    powertrainData.engine_oil_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_oil_pressure;
    powertrainData.engine_oil_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_oil_temperature;
    powertrainData.engine_coolant_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_coolant_temperature;
    powertrainData.engine_coolant_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_coolant_pressure;
    
    powertrainData.engine_rpm = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_rpm;
    powertrainData.engine_on_status = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_on_status;
    powertrainData.engine_run_switch_status = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_run_switch_status;
    powertrainData.throttle_position = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.throttle_position;
    
    powertrainData.current_gear = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.current_gear;
    powertrainData.gear_shift_status = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.gear_shift_status;

    powertrainData.transmission_oil_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.transmission_oil_pressure;
    powertrainData.transmission_accumulator_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.transmission_accumulator_pressure;
    powertrainData.transmission_oil_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.transmission_oil_temperature;
    powertrainData.vehicle_speed_kmph = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.vehicle_speed_kmph;
    powertrainData.torque_wheels_nm = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.torque_wheels_nm;

    // Header
    powertrainData.header.frame_id = "ego";

    if(this->simModeEnabled)
    {
      powertrainData.header.stamp.sec = this->sec;
      powertrainData.header.stamp.nanosec = this->nsec;
    }
    else
    {
      powertrainData.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      powertrainData.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (powertrainData.header.stamp.sec*1000000000);
    }

    this->powertrainDataPublisher_->publish(powertrainData);
  }

  void SutTeBridgeNode::publishGroundTruthArray()
  {
    auto groundTruthArray = autonoma_msgs::msg::GroundTruthArray();

    groundTruthArray.vehicles.resize(this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count);

    // Header
    groundTruthArray.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      groundTruthArray.header.stamp.sec = this->sec;
      groundTruthArray.header.stamp.nanosec = this->nsec;
    }
    else
    {
      groundTruthArray.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      groundTruthArray.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (groundTruthArray.header.stamp.sec*1000000000);
    }

    for (size_t vehicleID = 0; vehicleID < this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count; vehicleID++)
    {
      groundTruthArray.vehicles[vehicleID].header.frame_id = groundTruthArray.header.frame_id;
      groundTruthArray.vehicles[vehicleID].header.stamp.sec = groundTruthArray.header.stamp.sec;
      groundTruthArray.vehicles[vehicleID].header.stamp.nanosec = groundTruthArray.header.stamp.nanosec;

      groundTruthArray.vehicles[vehicleID].car_num = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.car_num[vehicleID];
      
      groundTruthArray.vehicles[vehicleID].lat = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[vehicleID];
      groundTruthArray.vehicles[vehicleID].lon = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[vehicleID];
      groundTruthArray.vehicles[vehicleID].hgt = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[vehicleID];
      
      groundTruthArray.vehicles[vehicleID].vx = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.vx[vehicleID];
      groundTruthArray.vehicles[vehicleID].vy = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.vy[vehicleID];
      groundTruthArray.vehicles[vehicleID].vz = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.vz[vehicleID];
      
      groundTruthArray.vehicles[vehicleID].yaw = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.yaw[vehicleID];
      groundTruthArray.vehicles[vehicleID].pitch = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.pitch[vehicleID];
      groundTruthArray.vehicles[vehicleID].roll = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.roll[vehicleID];
      
      groundTruthArray.vehicles[vehicleID].del_x = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.del_x[vehicleID];
      groundTruthArray.vehicles[vehicleID].del_y = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.del_y[vehicleID];
      groundTruthArray.vehicles[vehicleID].del_z = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.del_z[vehicleID];
    }
    
    this->groundTruthArrayPublisher_->publish(groundTruthArray);
  }

  void SutTeBridgeNode::publishVectorNavData()
  {

    if (this->verbosePrinting)
    {
      std::cout << "publishVectorNavData" << '\n';
    }

    auto attitudeGroup = vectornav_msgs::msg::AttitudeGroup();
    auto commonGroup = vectornav_msgs::msg::CommonGroup();
    auto imuGroup = vectornav_msgs::msg::ImuGroup();
    auto gpsGroup = vectornav_msgs::msg::GpsGroup();
    auto insGroup = vectornav_msgs::msg::InsGroup();
    auto timeGroup = vectornav_msgs::msg::TimeGroup();

    // Header
    attitudeGroup.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      attitudeGroup.header.stamp.sec = this->sec;
      attitudeGroup.header.stamp.nanosec = this->nsec;
    }
    else
    {
      attitudeGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      attitudeGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (attitudeGroup.header.stamp.sec*1000000000);
    }

    attitudeGroup.vpestatus.attitude_quality = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.attitude_quality;
    attitudeGroup.vpestatus.gyro_saturation = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.gyro_saturation;
    attitudeGroup.vpestatus.gyro_saturation_recovery = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.gyro_saturation_recovery;
    attitudeGroup.vpestatus.mag_disturbance = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.mag_disturbance;
    attitudeGroup.vpestatus.mag_saturation = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.mag_saturation;
    attitudeGroup.vpestatus.acc_disturbance = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.acc_disturbance;
    attitudeGroup.vpestatus.acc_saturation = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.acc_saturation;
    attitudeGroup.vpestatus.known_mag_disturbance = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.known_mag_disturbance;
    attitudeGroup.vpestatus.known_accel_disturbance = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.known_accel_disturbance;

    attitudeGroup.yawpitchroll.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.yawpitchroll_var.x; 
    attitudeGroup.yawpitchroll.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.yawpitchroll_var.y; 
    attitudeGroup.yawpitchroll.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.yawpitchroll_var.z;

    attitudeGroup.quaternion.w = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.quaternion_var.w;
    attitudeGroup.quaternion.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.quaternion_var.x;
    attitudeGroup.quaternion.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.quaternion_var.y;
    attitudeGroup.quaternion.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.quaternion_var.z;
    
    attitudeGroup.dcm[0] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[0];
    attitudeGroup.dcm[1] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[1];
    attitudeGroup.dcm[2] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[2];
    attitudeGroup.dcm[3] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[3];
    attitudeGroup.dcm[4] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[4];
    attitudeGroup.dcm[5] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[5];
    attitudeGroup.dcm[6] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[6];
    attitudeGroup.dcm[7] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[7];
    attitudeGroup.dcm[8] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[8];
    
    attitudeGroup.magned.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.magned_var.x;
    attitudeGroup.magned.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.magned_var.y;
    attitudeGroup.magned.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.magned_var.z;

    attitudeGroup.accelned.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.accelned_var.x;
    attitudeGroup.accelned.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.accelned_var.y;
    attitudeGroup.accelned.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.accelned_var.z;

    attitudeGroup.linearaccelbody.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelbody_var.x;
    attitudeGroup.linearaccelbody.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelbody_var.y;
    attitudeGroup.linearaccelbody.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelbody_var.z;

    attitudeGroup.linearaccelned.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelned_var.x;
    attitudeGroup.linearaccelned.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelned_var.y;
    attitudeGroup.linearaccelned.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelned_var.z;
    
    attitudeGroup.ypru.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.ypru_var.x;
    attitudeGroup.ypru.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.ypru_var.y;
    attitudeGroup.ypru.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.ypru_var.z;

    this->verctorNavAttitudeGroupPublisher_->publish(attitudeGroup);

    // Header
    commonGroup.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      commonGroup.header.stamp.sec = this->sec;
      commonGroup.header.stamp.nanosec = this->nsec;
    }
    else
    {
      commonGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      commonGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (commonGroup.header.stamp.sec*1000000000);
    }

    commonGroup.timestartup = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.timestartup;
    commonGroup.timegps = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.timegps;
    commonGroup.timesyncin = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.timesyncin;

    commonGroup.yawpitchroll.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.yawpitchroll_var.x;
    commonGroup.yawpitchroll.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.yawpitchroll_var.y;
    commonGroup.yawpitchroll.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.yawpitchroll_var.z;

    commonGroup.quaternion.w = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.quaternion_var.w;
    commonGroup.quaternion.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.quaternion_var.x;
    commonGroup.quaternion.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.quaternion_var.y;
    commonGroup.quaternion.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.quaternion_var.z;

    commonGroup.angularrate.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.angularrate_var.x;
    commonGroup.angularrate.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.angularrate_var.y;
    commonGroup.angularrate.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.angularrate_var.z;

    commonGroup.position.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.position_var.x;
    commonGroup.position.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.position_var.y;
    commonGroup.position.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.position_var.z;

    commonGroup.velocity.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.velocity_var.x;
    commonGroup.velocity.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.velocity_var.y;
    commonGroup.velocity.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.velocity_var.z;

    commonGroup.accel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.accel_var.x;
    commonGroup.accel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.accel_var.y;
    commonGroup.accel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.accel_var.z;

    commonGroup.imu_accel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_accel_var.x;
    commonGroup.imu_accel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_accel_var.y;
    commonGroup.imu_accel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_accel_var.z;

    commonGroup.imu_rate.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_rate_var.x;
    commonGroup.imu_rate.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_rate_var.y;
    commonGroup.imu_rate.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_rate_var.z;

    commonGroup.magpres_mag.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.magpres_mag_var.x;
    commonGroup.magpres_mag.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.magpres_mag_var.y;
    commonGroup.magpres_mag.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.magpres_mag_var.z;

    commonGroup.magpres_temp = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.magpres_temp;
    commonGroup.magpres_pres = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.magpres_pres;
    commonGroup.deltatheta_dtime = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dtime;

    commonGroup.deltatheta_dtheta.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dtheta_var.x;
    commonGroup.deltatheta_dtheta.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dtheta_var.y;
    commonGroup.deltatheta_dtheta.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dtheta_var.z;

    commonGroup.deltatheta_dvel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dvel_var.x;
    commonGroup.deltatheta_dvel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dvel_var.y;
    commonGroup.deltatheta_dvel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dvel_var.z;

    commonGroup.insstatus.gps_fix = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.gps_fix;
    commonGroup.insstatus.time_error = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.time_error;
    commonGroup.insstatus.imu_error = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.imu_error;
    commonGroup.insstatus.mag_pres_error = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.mag_pres_error;
    commonGroup.insstatus.gps_error = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.gps_error;
    commonGroup.insstatus.gps_heading_ins = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.gps_heading_ins;
    commonGroup.insstatus.gps_compass = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.gps_compass;

    commonGroup.syncincnt = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.syncincnt;
    commonGroup.timegpspps = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.timegpspps;

    this->verctorNavCommonGroupPublisher_->publish(commonGroup);

    // Header
    imuGroup.header.frame_id = "ego";

    if(this->simModeEnabled)
    {
      imuGroup.header.stamp.sec = this->sec;
      imuGroup.header.stamp.nanosec = this->nsec;
    }
    else
    {
      imuGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      imuGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (imuGroup.header.stamp.sec*1000000000);
    }

    imuGroup.imustatus = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.imustatus;

    imuGroup.uncompmag.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.x;
    imuGroup.uncompmag.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.y;
    imuGroup.uncompmag.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.z;

    imuGroup.uncompmag.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.x;
    imuGroup.uncompmag.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.y;
    imuGroup.uncompmag.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.z;

    imuGroup.uncompaccel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompaccel_var.x;
    imuGroup.uncompaccel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompaccel_var.y;
    imuGroup.uncompaccel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompaccel_var.z;

    imuGroup.uncompgyro.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompgyro_var.x;
    imuGroup.uncompgyro.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompgyro_var.y;
    imuGroup.uncompgyro.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompgyro_var.z;

    imuGroup.temp = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.temp;
    imuGroup.pres = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.pres;
    imuGroup.deltatheta_time = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltatheta_time;

    imuGroup.deltatheta_dtheta.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltatheta_dtheta_var.x;
    imuGroup.deltatheta_dtheta.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltatheta_dtheta_var.y;
    imuGroup.deltatheta_dtheta.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltatheta_dtheta_var.z;

    imuGroup.deltavel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltavel_var.x;
    imuGroup.deltavel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltavel_var.y;
    imuGroup.deltavel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltavel_var.z;

    imuGroup.mag.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.mag_var.x;
    imuGroup.mag.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.mag_var.y;
    imuGroup.mag.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.mag_var.z;

    imuGroup.accel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.accel_var.x;
    imuGroup.accel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.accel_var.y;
    imuGroup.accel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.accel_var.z;

    imuGroup.angularrate.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.angularrate_var.x;
    imuGroup.angularrate.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.angularrate_var.y;
    imuGroup.angularrate.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.angularrate_var.z;

    imuGroup.sensat = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.sensat;

    this->verctorNavImuGroupPublisher_->publish(imuGroup);
    

    for(int i=0;i<2;i++)
    {
      gps_group currentGPS;
      if (i == 0)
      {
        currentGPS = this->canBus->sim_interface_var.vector_nav_vn1_var.gps_group1_var;
        this->verctorNavGpsGroupPublisher = this->verctorNavGpsGroupRightPublisher_;
        }
      else if (i == 1)
      {
        currentGPS = this->canBus->sim_interface_var.vector_nav_vn1_var.gps_group2_var;
        this->verctorNavGpsGroupPublisher = this->verctorNavGpsGroupLeftPublisher_;
      }
      else
      {
        std::cerr << "Only two Vectornav GPS antennas are supported.\n";
      }
      
      // Header
      gpsGroup.header.frame_id = "world";

      if(this->simModeEnabled)
      {
        gpsGroup.header.stamp.sec = this->sec;
        gpsGroup.header.stamp.nanosec = this->nsec;
      }
      else
      {
        gpsGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        gpsGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (gpsGroup.header.stamp.sec*1000000000);
      }


      gpsGroup.utc.year = currentGPS.utc_var.year;
      gpsGroup.utc.month = currentGPS.utc_var.month;
      gpsGroup.utc.day = currentGPS.utc_var.day;
      gpsGroup.utc.hour = currentGPS.utc_var.hour;
      gpsGroup.utc.min = currentGPS.utc_var.min;
      gpsGroup.utc.sec = currentGPS.utc_var.sec;
      gpsGroup.utc.ms = currentGPS.utc_var.ms;

      gpsGroup.tow = currentGPS.tow;
      gpsGroup.week = currentGPS.week;
      gpsGroup.numsats = currentGPS.numsats;
      gpsGroup.fix = currentGPS.fix;

      gpsGroup.poslla.x = currentGPS.poslla_var.x;
      gpsGroup.poslla.y = currentGPS.poslla_var.y;
      gpsGroup.poslla.z = currentGPS.poslla_var.z;

      gpsGroup.posecef.x = currentGPS.posecef_var.x;
      gpsGroup.posecef.y = currentGPS.posecef_var.y;
      gpsGroup.posecef.z = currentGPS.posecef_var.z;

      gpsGroup.velned.x = currentGPS.velned_var.x;
      gpsGroup.velned.y = currentGPS.velned_var.y;
      gpsGroup.velned.z = currentGPS.velned_var.z;

      gpsGroup.velecef.x = currentGPS.velecef_var.x;
      gpsGroup.velecef.y = currentGPS.velecef_var.y;
      gpsGroup.velecef.z = currentGPS.velecef_var.z;

      gpsGroup.posu.x = currentGPS.posu_var.x;
      gpsGroup.posu.y = currentGPS.posu_var.y;
      gpsGroup.posu.z = currentGPS.posu_var.z;

      gpsGroup.velu = currentGPS.velu;
      gpsGroup.timeu = currentGPS.timeu;
      gpsGroup.timeinfo_status = currentGPS.timeinfo_status;
      gpsGroup.timeinfo_leapseconds = currentGPS.timeinfo_leapseconds;

      gpsGroup.dop.g = currentGPS.dop_var.g;
      gpsGroup.dop.p = currentGPS.dop_var.p;
      gpsGroup.dop.t = currentGPS.dop_var.t;
      gpsGroup.dop.v = currentGPS.dop_var.v;
      gpsGroup.dop.h = currentGPS.dop_var.h;
      gpsGroup.dop.n = currentGPS.dop_var.n;
      gpsGroup.dop.e = currentGPS.dop_var.e;

      this->verctorNavGpsGroupPublisher->publish(gpsGroup);
    }
    
    // Header
    insGroup.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      insGroup.header.stamp.sec = this->sec;
      insGroup.header.stamp.nanosec = this->nsec;
    }
    else
    {
      insGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      insGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (insGroup.header.stamp.sec*1000000000);
    }

    insGroup.insstatus.gps_fix = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.gps_fix;
    insGroup.insstatus.time_error = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.time_error;
    insGroup.insstatus.imu_error = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.imu_error;
    insGroup.insstatus.mag_pres_error = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.mag_pres_error;
    insGroup.insstatus.gps_error = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.gps_error;
    insGroup.insstatus.gps_heading_ins = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.gps_heading_ins;
    insGroup.insstatus.gps_compass = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.gps_compass;

    insGroup.poslla.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.poslla_var.x;
    insGroup.poslla.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.poslla_var.y;
    insGroup.poslla.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.poslla_var.z;

    insGroup.posecef.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.posecef_var.x;
    insGroup.posecef.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.posecef_var.y;
    insGroup.posecef.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.posecef_var.z;

    insGroup.velbody.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velbody_var.x;
    insGroup.velbody.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velbody_var.y;
    insGroup.velbody.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velbody_var.z;

    insGroup.velned.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velned_var.x;
    insGroup.velned.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velned_var.y;
    insGroup.velned.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velned_var.z;

    insGroup.velecef.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velecef_var.x;
    insGroup.velecef.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velecef_var.y;
    insGroup.velecef.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velecef_var.z;

    insGroup.magecef.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.magecef_var.x;
    insGroup.magecef.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.magecef_var.y;
    insGroup.magecef.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.magecef_var.z;

    insGroup.accelecef.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.accelecef_var.x;
    insGroup.accelecef.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.accelecef_var.y;
    insGroup.accelecef.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.accelecef_var.z;

    insGroup.linearaccelecef.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.linearaccelecef_var.x;
    insGroup.linearaccelecef.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.linearaccelecef_var.y;
    insGroup.linearaccelecef.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.linearaccelecef_var.z;

    insGroup.posu = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.posu_var;
    insGroup.velu = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velu;

    this->verctorNavInsGroupPublisher_->publish(insGroup);
    
    // Header
    timeGroup.header.frame_id = "";
    
    if(this->simModeEnabled)
    {
      timeGroup.header.stamp.sec = this->sec;
      timeGroup.header.stamp.nanosec = this->nsec;
    }
    else
    {
      timeGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      timeGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (timeGroup.header.stamp.sec*1000000000);
    }

    timeGroup.timestartup = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timestartup;
    timeGroup.timegps = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timegps;
    timeGroup.gpstow = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.gpstow;
    timeGroup.gpsweek = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.gpsweek;
    timeGroup.timesyncin = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timesyncin;
    timeGroup.timegpspps = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timegpspps;

    timeGroup.timeutc.year = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.year;
    timeGroup.timeutc.month = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.month;
    timeGroup.timeutc.day = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.day;
    timeGroup.timeutc.hour = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.hour;
    timeGroup.timeutc.min = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.min;
    timeGroup.timeutc.sec = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.sec;
    timeGroup.timeutc.ms = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.ms;

    timeGroup.syncincnt = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.syncincnt;
    timeGroup.syncoutcnt = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.syncoutcnt;

    timeGroup.timestatus.time_ok = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timestatus_var.time_ok;
    timeGroup.timestatus.date_ok = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timestatus_var.date_ok;
    timeGroup.timestatus.utctime_ok = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timestatus_var.utctime_ok;

    this->verctorNavTimeGroupPublisher_->publish(timeGroup);
  }

  void SutTeBridgeNode::publishNovatelData(uint8_t novatelID)
  {
    if (this->verbosePrinting)
    {
      std::cout << "publishNovatelData" << '\n';
    }
    
    nova_tel_pwr_pak currentNovatel;
    
    if (novatelID == 1)
    {
      currentNovatel = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var;
      this->novaTelBestPosPublisher = this->novaTelBestPosPublisher1_;
      this->novaTelBestGNSSPosPublisher = this->novaTelBestGNSSPosPublisher1_;
      this->novaTelBestVelPublisher = this->novaTelBestVelPublisher1_;
      this->novaTelBestGNSSVelPublisher = this->novaTelBestGNSSVelPublisher1_;
      // this->novaTelInspvaPublisher = this->novaTelInspvaPublisher1_;
      // this->novaTelHeading2Publisher = this->novaTelHeading2Publisher1_;
      this->novaTelRawImuXPublisher = this->novaTelRawImuXPublisher1_;
      this->novaTelImuPublisher = this->novaTelImuPublisher1_;
      this->novaTelOdomPublisher = this->novaTelOdomPublisher1_;
      this->novaTelFixPublisher = this->novaTelFixPublisher1_;
      }
    else if (novatelID == 2)
    {
      currentNovatel = this->canBus->sim_interface_var.nova_tel_pwr_pak2_var;
      this->novaTelBestPosPublisher = this->novaTelBestPosPublisher2_;
      this->novaTelBestGNSSPosPublisher = this->novaTelBestGNSSPosPublisher2_;
      this->novaTelBestVelPublisher = this->novaTelBestVelPublisher2_;
      this->novaTelBestGNSSVelPublisher = this->novaTelBestGNSSVelPublisher2_;
      // this->novaTelInspvaPublisher = this->novaTelInspvaPublisher2_;
      // this->novaTelHeading2Publisher = this->novaTelHeading2Publisher2_;
      this->novaTelRawImuXPublisher = this->novaTelRawImuXPublisher2_;
      this->novaTelImuPublisher = this->novaTelImuPublisher2_;
      this->novaTelOdomPublisher = this->novaTelOdomPublisher2_;
      this->novaTelFixPublisher = this->novaTelFixPublisher2_;
    }
    else
    {
      std::cerr << "Unknown ID of Novatel Device. Only two Novatels are supported.\n";
    }
    
    // Best Pos

    std::pair<uint16_t, uint32_t> gps_week_and_ms = GetGPSWeekAndMS();
    uint16_t gps_week_ = gps_week_and_ms.first;
    uint32_t gps_week_ms_ = gps_week_and_ms.second;
    float covariance_val = 0.0001;

    auto bestPos = novatel_oem7_msgs::msg::BESTPOS();
    bestPos.nov_header.message_name = currentNovatel.best_pos_var.nov_header_var.message_name[0];
    bestPos.nov_header.message_id = 0; //currentNovatel.best_pos_var.nov_header_var.message_id;
    bestPos.nov_header.message_type = 0; //currentNovatel.best_pos_var.nov_header_var.message_type;
    bestPos.nov_header.sequence_number = 0; //currentNovatel.best_pos_var.nov_header_var.sequence_number;
    bestPos.nov_header.time_status = 0; //currentNovatel.best_pos_var.nov_header_var.time_status;
    bestPos.nov_header.gps_week_number = gps_week_; //currentNovatel.best_pos_var.nov_header_var.gps_week_number;
    bestPos.nov_header.gps_week_milliseconds = gps_week_ms_; //currentNovatel.best_pos_var.nov_header_var.gps_week_milliseconds;
    bestPos.nov_header.idle_time = 0; //currentNovatel.best_pos_var.nov_header_var.idle_time;
    bestPos.nov_header.receiver_status = 0;

    bestPos.sol_status.status = currentNovatel.best_pos_var.sol_status;

    bestPos.pos_type.type = 50; //currentNovatel.best_pos_var.pos_type;
    
    bestPos.lat = currentNovatel.best_pos_var.lat;
    bestPos.lon = currentNovatel.best_pos_var.lon;
    bestPos.hgt = currentNovatel.best_pos_var.hgt;
    bestPos.undulation = currentNovatel.best_pos_var.undulation;
    bestPos.datum_id = currentNovatel.best_pos_var.datum_id;
    bestPos.lat_stdev = currentNovatel.best_pos_var.lat_stdev;
    bestPos.lon_stdev = currentNovatel.best_pos_var.lon_stdev;
    bestPos.hgt_stdev = currentNovatel.best_pos_var.hgt_stdev;

    bestPos.stn_id[0] = currentNovatel.best_pos_var.stn_id[0];
    bestPos.stn_id[1] = currentNovatel.best_pos_var.stn_id[1];
    bestPos.stn_id[2] = currentNovatel.best_pos_var.stn_id[2];
    bestPos.stn_id[3] = currentNovatel.best_pos_var.stn_id[3];

    bestPos.diff_age = 0.0; //currentNovatel.best_pos_var.diff_age;
    bestPos.sol_age = 0.0; //currentNovatel.best_pos_var.sol_age;
    bestPos.num_svs = 15; //currentNovatel.best_pos_var.num_svs;
    bestPos.num_sol_svs = 12; //currentNovatel.best_pos_var.num_sol_svs;
    bestPos.num_sol_l1_svs = 6; //currentNovatel.best_pos_var.num_sol_l1_svs;
    bestPos.num_sol_multi_svs = 6; //currentNovatel.best_pos_var.num_sol_multi_svs;
    bestPos.reserved = 0; //currentNovatel.best_pos_var.reserved;
    bestPos.ext_sol_stat.status = 0; //currentNovatel.best_pos_var.ext_sol_stat;

    bestPos.galileo_beidou_sig_mask = 0; //currentNovatel.best_pos_var.galileo_beidou_sig_mask;
    bestPos.gps_glonass_sig_mask = 0; //currentNovatel.best_pos_var.gps_glonass_sig_mask;

    // Header
    // if (novatelID == 2){ //bottom
    //   bestPos.header.frame_id = "world";
    // }
    // else{
    //   bestPos.header.frame_id = "";
    // }
    bestPos.header.frame_id = "";

    if(this->simModeEnabled)
    {
      bestPos.header.stamp.sec = this->sec;
      bestPos.header.stamp.nanosec = this->nsec;
    }
    else
    {
      bestPos.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      bestPos.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (bestPos.header.stamp.sec*1000000000);
    }

    this->novaTelBestPosPublisher->publish(bestPos);
    this->novaTelBestGNSSPosPublisher->publish(bestPos);
    
    // Best Vel
    auto bestVel = novatel_oem7_msgs::msg::BESTVEL();

    bestVel.nov_header.message_name = currentNovatel.best_vel_var.nov_header_var.message_name[0];
    bestVel.nov_header.message_id = 0; //currentNovatel.best_vel_var.nov_header_var.message_id;
    bestVel.nov_header.message_type = 0; //currentNovatel.best_vel_var.nov_header_var.message_type;
    bestVel.nov_header.sequence_number = 0; //currentNovatel.best_vel_var.nov_header_var.sequence_number;
    bestVel.nov_header.time_status = 0; //currentNovatel.best_vel_var.nov_header_var.time_status;
    bestVel.nov_header.gps_week_number = gps_week_; //currentNovatel.best_vel_var.nov_header_var.gps_week_number;
    bestVel.nov_header.gps_week_milliseconds = gps_week_ms_; //currentNovatel.best_vel_var.nov_header_var.gps_week_milliseconds;
    bestVel.nov_header.idle_time = 0.0; //currentNovatel.best_vel_var.nov_header_var.idle_time;
    bestVel.nov_header.receiver_status = 0;

    bestVel.sol_status.status = 50; //currentNovatel.best_vel_var.sol_status;

    bestVel.vel_type.type = 0; //currentNovatel.best_vel_var.vel_type;
    
    bestVel.latency = currentNovatel.best_vel_var.latency;
    bestVel.diff_age = currentNovatel.best_vel_var.diff_age;
    bestVel.hor_speed = currentNovatel.best_vel_var.hor_speed;
    bestVel.trk_gnd = currentNovatel.best_vel_var.trk_gnd;
    bestVel.ver_speed = currentNovatel.best_vel_var.ver_speed;
    bestVel.reserved = currentNovatel.best_vel_var.reserved;

    // Header
    bestVel.header.frame_id = "";

    if(this->simModeEnabled)
    {
      bestVel.header.stamp.sec = this->sec;
      bestVel.header.stamp.nanosec = this->nsec;
    }
    else
    {
      bestVel.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      bestVel.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (bestVel.header.stamp.sec*1000000000);
    }

    this->novaTelBestVelPublisher->publish(bestVel);
    this->novaTelBestGNSSVelPublisher->publish(bestVel);

    // // Inspva
    // auto inspva = novatel_oem7_msgs::msg::INSPVA();

    // inspva.nov_header.message_name = currentNovatel.inspava_var.nov_header_var.message_name[0];
    // inspva.nov_header.message_id = currentNovatel.inspava_var.nov_header_var.message_id;
    // inspva.nov_header.message_type = currentNovatel.inspava_var.nov_header_var.message_type;
    // inspva.nov_header.sequence_number = currentNovatel.inspava_var.nov_header_var.sequence_number;
    // inspva.nov_header.time_status = currentNovatel.inspava_var.nov_header_var.time_status;
    // inspva.nov_header.gps_week_number = currentNovatel.inspava_var.nov_header_var.gps_week_number;
    // inspva.nov_header.gps_week_milliseconds = currentNovatel.inspava_var.nov_header_var.gps_week_milliseconds;
    // inspva.nov_header.idle_time = currentNovatel.inspava_var.nov_header_var.idle_time;

    // inspva.latitude = currentNovatel.inspava_var.latitude;
    // inspva.longitude = currentNovatel.inspava_var.longitude;
    // inspva.height = currentNovatel.inspava_var.height;
    // inspva.north_velocity = currentNovatel.inspava_var.north_velocity;
    // inspva.east_velocity = currentNovatel.inspava_var.east_velocity;
    // inspva.up_velocity = currentNovatel.inspava_var.up_velocity;
    // inspva.roll = currentNovatel.inspava_var.roll;
    // inspva.pitch = currentNovatel.inspava_var.pitch;
    // inspva.azimuth = currentNovatel.inspava_var.azimuth;

    // inspva.status.status = currentNovatel.inspava_var.status_var.status_var;

    // // Header
    // inspva.header.frame_id = "world";

    // if(this->simModeEnabled)
    // {
    //   inspva.header.stamp.sec = this->sec;
    //   inspva.header.stamp.nanosec = this->nsec;
    // }
    // else
    // {
    //   inspva.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    //   inspva.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (inspva.header.stamp.sec*1000000000);
    // }

    // this->novaTelInspvaPublisher->publish(inspva);

    // // Heading 2
    // auto heading2 = novatel_oem7_msgs::msg::HEADING2();

    // heading2.nov_header.message_name = currentNovatel.heading_2_var.nov_header_var.message_name[0];
    // heading2.nov_header.message_id = currentNovatel.heading_2_var.nov_header_var.message_id;
    // heading2.nov_header.message_type = currentNovatel.heading_2_var.nov_header_var.message_type;
    // heading2.nov_header.sequence_number = currentNovatel.heading_2_var.nov_header_var.sequence_number;
    // heading2.nov_header.time_status = currentNovatel.heading_2_var.nov_header_var.time_status;
    // heading2.nov_header.gps_week_number = currentNovatel.heading_2_var.nov_header_var.gps_week_number;
    // heading2.nov_header.gps_week_milliseconds = currentNovatel.heading_2_var.nov_header_var.gps_week_milliseconds;
    // heading2.nov_header.idle_time = currentNovatel.heading_2_var.nov_header_var.idle_time;

    // heading2.sol_status.status = currentNovatel.heading_2_var.sol_status;

    // heading2.pos_type.type = currentNovatel.heading_2_var.pos_type;

    // heading2.length = currentNovatel.heading_2_var.length;
    // heading2.heading = currentNovatel.heading_2_var.heading;
    // heading2.pitch = currentNovatel.heading_2_var.pitch;
    // heading2.reserved = currentNovatel.heading_2_var.reserved;
    // heading2.heading_stdev = currentNovatel.heading_2_var.heading_stdev;
    // heading2.pitch_stdev = currentNovatel.heading_2_var.pitch_stdev;
    // heading2.rover_stn_id[0] = currentNovatel.heading_2_var.rover_stn_id[0];
    // heading2.rover_stn_id[1] = currentNovatel.heading_2_var.rover_stn_id[1];
    // heading2.rover_stn_id[2] = currentNovatel.heading_2_var.rover_stn_id[2];
    // heading2.rover_stn_id[3] = currentNovatel.heading_2_var.rover_stn_id[3];
    // heading2.master_stn_id[0] = currentNovatel.heading_2_var.master_stn_id[0];
    // heading2.master_stn_id[1] = currentNovatel.heading_2_var.master_stn_id[1];
    // heading2.master_stn_id[2] = currentNovatel.heading_2_var.master_stn_id[2];
    // heading2.master_stn_id[3] = currentNovatel.heading_2_var.master_stn_id[3];
    // heading2.num_sv_tracked = currentNovatel.heading_2_var.num_sv_tracked;
    // heading2.num_sv_in_sol = currentNovatel.heading_2_var.num_sv_in_sol;
    // heading2.num_sv_obs = currentNovatel.heading_2_var.num_sv_obs;
    // heading2.num_sv_multi = currentNovatel.heading_2_var.num_sv_multi;
    // heading2.sol_source.source = currentNovatel.heading_2_var.sol_source;
    // heading2.ext_sol_status.status = currentNovatel.heading_2_var.ext_sol_status;
    // heading2.galileo_beidou_sig_mask = currentNovatel.heading_2_var.galileo_beidou_sig_mask;
    // heading2.gps_glonass_sig_mask = currentNovatel.heading_2_var.gps_glonass_sig_mask;
    
    // // Header
    // heading2.header.frame_id = "world";

    // if(this->simModeEnabled)
    // {
    //   heading2.header.stamp.sec = this->sec;
    //   heading2.header.stamp.nanosec = this->nsec;
    // }
    // else
    // {
    //   heading2.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    //   heading2.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (heading2.header.stamp.sec*1000000000);
    // }

    // this->novaTelHeading2Publisher->publish(heading2);

    // Raw IMUX
    auto rawImuX = sensor_msgs::msg::Imu();

  //no orietnation

    rawImuX.angular_velocity.x = currentNovatel.raw_imu_var.angular_velocity_var.x;
    rawImuX.angular_velocity.y = currentNovatel.raw_imu_var.angular_velocity_var.y;
    rawImuX.angular_velocity.z = currentNovatel.raw_imu_var.angular_velocity_var.z;
    for (size_t i = 0; i < 9; i++) {rawImuX.angular_velocity_covariance[i] = 0;}
    rawImuX.angular_velocity_covariance[0] = covariance_val;
    rawImuX.angular_velocity_covariance[4] = covariance_val;
    rawImuX.angular_velocity_covariance[8] = covariance_val;

    rawImuX.linear_acceleration.x = currentNovatel.raw_imu_var.linear_acceleration_var.x;
    rawImuX.linear_acceleration.y = currentNovatel.raw_imu_var.linear_acceleration_var.y;
    rawImuX.linear_acceleration.z = currentNovatel.raw_imu_var.linear_acceleration_var.z;
    for (size_t i = 0; i < 9; i++) {rawImuX.linear_acceleration_covariance[i] = 0;}
    rawImuX.linear_acceleration_covariance[0] = covariance_val;
    rawImuX.linear_acceleration_covariance[4] = covariance_val;
    rawImuX.linear_acceleration_covariance[8] = covariance_val;
    
    // Header
    if (novatelID == 1)
    {
      rawImuX.header.frame_id = "gps_top_imu";
    }
    else
    {
      rawImuX.header.frame_id = "gps_bottom_imu";
    }

    if(this->simModeEnabled)
    {
      rawImuX.header.stamp.sec = this->sec;
      rawImuX.header.stamp.nanosec = this->nsec;
    }
    else
    {
      rawImuX.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      rawImuX.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (rawImuX.header.stamp.sec*1000000000);
    }

    this->novaTelRawImuXPublisher->publish(rawImuX);

    // Imu (imu/data)
    auto imuMsg = sensor_msgs::msg::Imu();

    // std::cout << "Checkpoint: Getting latitude, longitude, and height" << std::endl;
    double lat = currentNovatel.best_pos_var.lat;
    double lon =currentNovatel.best_pos_var.lon;
    double height = currentNovatel.best_pos_var.hgt;

    // std::cout << "Checkpoint: Calculating yaw" << std::endl;
    // double yaw = currentNovatel.heading_2_var.heading + 90.0;
    // double yaw = std::fmod(450.0 - currentNovatel.inspava_var.azimuth, 360.0) + 90.0;
    // std::cout << "INSPVA Azimuth: " << currentNovatel.inspava_var.azimuth << std::endl;
    double yaw = std::fmod(450.0 - currentNovatel.inspava_var.azimuth, 360.0) - 90.0;
    yaw = yaw * (3.1415926535 / 180.0);
    yaw = yaw * -1;
    // std::cout << "Calculated Yaw: " << yaw << std::endl;
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);

    imuMsg.orientation.x = 0.0; 
    imuMsg.orientation.y = 0.0;
    imuMsg.orientation.z = sy;
    imuMsg.orientation.w = cy;
    for (size_t i = 0; i < 9; i++) {rawImuX.orientation_covariance[i] = 0;}
    imuMsg.orientation_covariance[0] = covariance_val;
    imuMsg.orientation_covariance[4] = covariance_val;
    imuMsg.orientation_covariance[8] = covariance_val;
    
    imuMsg.angular_velocity.x = currentNovatel.raw_imu_var.angular_velocity_var.x;
    imuMsg.angular_velocity.y = currentNovatel.raw_imu_var.angular_velocity_var.y;
    imuMsg.angular_velocity.z = currentNovatel.raw_imu_var.angular_velocity_var.z;
    for (size_t i = 0; i < 9; i++) {imuMsg.angular_velocity_covariance[i] = 0;}
    imuMsg.angular_velocity_covariance[0] = covariance_val;
    imuMsg.angular_velocity_covariance[4] = covariance_val;
    imuMsg.angular_velocity_covariance[8] = covariance_val;

    imuMsg.linear_acceleration.x = currentNovatel.raw_imu_var.linear_acceleration_var.x;
    imuMsg.linear_acceleration.y = currentNovatel.raw_imu_var.linear_acceleration_var.y;
    imuMsg.linear_acceleration.z = currentNovatel.raw_imu_var.linear_acceleration_var.z;
    for (size_t i = 0; i < 9; i++) {imuMsg.linear_acceleration_covariance[i] = 0;}
    imuMsg.linear_acceleration_covariance[0] = covariance_val;
    imuMsg.linear_acceleration_covariance[4] = covariance_val;
    imuMsg.linear_acceleration_covariance[8] = covariance_val;

    // Header
    if (novatelID == 1)
    {
      imuMsg.header.frame_id = "gps_top";
    }
    else
    {
      imuMsg.header.frame_id = "gps_bottom";
    }

    // std::cout << "Checkpoint: 3" << std::endl;
    if(this->simModeEnabled)
    {
      imuMsg.header.stamp.sec = this->sec;
      imuMsg.header.stamp.nanosec = this->nsec;
    }
    else
    {
      imuMsg.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      imuMsg.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }

    // std::cout << "Checkpoint: 4" << std::endl;
    this->novaTelImuPublisher->publish(imuMsg);

    //Nav Sat Fix 
    auto fixMsg = sensor_msgs::msg::NavSatFix();

    fixMsg.status.status = 2;
    fixMsg.status.service = 3;
    fixMsg.latitude = currentNovatel.best_pos_var.lat; //seems lat and lon are switched for inspva, so use novatel instead
    fixMsg.longitude = currentNovatel.best_pos_var.lon;
    fixMsg.altitude = currentNovatel.inspava_var.height;
    fixMsg.position_covariance_type = 2;
    // Header
    if (novatelID == 1)
    {
      fixMsg.header.frame_id = "gps_top_ant1";
    }
    else
    {
      fixMsg.header.frame_id = "gps_bottom_ant1";
    }
    if(this->simModeEnabled)
    {
      fixMsg.header.stamp.sec = this->sec;
      fixMsg.header.stamp.nanosec = this->nsec;
    }
    else
    {
      fixMsg.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      fixMsg.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }

    this->novaTelFixPublisher->publish(fixMsg);

    //Odom
    auto odomMsg = nav_msgs::msg::Odometry();

    double easting, northing;
    int zone;
    bool north;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, north, easting, northing);
    // Now `easting` and `northing` contain the UTM coordinates, and `zone` and `north` indicate the UTM zone

    odomMsg.pose.pose.position.x = easting;
    odomMsg.pose.pose.position.y = northing;
    odomMsg.pose.pose.position.z = height;
    odomMsg.pose.pose.orientation.x = 0.0; 
    odomMsg.pose.pose.orientation.y = 0.0;
    odomMsg.pose.pose.orientation.z = sy;
    odomMsg.pose.pose.orientation.w = cy;

    for (size_t i = 0; i < 9; i++) {odomMsg.pose.covariance[i] = 0;}
    
    double fl = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_front_left;
    double fr = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_front_right;
    double rl = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_rear_left;
    double rr = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_rear_right;
    double vx = (fl + fr + rl + rr) / 4.0 / 3.6;

    odomMsg.twist.twist.linear.x = vx;


    odomMsg.twist.twist.angular.x = currentNovatel.raw_imu_var.angular_velocity_var.x;
    odomMsg.twist.twist.angular.y = currentNovatel.raw_imu_var.angular_velocity_var.y;
    odomMsg.twist.twist.angular.z = currentNovatel.raw_imu_var.angular_velocity_var.z;
    for (size_t i = 0; i < 9; i++) {odomMsg.twist.covariance[i] = 0;}

    // Header
    if (novatelID == 1)
    {
      odomMsg.header.frame_id = "utm";
      odomMsg.child_frame_id = "gps_top_ant1";
    }
    else
    {
      odomMsg.header.frame_id = "utm";
      odomMsg.child_frame_id = "gps_bottom_ant1";
    }
    if(this->simModeEnabled)
    {
      odomMsg.header.stamp.sec = this->sec;
      odomMsg.header.stamp.nanosec = this->nsec;
    }
    else
    {
      odomMsg.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      odomMsg.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }

    this->novaTelOdomPublisher->publish(odomMsg);
  }

  std::pair<uint16_t, uint32_t> SutTeBridgeNode::GetGPSWeekAndMS() {
      // Constants
      constexpr double GPS_EPOCH_OFFSET = 3657.0 * 24.0 * 60.0 * 60.0; // Seconds from Unix epoch to GPS epoch
      constexpr double WEEK_SECONDS = 604800.0;                        // Seconds in a GPS week
      constexpr double LEAP_SECONDS = 18.0;                            // Current leap seconds

      // Get current time in seconds since Unix epoch
      auto now = std::chrono::system_clock::now();
      auto unix_time = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

      // Calculate GPS time in seconds
      double gps_time = unix_time + LEAP_SECONDS - GPS_EPOCH_OFFSET;

      // Calculate GPS week and remaining milliseconds
      uint16_t gps_week = static_cast<uint16_t>(gps_time / WEEK_SECONDS);
      double remaining_seconds = gps_time - (gps_week * WEEK_SECONDS);
      uint32_t gps_week_ms = static_cast<uint32_t>(remaining_seconds * 1000.0);

      return {gps_week, gps_week_ms};
  }
}


int main(int argc, char * argv[])
{
  try
  {
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr SutTeBridgeNodePtr = std::make_shared<bridge::SutTeBridgeNode>();
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(SutTeBridgeNodePtr);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
  catch(const std::exception& e)
  {
    std::cerr << "Failed to initialize SUT-TE-Bridge node: " << e.what() << '\n';
  }
  
}
