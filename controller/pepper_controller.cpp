/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "pepper_controller.h"

namespace pepper_controller
{
    /**
     * @brief Constructor
     *
     * @param[in] broker
     * @param[in] name
     * @param[in] config_path
     */
    PepperController::PepperController(boost::shared_ptr<AL::ALBroker> broker,
                           const std::string& name, const std::string& config_path) : AL::ALModule(broker, name),
                           actuators_names_(Actuators::ACTUATORS_NUM, "Device/SubDeviceList/"),
                           memory_fast_access_(boost::shared_ptr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess())),
                           config_path_   (config_path),
                           wb_controller_ (config_path),
                           mpc_controller_(config_path),
                           pepper_controller_parameters_(config_path + "pepper-controller-parameters.yaml")
#ifdef CONTROLLER_LOGGING_ENABLED
                           ,pepper_logger_("/home/nao/pepper-controller-log.m")
#endif                           
    {
        setModuleDescription("BIP/INRIA motion controller");

        functionName("setActuators",               getName(), "set joints to a given configuration");
        BIND_METHOD(PepperController::setActuators);
        functionName("setActuatorsStiffness",      getName(), "set joints stiffness to a given value");
        BIND_METHOD(PepperController::setActuatorsStiffness);
        functionName("setUpperJointsStiffness",    getName(), "set upper joints stiffness to a given value");
        BIND_METHOD(PepperController::setUpperJointsStiffness);
        functionName("setWheelsStiffness",         getName(), "set wheels stiffness to a given value");
        BIND_METHOD(PepperController::setWheelsStiffness);
        functionName("startControl",               getName(), "starts DCM callback");
        BIND_METHOD(PepperController::startControl);
        functionName("stopControl",                getName(), "stop controller");
        BIND_METHOD(PepperController::stopControl);
        functionName("goInitialPose",              getName(), "go to initial pose");
        BIND_METHOD(PepperController::goInitialPose);
        functionName("goRestPose",                 getName(), "go to resting pose");
        BIND_METHOD(PepperController::goRestPose);
        functionName("setMPCMotionParameters",     getName(), "send mpc motion parameters");
        BIND_METHOD(PepperController::setMPCMotionParameters);
        functionName("setMPCMotionParametersIdle", getName(), "send mpc motion parameters to idle");
        BIND_METHOD(PepperController::setMPCMotionParametersIdle);
        functionName("printRootPose",              getName(), "print pose of the model root");
        BIND_METHOD(PepperController::printRootPose);
        functionName("killALMotionModule",         getName(), "kill ALMotion module");
        BIND_METHOD(PepperController::killALMotionModule);

        reset();
    }


    /**
     * @brief Destructor
     */
    PepperController::~PepperController()
    {
    }

    
    /**
     * @brief Get sensor readings
     *
     * @param[in, out] sensor_readings 
     */
    const std::vector<float> PepperController::getSensorReadings()
    {
        std::vector<float> sensor_readings;
        try
        {
            memory_fast_access_->GetValues(sensor_readings);
        }
        catch(const AL::ALError &e)
        {
            throw ALERROR(getName(), "getSensorReadings()", "Error reading sensor values: " + e.toString());
        }

        return(sensor_readings);
    }


    /**
     * @brief Get current generalized coordinates
     *
     * @return Generalized coordinates
     */
    humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES> PepperController::getCurrentGeneralizedCoordinatesFromSensors()
    {
        std::vector<float> readings = getSensorReadings();
        
        //first coords should be root coords - need to handle this (estimate from sensors)
        humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES> general_coords;
        general_coords.root_pose_ = wb_controller_.getModel().getState().root_pose_;

        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HeadPitch]      = readings[Actuators::HEAD_PITCH];      
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HeadYaw]        = readings[Actuators::HEAD_YAW];        
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LElbowRoll]     = readings[Actuators::L_ELBOW_ROLL];    
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LElbowYaw]      = readings[Actuators::L_ELBOW_YAW];     
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RElbowRoll]     = readings[Actuators::R_ELBOW_ROLL];    
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RElbowYaw]      = readings[Actuators::R_ELBOW_YAW];     
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LWristYaw]      = readings[Actuators::L_WRIST_YAW];
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RWristYaw]      = readings[Actuators::R_WRIST_YAW];     
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LShoulderPitch] = readings[Actuators::L_SHOULDER_PITCH];
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LShoulderRoll]  = readings[Actuators::L_SHOULDER_ROLL]; 
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RShoulderPitch] = readings[Actuators::R_SHOULDER_PITCH];
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RShoulderRoll]  = readings[Actuators::R_SHOULDER_ROLL]; 
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HipPitch]       = readings[Actuators::HIP_PITCH];       
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HipRoll]        = readings[Actuators::HIP_ROLL];        
        general_coords.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::KneePitch]      = readings[Actuators::KNEE_PITCH];      

        return(general_coords);
    }


    /**
     * @brief Go to the rest pose
     *
     * @param[in] execution_time_ms
     */
    void PepperController::goRestPose(const int execution_time_ms)
    {
        try
        {
            humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES> general_coords_from = getCurrentGeneralizedCoordinatesFromSensors();
            humoto::pepper_ik::Model<MODEL_FEATURES>                  model = wb_controller_.getModel();
            
            model.updateState(general_coords_from); 

#ifdef CONTROLLER_LOGGING_ENABLED
            humoto::pepper_ik::Model<MODEL_FEATURES> model_from = wb_controller_.getModel();
#endif            

            model.saveCurrentState();

            bool crash_on_missing_entry = true;
            humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES> general_coords_to(config_path_ + "rest-state-pepper-ik-planar.yaml",
                                                                                        crash_on_missing_entry);
           
            etools::Vector3 root_orientation_from;
            etools::Vector3 root_position_from;
            general_coords_from.getRootPosition(root_position_from);
            general_coords_from.getRootOrientation(root_orientation_from);

            general_coords_to.setRootPosition(root_position_from); 
            general_coords_to.setRootOrientation(root_orientation_from); 
            
            model.updateState(general_coords_to);

            humoto::pepper_ik::RobotCommand wb_command;
            model.getRobotCommand(wb_command, humoto::convertMillisecondToSecond(execution_time_ms));
 
#ifdef CONTROLLER_LOGGING_ENABLED
            if(pepper_controller_parameters_.log_for_simulation_)
            {
                pepper_logger_.logForSimulation(model_from, model, execution_time_ms, wb_controller_.getWBCParameters().control_interval_ms_);
            }
#endif

            sendWBCCommandWithOffset(execution_time_ms, wb_command);
        }
        catch(const std::exception& e)
        {
            stopRobotBase(); 
            qiLogInfo("goRestPose()", "exception: ") << e.what();
            throw;
        }
    }


    /**
     * @brief Go to initial pose
     *
     * @param[in] execution_time_ms
     */
    void PepperController::goInitialPose(const int execution_time_ms)
    {
        try
        {
            humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES> general_coords_from = getCurrentGeneralizedCoordinatesFromSensors();
            general_coords_from.root_pose_.setZero();
            
            humoto::pepper_ik::Model<MODEL_FEATURES> model = wb_controller_.getModel();
            
            // use this model for logging init state
            model.updateState(general_coords_from); 

#ifdef CONTROLLER_LOGGING_ENABLED
            humoto::pepper_ik::Model<MODEL_FEATURES> model_from = model;
#endif            

            model.saveCurrentState();

            bool crash_on_missing_entry = true;
            humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES> general_coords_to(config_path_ + "initial-state-pepper-ik-planar.yaml",
                                                                                        crash_on_missing_entry);
            // use this model to log final state
            model.updateState(general_coords_to);

            humoto::pepper_ik::RobotCommand wb_command;
            model.getRobotCommand(wb_command, humoto::convertMillisecondToSecond(execution_time_ms));

#ifdef CONTROLLER_LOGGING_ENABLED
            if(pepper_controller_parameters_.log_for_simulation_)
            {
                pepper_logger_.logForSimulation(model_from, model, execution_time_ms, wb_controller_.getWBCParameters().control_interval_ms_);
            }
#endif           

            sendWBCCommandWithOffset(execution_time_ms, wb_command);
        }
        catch(const std::exception& e)
        {
            stopRobotBase(); 
            qiLogInfo("goInitialPose()", "exception: ") << e.what();
            throw;
        }
    }


    /**
     * @brief Set MPC motion parameters to idle 
     */
    void PepperController::setMPCMotionParametersIdle()
    {
        mpc_controller_.readMPCMotionParametersIdle(); 
        qiLogInfo("setMPCMotionParametersIdle()", "Going to idle state.");
    }


    /**
     * @brief Set MPC motion parameters from file
     *
     * @param[in] filename
     */
    void PepperController::setMPCMotionParameters(const std::string& filename)
    {
        mpc_controller_.readMPCMotionParameters(config_path_ + filename); 
        qiLogInfo("setMPCMotionParameters()", "Motion parameters set.");
    }


    /**
     * @brief Connect callback to DCM loop
     */
    void PepperController::connectToDCMLoop()
    {
        // connect callback to the DCM post proccess
        try
        {
            DCM_connection_ = getParentBroker()->getProxy("DCM")->getModule()->
                                                atPostProcess(boost::bind(&PepperController::synchronisedDCMCallback, this));
        }
        catch(const AL::ALError &e)
        {
            throw ALERROR(getName(), "connectToDCMloop()", "Error when connecting to DCM postProccess: " + e.toString());
        }
    }


    /**
     * @brief Stop robot base
     */
    void PepperController::stopRobotBase()
    {
        std::vector<float> stop_command = getSensorReadings();
        stop_command[Actuators::JOINTS_NUM + Actuators::WHEEL_FR] = 0.0;
        stop_command[Actuators::JOINTS_NUM + Actuators::WHEEL_FL] = 0.0;
        stop_command[Actuators::JOINTS_NUM + Actuators::WHEEL_B]  = 0.0;
        setActuators(0, stop_command);
    }
    
    
    /**
     * @brief Start MPC controller
     */
    void PepperController::startControl()
    {
        connectToDCMLoop();
    }


    /**
     * @brief Stop MPC controller
     */
    void PepperController::stopControl()
    {
        disconnectFromDCMLoop();
        stopRobotBase();
        reset();
    }

    
    /**
     * @brief Reset controller
     */
    void PepperController::reset()
    {
        dcm_loop_counter_       = 0;
        expected_control_drift_ = 0.0;
        dcm_call_time_interval_ = 0.0;
        wb_commands_.clear();
        mpc_time_instants_.resize(pepper_controller_parameters_.n_states_future_);
        ik_motion_parameters_vector_.resize(pepper_controller_parameters_.n_states_future_);
    }


    /**
     * @brief Print root pose to the naoqi log file
     */
    void PepperController::printRootPose()
    {
        humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES> general_coords = wb_controller_.getModel().getState();
        
        etools::Vector3 base_position;
        etools::Vector3 base_rpy;
        general_coords.getRootPosition(base_position);
        general_coords.getRootOrientation(base_rpy);
        
        qiLogInfo("printBasePose()", "Base at: ") << base_position << " " <<
                                                     base_rpy      << " " << std::endl;
    }


    /**
     * @brief Callback function tied to DCM execution
     *
     * @attention Function executing in real-time thread
     */
    void PepperController::synchronisedDCMCallback()
    {
        if(!(dcm_loop_counter_ % pepper_controller_parameters_.dcm_iter_per_mpc_))
        {
            try
            {
                int dcm_current_time_ms  = *dcm_current_time_ptr_ms_;
                
                if(!(dcm_loop_counter_ == 0))
                {
                    dcm_call_time_interval_ = timer_dcm_.stop();
                }
                timer_dcm_.start();
                timer_control_.start();
                
                // compute DCM sampling time difference
                double sampling_interval_difference = (dcm_call_time_interval_ -
                                                       wb_controller_.getWBCParameters().control_interval_) + 
                                                       expected_control_drift_;

                for(std::size_t i = 0; i < mpc_time_instants_.size(); ++i)
                {
                    mpc_time_instants_[i] = (i + 1)*mpc_controller_.getMPCParameters().getSubsamplingTime() - expected_control_drift_;
                }

                // solve mpc
                if(!mpc_controller_.run(wb_controller_,
                                        ik_motion_parameters_vector_,
                                        wb_commands_,
                                        sampling_interval_difference,
                                        mpc_time_instants_))
                {
                    stopControl();
                    qiLogInfo("synchronisedDCMCallback()", "Controller successfully done.");
                    return;
                }

                wb_controller_.getWBMotionCommands(wb_commands_, mpc_time_instants_, ik_motion_parameters_vector_);
                expected_control_drift_ = timer_control_.stop();
                
                sendWBCCommand(dcm_current_time_ms, wb_commands_);

#ifdef CONTROLLER_LOGGING_ENABLED
                if(pepper_controller_parameters_.log_for_simulation_)
                {
                    pepper_logger_.logForSimulation(wb_controller_.getModel(), wb_commands_.front());
                    pepper_logger_.logValue(timer_control_.get(), "loop_solution_time", dcm_loop_counter_);
                }
#endif 

            }
            catch(const std::exception& e)
            {
                stopControl();
                qiLogInfo("synchronisedDCMCallback()", "exception: ") << e.what();
                throw;
            }
        }

        dcm_loop_counter_++;
    }


    /**
     * @brief Copy whole-body control command
     *
     * @param[in, out] command
     * @param[in]      wb_command
     * @param[in]      index 
     */
    void PepperController::copyCommand(AL::ALValue& command, const humoto::pepper_ik::RobotCommand& wb_command, const std::size_t index) const
    {
        // body
        command[5][Actuators::HEAD_PITCH][index]       = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HeadPitch];
        command[5][Actuators::HEAD_YAW][index]         = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HeadYaw]; 
        command[5][Actuators::L_ELBOW_ROLL][index]     = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LElbowRoll]; 
        command[5][Actuators::L_ELBOW_YAW][index]      = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LElbowYaw];
        command[5][Actuators::R_ELBOW_ROLL][index]     = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RElbowRoll]; 
        command[5][Actuators::R_ELBOW_YAW][index]      = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RElbowYaw]; 
        command[5][Actuators::L_WRIST_YAW][index]      = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LWristYaw];
        command[5][Actuators::R_WRIST_YAW][index]      = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RWristYaw];
        command[5][Actuators::L_SHOULDER_PITCH][index] = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LShoulderPitch]; 
        command[5][Actuators::L_SHOULDER_ROLL][index]  = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LShoulderRoll]; 
        command[5][Actuators::R_SHOULDER_PITCH][index] = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RShoulderPitch];
        command[5][Actuators::R_SHOULDER_ROLL][index]  = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RShoulderRoll];
        command[5][Actuators::HIP_PITCH][index]        = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HipPitch]; 
        command[5][Actuators::HIP_ROLL][index]         = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HipRoll];
        command[5][Actuators::KNEE_PITCH][index]       = wb_command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::KneePitch];
        // wheels                                                                                                                           
        command[5][Actuators::JOINTS_NUM + Actuators::WHEEL_FR][index] = wb_command.wheel_velocities_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::WHEEL_FRONT_RIGHT];
        command[5][Actuators::JOINTS_NUM + Actuators::WHEEL_FL][index] = wb_command.wheel_velocities_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::WHEEL_FRONT_LEFT]; 
        command[5][Actuators::JOINTS_NUM + Actuators::WHEEL_B][index]  = wb_command.wheel_velocities_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::WHEEL_BACK];
    }


    /**
     * @brief Send whole-body control command
     *
     * @param[in] dcm_current_time_ms
     * @param[in] wb_command
     */
    void PepperController::sendWBCCommand(const int dcm_current_time_ms, const humoto::pepper_ik::RobotCommand& wb_command)
    {
        try
        {
            AL::ALValue command = prepareActuatorCommand();
            command[4][0] = dcm_current_time_ms + static_cast<int>(wb_controller_.getWBCParameters().control_interval_ms_);
            copyCommand(command, wb_command); 
            dcm_proxy_->setAlias(command);
        }
        catch(const AL::ALError& e)
        {
            throw ALERROR(getName(), __FUNCTION__, "cannot send command");
        }
    }


    /**
     * @brief Send vector of whole-body control commands
     *
     * @param[in] dcm_current_time_ms
     * @param[in] wb_command
     */
    void PepperController::sendWBCCommand(const int dcm_current_time_ms, const std::vector<humoto::pepper_ik::RobotCommand>& wb_command)
    {
        try
        {
            AL::ALValue command = prepareActuatorCommand(wb_command.size());
            for(int i = 0; i < wb_command.size(); ++i)
            {
                command[4][i] = dcm_current_time_ms + (i + 1)*static_cast<int>(wb_controller_.getWBCParameters().control_interval_ms_);
                copyCommand(command, wb_command[i], i); 
            }
            
            dcm_proxy_->setAlias(command);
        }
        catch(const AL::ALError& e)
        {
            throw ALERROR(getName(), __FUNCTION__, "cannot send command");
        }
    }


    /**
     * @brief Disconnect callback from DCM loop
     */
    void PepperController::disconnectFromDCMLoop()
    {
        // remove the postProcess call back connection
        DCM_connection_.disconnect();
    }


    /**
     * @brief Read motion from file
     *
     * @param[in, out] result
     * @param[in]      filename
     */
    void PepperController::readMotionFromFile(std::vector<std::vector<float> >& result, const char* filename) const
    {
        std::ifstream input_file((config_path_ + filename).c_str());
        if(!input_file)
        {
            throw ALERROR(getName(), __FUNCTION__, "Couldn't open the file.");
        }

        std::string line;
        while(std::getline(input_file, line))
        {
            std::istringstream stream(line);
            std::vector<float> inner_vector;

            float value;
            while(stream >> value)
            {
                inner_vector.push_back(value);
            }

            result.push_back(inner_vector);
        }
    }
   

    /**
     * @brief Remove ALMotion module from naoqi
     */
    void PepperController::killALMotionModule()
    {
        qiLogInfo("Unregistering module ALMotion");
        
        bool is_motion_running = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("ALMotion"));
        if(is_motion_running)
        {
            qiLogInfo("Unregistering module ALMotion");
            motion_proxy_ = getParentBroker()->getProxy("ALMotion");
            motion_proxy_->callVoid("exit");
        }
        else
        {
            qiLogInfo("ALMotion already unregistered");
        }
    }


    /**
     * @brief Initialize the controller
     */
    void PepperController::init()
    {
        /**
         * Init is called just after construction.
         * Do something or not
         */
        qiLogInfo("module.PepperController", "Execution of init() function.");

        bool is_DCM_running;
        try
        {
            is_DCM_running = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("DCM"));

            if(!is_DCM_running)
            {
                throw ALERROR(getName(), __FUNCTION__, "DCM not running");
            }
            
            dcm_proxy_    = getParentBroker()->getDcmProxy();
            memory_proxy_ = getParentBroker()->getMemoryProxy();
        
        }
        catch (AL::ALError& e)
        {
            throw ALERROR(getName(), __FUNCTION__, std::string(e.what()));
        }

        // init joints
        actuators_names_[Actuators::HEAD_PITCH]       += "HeadPitch";
        actuators_names_[Actuators::HEAD_YAW]         += "HeadYaw";
        actuators_names_[Actuators::L_ELBOW_ROLL]     += "LElbowRoll";
        actuators_names_[Actuators::L_ELBOW_YAW]      += "LElbowYaw";
        actuators_names_[Actuators::R_ELBOW_ROLL]     += "RElbowRoll";
        actuators_names_[Actuators::R_ELBOW_YAW]      += "RElbowYaw";
        actuators_names_[Actuators::L_WRIST_YAW]      += "LWristYaw";
        actuators_names_[Actuators::R_WRIST_YAW]      += "RWristYaw";
        actuators_names_[Actuators::L_SHOULDER_PITCH] += "LShoulderPitch";
        actuators_names_[Actuators::L_SHOULDER_ROLL]  += "LShoulderRoll";
        actuators_names_[Actuators::R_SHOULDER_PITCH] += "RShoulderPitch";
        actuators_names_[Actuators::R_SHOULDER_ROLL]  += "RShoulderRoll";
        actuators_names_[Actuators::HIP_PITCH]        += "HipPitch";
        actuators_names_[Actuators::HIP_ROLL]         += "HipRoll";
        actuators_names_[Actuators::KNEE_PITCH]       += "KneePitch";
        
        actuators_names_[Actuators::JOINTS_NUM + Actuators::WHEEL_FR] += "WheelFR";
        actuators_names_[Actuators::JOINTS_NUM + Actuators::WHEEL_FL] += "WheelFL";
        actuators_names_[Actuators::JOINTS_NUM + Actuators::WHEEL_B]  += "WheelB";
        
        initFastMemoryAccess();
        createHardnessActuatorsAlias();
        createHardnessWheelsAlias();
        createHardnessUpperJointsAlias();
        createPositionActuatorAlias();    
    }


    /**
     * @brief Send whole-body control command with time offset 
     *
     * @param[in] time_offset
     * @param[in] wb_command
     */
    void PepperController::sendWBCCommandWithOffset(const int time_offset, const humoto::pepper_ik::RobotCommand& wb_command)
    {
        try
        {
            AL::ALValue command = prepareActuatorCommand();
            command[4][0] = dcm_proxy_->getTime(time_offset);
            copyCommand(command, wb_command); 
            dcm_proxy_->setAlias(command);
        }
        catch(const AL::ALError& e)
        {
            throw ALERROR(getName(), __FUNCTION__, "cannot send command");
        }
        
        qi::os::msleep(time_offset);
        qiLogInfo("module.PepperController", "Execution of sendWBCCommandWithOffset() is finished.");
    }


    /**
     * @brief Set actuators
     *
     * @param[in] time_offset
     * @param[in] values
     */
    void PepperController::setActuators(const std::size_t time_offset, const AL::ALValue& values)
    {
        if(!values.isArray())
        {
            throw ALERROR(getName(), __FUNCTION__, "array argument needed.");
        }
        else
        {
            if(values.getSize() != Actuators::ACTUATORS_NUM)
            {
                throw ALERROR(getName(), __FUNCTION__, "wrong actuators array size.");
            }
        }

        try
        {
            AL::ALValue command = prepareActuatorCommand();
            command[4][0] = dcm_proxy_->getTime(time_offset);
            
            for (std::size_t i = 0; i < Actuators::ACTUATORS_NUM; ++i)
            {
                command[5][i][0] = values[i];
            }
            
            dcm_proxy_->setAlias(command);
        }
        catch (const AL::ALError& e)
        {
            throw ALERROR(getName(), __FUNCTION__, "cannot send command");
        }
     
        qi::os::msleep(time_offset);
        qiLogInfo("module.PepperController", "Execution of setActuators() is finished.");
    }


    /**
     * @brief Prepare command to send to actuators
     *
     * @param[in] n_commands
     * 
     * @return command
     */
    AL::ALValue PepperController::prepareActuatorCommand(const std::size_t n_commands) const
    {
        AL::ALValue command;

        command.arraySetSize(6);
        command[0] = std::string("jointActuator");
        command[1] = std::string("ClearAll");
        command[2] = std::string("time-separate");
        command[3] = 0;

        // commands[4][0]  Will be the new time
        command[4].arraySetSize(n_commands);

        // for all joints
        command[5].arraySetSize(Actuators::ACTUATORS_NUM);

        for(std::size_t i = 0; i < Actuators::ACTUATORS_NUM; ++i)
        {
            // commands[5][i][0] will be the new angle
            command[5][i].arraySetSize(n_commands);
        }

        return(command);
    }


    /**
     * @brief Create alias for position commands for actuators
     */
    void PepperController::createPositionActuatorAlias()
    {
        AL::ALValue actuators_aliases;
       
        actuators_aliases.clear();
        actuators_aliases.arraySetSize(2);
        
        // alias for all joint actuators
        actuators_aliases[0] = std::string("jointActuator");
        actuators_aliases[1].arraySetSize(Actuators::ACTUATORS_NUM);
        
        try
        {
            for(std::size_t i = 0; i < Actuators::JOINTS_NUM; ++i)
            {
                actuators_aliases[1][i] = actuators_names_[i] + "/Position/Actuator/Value";      
            }
            // wheels
            for(std::size_t i = 0; i < Actuators::WHEELS_NUM; ++i)
            {
                actuators_aliases[1][Actuators::JOINTS_NUM + i] = actuators_names_[Actuators::JOINTS_NUM + i] + "/Speed/Actuator/Value";      
            }
           
            // create alias
            dcm_proxy_->createAlias(actuators_aliases);
        }
        catch (const AL::ALError &e)
        {
            throw ALERROR(getName(), "createPositionActuatorAlias()", "Error when creating Alias : " + e.toString());
        }
    }


    /**
     * @brief Create alias for stiffness commands for actuators
     */
    void PepperController::createHardnessActuatorsAlias()
    {
        AL::ALValue actuators_aliases;
       
        actuators_aliases.clear();
        actuators_aliases.arraySetSize(2);
        
        // alias for all joint actuators
        actuators_aliases[0] = std::string("actuatorsStiffness");
        actuators_aliases[1].arraySetSize(Actuators::ACTUATORS_NUM);
        
        try
        {
            for(std::size_t i = 0; i < Actuators::JOINTS_NUM; ++i)
            {
                actuators_aliases[1][i] = actuators_names_[i] + "/Hardness/Actuator/Value";      
            }
            // wheels
            for(std::size_t i = 0; i < Actuators::WHEELS_NUM; ++i)
            {
                actuators_aliases[1][Actuators::JOINTS_NUM + i] = actuators_names_[Actuators::JOINTS_NUM + i] + "/Stiffness/Actuator/Value";      
            }
            
            // create alias
            dcm_proxy_->createAlias(actuators_aliases);
        }
        catch (const AL::ALError &e)
        {
            throw ALERROR(getName(), "createHardnessActuatorsAlias()", "Error when creating Alias : " + e.toString());
        }
    }


    /**
     * @brief Create alias for stiffness commands for wheels actuators
     */
    void PepperController::createHardnessWheelsAlias()
    {
        AL::ALValue actuators_aliases;
       
        actuators_aliases.clear();
        actuators_aliases.arraySetSize(2);
        
        // alias for all joint actuators
        actuators_aliases[0] = std::string("wheelsStiffness");
        actuators_aliases[1].arraySetSize(Actuators::WHEELS_NUM);
        
        try
        {
            for(std::size_t i = 0; i < Actuators::WHEELS_NUM; ++i)
            {
                actuators_aliases[1][i] = actuators_names_[Actuators::JOINTS_NUM + i] + "/Stiffness/Actuator/Value";      
            }
            
            // create alias
            dcm_proxy_->createAlias(actuators_aliases);
        }
        catch (const AL::ALError &e)
        {
            throw ALERROR(getName(), "createHardnessWheelsAlias()", "Error when creating Alias : " + e.toString());
        }
    }


    /**
     * @brief Create alias for stiffness commands for upper joints actuators
     */
    void PepperController::createHardnessUpperJointsAlias()
    {
        AL::ALValue actuators_aliases;
       
        actuators_aliases.clear();
        actuators_aliases.arraySetSize(2);
        
        // alias for all joint actuators
        actuators_aliases[0] = std::string("upperJointsStiffness");
        actuators_aliases[1].arraySetSize(Actuators::JOINTS_NUM);
        
        try
        {
            for(std::size_t i = 0; i < Actuators::JOINTS_NUM; ++i)
            {
                actuators_aliases[1][i] = actuators_names_[i] + "/Hardness/Actuator/Value";      
            }
            
            // create alias
            dcm_proxy_->createAlias(actuators_aliases);
        }
        catch (const AL::ALError &e)
        {
            throw ALERROR(getName(), "createHardnessUpperJointsAlias()", "Error when creating Alias : " + e.toString());
        }
    }


    /**
     * @brief Initialize fast memory access
     */
    void PepperController::initFastMemoryAccess()
    {
        std::vector<std::string> actuators_sensors_keys;
        actuators_sensors_keys.resize(Actuators::ACTUATORS_NUM);
        
        for(std::size_t i = 0; i < Actuators::JOINTS_NUM; ++i)
        {
            actuators_sensors_keys[i] = actuators_names_[i] + "/Position/Sensor/Value";      
        }
        // wheels
        for(std::size_t i = 0; i < Actuators::WHEELS_NUM; ++i)
        {
            actuators_sensors_keys[Actuators::JOINTS_NUM + i] = actuators_names_[Actuators::JOINTS_NUM + i] + "/Speed/Sensor/Value";      
        }

        // create the fast memory access
        memory_fast_access_->ConnectToVariables(getParentBroker(), actuators_sensors_keys, false);
        dcm_current_time_ptr_ms_ = static_cast<int*>(memory_proxy_->getDataPtr("DCM/Time"));
    }


    AL::ALValue PepperController::prepareStiffnessCommand(const float stiffness_value, const std::string& name) const
    {
        if ((stiffness_value < 0.) || (stiffness_value > 1.))
        {
            throw ALERROR(getName(), __FUNCTION__, "wrong stiffness");
        }
        
        AL::ALValue command;

        command.arraySetSize(3);
        command[0] = std::string(name);
        command[1] = std::string("Merge");
        command[2].arraySetSize(1);
        command[2][0].arraySetSize(2);
        command[2][0][0] = stiffness_value;

        return(command);
    }

    /**
     * @brief Set stiffness of wheels
     *
     * @param[in] time_offset
     * @param[in] stiffness_value
     */
    void PepperController::setWheelsStiffness(const std::size_t time_offset, const float stiffness_value)
    {
        try
        {
            AL::ALValue command = prepareStiffnessCommand(stiffness_value, "wheelsStiffness");
            command[2][0][1] = dcm_proxy_->getTime(time_offset);
            dcm_proxy_->set(command);
        }
        catch (const AL::ALError &e)
        {
            throw ALERROR(getName(), __FUNCTION__, "cannot send stiffness command");
        }
        
        qi::os::msleep(time_offset);
        qiLogInfo ("module.PepperController", "Execution of setWheelsStiffness() is finished.");
    }


    /**
     * @brief Set stiffness of upper joints 
     *
     * @param[in] time_offset
     * @param[in] stiffness_value
     */
    void PepperController::setUpperJointsStiffness(const std::size_t time_offset, const float stiffness_value)
    {
        try
        {
            AL::ALValue command = prepareStiffnessCommand(stiffness_value, "upperJointsStiffness");
            command[2][0][1] = dcm_proxy_->getTime(time_offset);
            dcm_proxy_->set(command);
        }
        catch (const AL::ALError &e)
        {
            throw ALERROR(getName(), __FUNCTION__, "cannot send stiffness command");
        }
        
        qi::os::msleep(time_offset);
        qiLogInfo ("module.PepperController", "Execution of setUpperJointsStiffness() is finished.");
    }


    /**
     * @brief Set stiffness of actuators
     *
     * @param[in] time_offset
     * @param[in] stiffness_value
     */
    void PepperController::setActuatorsStiffness(const std::size_t time_offset, const float stiffness_value)
    {
        try
        {
            AL::ALValue command = prepareStiffnessCommand(stiffness_value, "actuatorsStiffness");
            command[2][0][1] = dcm_proxy_->getTime(time_offset);
            dcm_proxy_->set(command);
        }
        catch (const AL::ALError &e)
        {
            throw ALERROR(getName(), __FUNCTION__, "cannot send stiffness command");
        }
        
        qi::os::msleep(time_offset);
        qiLogInfo ("module.PepperController", "Execution of setActuatorsStiffness() is finished.");
    }
} //pepper_controller
