/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <alcommon/almodule.h>

#include "pepper_controller_includes.h"

/**
 * @brief Namespace AL
 */
namespace AL
{
  // This is a forward declaration of AL:ALBroker which
  // avoids including <alcommon/albroker.h> in this header
  class ALBroker;
  class ALMemoryProxy;
  class DCMProxy;
  class ALMemoryFastAccess;
  class ALProxy;
}

/**
 * @brief namespace of pepper controller
 */
namespace pepper_controller
{
    /**
     * @brief namespace of pepper actuators
     */
    namespace Actuators
    {
        /**
         * @brief Wheels IDs
         */
        enum WheelsID
        {
            WHEEL_FR         = 0, 
            WHEEL_FL         = 1, 
            WHEEL_B          = 2, 

            WHEELS_NUM       = 3
        };
        

        /**
         * @brief Joints IDs
         */
        enum JointsID
        {
            HEAD_PITCH       = 0,
            HEAD_YAW         = 1, 
            L_ELBOW_ROLL     = 2,
            L_ELBOW_YAW      = 3, 
            R_ELBOW_ROLL     = 4, 
            R_ELBOW_YAW      = 5, 
            L_WRIST_YAW      = 6, 
            R_WRIST_YAW      = 7, 
            L_SHOULDER_PITCH = 8, 
            L_SHOULDER_ROLL  = 9, 
            R_SHOULDER_PITCH = 10, 
            R_SHOULDER_ROLL  = 11, 
            HIP_PITCH        = 12, 
            HIP_ROLL         = 13, 
            KNEE_PITCH       = 14,

            JOINTS_NUM       = 15,
            
            ACTUATORS_NUM    = JOINTS_NUM + WHEELS_NUM
        };
    }


    /**
     * This class inherits AL::ALModule. This allows it to bind methods
     */
    class PepperController : public AL::ALModule
    {
        public:
            PepperController(boost::shared_ptr<AL::ALBroker> broker,
                             const std::string& name,
                             const std::string& config_path = "/home/nao/config-pepper/");

            virtual ~PepperController();

            /**
             * Overloading ALModule::init().
             * This is called right after the module has been loaded
             */
            virtual void init();

            // after that you may add all your bind method.
            void setActuators(const std::size_t time_offset, const AL::ALValue& values);
            void setActuatorsStiffness(const std::size_t time_offset, const float value);
            void setUpperJointsStiffness(const std::size_t time_offset, const float value);
            void setWheelsStiffness(const std::size_t time_offset, const float value);
            void setTagAngularVelocity(const std::vector<double>& angular_velocity);
            void startControl();
            void stopControl();
            void goInitialPose(const int execution_time_ms);
            void goRestPose(const int execution_time_ms);
            void setMPCMotionParameters(const std::string& filename);
            void setMPCMotionParametersIdle();
            void printRootPose();
            void killALMotionModule();

        private:
            void createPositionActuatorAlias();
            void createHardnessActuatorsAlias();
            void createHardnessWheelsAlias();
            void createHardnessUpperJointsAlias();
            void initFastMemoryAccess();
            
            AL::ALValue prepareActuatorCommand(const std::size_t = 1) const;
            AL::ALValue prepareStiffnessCommand(const float stiffness_value, const std::string& name) const;
            
            void stopRobotBase();
            void sendWBCCommand(const int dcm_current_time, const humoto::pepper_ik::RobotCommand& wb_command);
            void sendWBCCommand(const int dcm_current_time, const std::vector<humoto::pepper_ik::RobotCommand>& wb_command);
            void sendWBCCommandWithOffset(const int time_offset, const humoto::pepper_ik::RobotCommand& wb_command);
            void copyCommand(AL::ALValue& command, const humoto::pepper_ik::RobotCommand& wb_command, const std::size_t index = 0) const; 
            
            void synchronisedDCMCallback();
            void connectToDCMLoop();
            void disconnectFromDCMLoop();
            void reset();

            const std::vector<float> getSensorReadings();
            humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES> getCurrentGeneralizedCoordinatesFromSensors(); 

            void readMotionFromFile(std::vector<std::vector<float> >& result, const char* filename) const;

            std::vector<std::string>                  actuators_names_;
            std::vector<double>                       mpc_time_instants_;

            boost::shared_ptr<AL::DCMProxy>           dcm_proxy_;
            boost::shared_ptr<AL::ALMemoryProxy>      memory_proxy_;
            boost::shared_ptr<AL::ALMemoryFastAccess> memory_fast_access_;

            // ased for postProcess sync with the DCM
            ProcessSignalConnection                   DCM_connection_;

            boost::shared_ptr<AL::ALProxy>            motion_proxy_;

            std::size_t                               dcm_loop_counter_;
            std::string                               config_path_;
            int*                                      dcm_current_time_ptr_ms_;
            double                                    expected_control_drift_;
            double                                    dcm_call_time_interval_;

            // humoto whole-body controller
            WBController                                 wb_controller_;
            // humoto mpc controller
            MPCController                                mpc_controller_;
            // pepper controller parameters
            PepperControllerParameters                   pepper_controller_parameters_;
            // vector of whole-body control commands
            std::vector<humoto::pepper_ik::RobotCommand>     wb_commands_;
            std::vector<humoto::pepper_ik::MotionParameters> ik_motion_parameters_vector_;

#ifdef CONTROLLER_LOGGING_ENABLED
            PepperControllerLogger                       pepper_logger_;
#endif            

            humoto::Timer                                timer_control_;
            humoto::Timer                                timer_dcm_;
    };
} //pepper_controller
