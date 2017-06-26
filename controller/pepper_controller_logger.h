/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace pepper_controller
{
    class HUMOTO_LOCAL PepperControllerLogger
    {
        /**
         * @brief Pepper controller logger class
         */
        public:
            /**
             * @brief Constructor
             *
             * @param[in] filename
             */
            PepperControllerLogger(const std::string& filename) : logger_(filename)
            {
                reset();
            }


            /**
             * @brief Log wheels velocities
             *
             * @param[in] sensor_readings
             * @param[in] wb_commands
             * @param[in] iter
             */
            void logWheelsVelocities(const std::vector<float>& sensor_readings,
                                     const humoto::pepper_ik::RobotCommand& wb_command,
                                     const std::size_t iter)
            {
                std::size_t wheels_number = 3;
                std::vector<float> wheels_velocities(sensor_readings.end() - wheels_number,
                                                     sensor_readings.end());
                humoto::LogEntryName prefix = humoto::LogEntryName("control_loop").add(iter);
                logger_.log(humoto::LogEntryName(prefix).add("sensor_velocities"), wheels_velocities);
                
                wb_command.log(logger_, prefix);
            }


            /**
             * @brief Log values for simulation
             *
             * @param[in] model
             * @param[in] wb_commands
             */
            void logForSimulation(humoto::pepper_ik::Model<MODEL_FEATURES>& model,
                                  const humoto::pepper_ik::RobotCommand& wb_command)
            {
                humoto::LogEntryName prefix = humoto::LogEntryName("control_loop").add(sim_loop_counter_);
                wb_command.log(logger_, prefix);
                model.getTorsoRootPose().log(logger_, prefix, "torso_root_pose");

                sim_loop_counter_++;
            }
            
            
            /**
             * @brief Log values for simulation
             *
             * @param[in] value 
             * @param[in] description
             * @param[in] iter
             */
            template <typename T>
            void logValue(const T value, const std::string& description, const std::size_t iter)
            {
                humoto::LogEntryName prefix = humoto::LogEntryName("control_loop").add(iter);
                logger_.log(prefix.add(description), value);
            }
            
            
            /**
             * @brief Log values for simulation
             *
             * @param[in] model_from
             * @param[in] model_to
             * @param[in] execution_time_ms
             * @param[in] timestep_ms
             */
            void logForSimulation(humoto::pepper_ik::Model<MODEL_FEATURES>& model_from,
                                  humoto::pepper_ik::Model<MODEL_FEATURES>& model_to,
                                  const std::size_t execution_time_ms, 
                                  const std::size_t timestep_ms)
            {
                std::size_t n_of_iters = execution_time_ms / timestep_ms;

                Eigen::MatrixXd joints;
                interpolateLinear(joints, model_from.getState().joint_angles_,
                                  model_to.getState().joint_angles_, n_of_iters);

                Eigen::MatrixXd torso_position;
                interpolateLinear(torso_position, model_from.getTorsoRootPose().position_,
                                  model_to.getTorsoRootPose().position_, n_of_iters);
                
                Eigen::MatrixXd torso_rpy;
                interpolateLinear(torso_rpy, model_from.getTorsoRootPose().rpy_,
                                  model_to.getTorsoRootPose().rpy_, n_of_iters);

                logRobotPose(joints, torso_position, torso_rpy, n_of_iters);
                
                sim_loop_counter_ += n_of_iters;
            }
            
            
            /**
             * @brief Get humoto logger
             *
             * @return humoto logger
             */
            const humoto::Logger& getHumotoLogger() const
            {
                return(logger_);
            }
            

        private:
            /**
             * @brief Reset counter
             */
            void reset()
            {
                sim_loop_counter_ = 0;
            }

            
            /**
             * @brief Log robot pose
             *
             * @param[in] joints
             * @param[in] torso_position
             * @param[in] torso_rpy
             * @param[in] n_of_iters
             */
            void logRobotPose(Eigen::MatrixXd& joints,
                              Eigen::MatrixXd& torso_position,
                              Eigen::MatrixXd& torso_rpy, 
                              const std::size_t n_of_iters)
            {
                for(std::size_t i = 0; i < n_of_iters; ++i)
                {
                    humoto::LogEntryName prefix1 = humoto::LogEntryName("control_loop").add(sim_loop_counter_ + i);
                    humoto::LogEntryName prefix2 = prefix1;
                    humoto::LogEntryName prefix3 = prefix1;
                    
                    logger_.log(prefix1.add("robot_command").add("joint_angles"), joints.col(i));
                    logger_.log(prefix2.add("torso_root_pose").add("position"),   torso_position.col(i));
                    logger_.log(prefix3.add("torso_root_pose").add("rpy")     ,   torso_rpy.col(i));
                }
            }


            /**
             * @brief Interpolate linearly between two values
             *
             * @param[in]      values_from
             * @param[in]      values_to
             * @param[in, out] result
             * @param[in]      n_of_iters
             */
            void interpolateLinear(Eigen::MatrixXd&       result,
                                   const Eigen::VectorXd& values_from,
                                   const Eigen::VectorXd& values_to,
                                   const std::size_t n_of_iters)
            {
                if(values_from.size() != values_to.size())
                {    
                    HUMOTO_THROW_MSG("Vectors must be of same size.");
                }
                
                result.resize(values_to.size(), n_of_iters); 
                for(std::size_t i = 0; i < values_to.size(); ++i)
                {
                    result.row(i) = Eigen::VectorXd::LinSpaced(n_of_iters, values_from(i), values_to(i));
                }
            }


        private:
            std::size_t    sim_loop_counter_;
            humoto::Logger logger_;
    };
} //pepper_controller
