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
    /**
     * @brief MPC controller
     */
    class HUMOTO_LOCAL MPCController
    {
        public:
            /**
             * @brief Construct and initialize mpc controller parameters
             *
             * @param[in] config_path
             */
            MPCController(const std::string& config_path) : config_reader_   (config_path + "pepper-controller-config.yaml"),
                                                            solver_          (solver_parameters_  ),
                                                            robot_parameters_(config_reader_, true),
                                                            model_           (robot_parameters_   ),
                                                            mg_parameters_   (config_reader_, true),
                                                            mg_              (mg_parameters_      )
            {
                bool crash_on_missing_entry = true;
                opt_problem_.readConfig<humoto::config::yaml::Reader>(config_reader_, "MPCOptimizationProblem", crash_on_missing_entry);
                mpc_motion_parameters_.readConfig<humoto::config::yaml::Reader>(config_reader_, "MPCMotionParameters", crash_on_missing_entry);
                // MotionParameters for unlimited motion
                readMPCMotionParametersIdle();
            }


            /**
             * @brief Read and set idle mpc motion parameters
             */
            void readMPCMotionParametersIdle()
            {
                humoto::pepper_mpc::MotionParameters mpc_motion_parameters;
                mpc_motion_parameters.setIdle();
                mpc_motion_parameters_deque_.clear();
                mpc_motion_parameters_deque_.push_back(mpc_motion_parameters);
            }
            
            
            /**
             * @brief Read and set mpc motion parameters
             *
             * @param[in] filename
             */
            void readMPCMotionParameters(const std::string& filename)
            {
                humoto::config::yaml::Reader config_reader(filename);
                bool crash_on_missing_entry = true;
                
                std::size_t n_of_parameters;
                config_reader.readScalar(n_of_parameters, "NumberOfParameters", crash_on_missing_entry);
                
                mpc_motion_parameters_deque_.clear();
                for(std::size_t i = 1; i < n_of_parameters + 1; ++i)
                {
                    std::ostringstream ss;
                    ss << i;

                    humoto::pepper_mpc::MotionParameters mpc_motion_parameters(config_reader,
                                                                           "MPCMotionParameters_" + ss.str(), 
                                                                           crash_on_missing_entry);
                    mpc_motion_parameters_deque_.push_back(mpc_motion_parameters);
                }

                // add idle state at the end
                humoto::pepper_mpc::MotionParameters mpc_motion_parameters;
                mpc_motion_parameters.setIdle();
                mpc_motion_parameters_deque_.push_back(mpc_motion_parameters);
            }


            /**
             * @brief Run MPC control loop
             *
             * @param[in, out] wb_controller
             * @param[in, out] ik_motion_parameters_vector
             * @param[in]      commands
             * @param[in]      sampling_interval_difference
             * @param[in]      mpc_time_instants 
             *
             * @return         success flag
             */
            bool run(WBController& wb_controller,
                     std::vector<humoto::pepper_ik::MotionParameters>& ik_motion_parameters_vector,
                     const std::vector<humoto::pepper_ik::RobotCommand>& commands,
                     const double sampling_interval_difference,
                     const std::vector<double>& mpc_time_instants)
            {

#ifdef CONTROLLER_BASE_MOTION_VISION_ENABLED
                etools::Vector6 base_velocity = wb_controller.getBaseVelocityInGlobal();
                mpc_motion_parameters_.base_velocity_         << base_velocity(3), base_velocity(4); 
                mpc_motion_parameters_.base_angular_velocity_ =  base_velocity(2);
#endif      

                // -----------------feedback--------------------------------
                if(!commands.empty())
                {
                    // in the perfect case
                    // sampling_interval_difference = 0.0
                    wb_controller.getModel().correct(wb_controller.getModelStateToUpdate(),
                                                     commands.back(),
                                                     sampling_interval_difference);

                    mg_.shift(mpc_motion_parameters_,
                              mg_parameters_.subsampling_time_ms_
                              + humoto::convertSecondToMillisecond(sampling_interval_difference));
                }
                // -----------------feedback--------------------------------
                
                // -----------------sync-models--------------------------------
                synchronizeModels(wb_controller);
                // -----------------sync-models--------------------------------

                // prepare control problem for new iteration
                humoto::ControlProblemStatus::Status control_status = mg_.update(mpc_motion_parameters_,
                                                                                 model_);
                
                if(control_status != humoto::ControlProblemStatus::OK)
                {
                    return(false);
                }

#ifdef CONTROLLER_MPC_HOTSTARTING_ENABLED                
                // form an optimization problem
                opt_problem_.form(solution_, solution_guess_, active_set_guess_, model_, mg_, old_solution_);
                // solve an optimization problem
                solver_.solve(solution_, active_set_actual_, opt_problem_, solution_guess_, active_set_guess_);
                opt_problem_.processActiveSet(active_set_actual_);
                old_solution_ = solution_;
#else                
                // form an optimization problem
                opt_problem_.form(solution_, model_, mg_);
                // solve an optimization problem
                solver_.solve(solution_, opt_problem_);
#endif

                mg_.parseSolution(solution_);

                for(std::size_t i = 0; i < mpc_time_instants.size(); ++i)
                {
                    humoto::pepper_mpc::ModelState model_state = mg_.getModelState(model_, mpc_time_instants[i]);
                    ik_motion_parameters_vector[i] = computeWBMotionParameters(model_state);
                    if(i == 0)
                    {
                        model_state_ = model_state;
                    }
                }

                return(true);
            }
            

            /**
             * @brief Synchronize mpc model to whole-body model
             *
             * @param[in] wb_controller
             */
            void synchronizeModels(const WBController& wb_controller)
            {
                model_state_.update(wb_controller.getModel().getBaseMass(),
                                    wb_controller.getModel().getBodyMass(),
                                    wb_controller.getModel().getBaseCoM(),
                                    wb_controller.getModel().getBodyCoM(), 
                                    wb_controller.getModel().getBaseYaw());
                model_.updateState(model_state_);
            }


            /**
             * @brief Get MPC parameters
             *
             * @return MPC control parameters 
             */
            const humoto::pepper_mpc::MPCParameters& getMPCParameters() const
            {
                return(mg_parameters_);
            }
            

        private:
            /**
             * @brief Compute whole-body motion parameters
             *
             * @parame[in] model_state
             *
             * @return     Inverse kinematics motion parameters
             */
            humoto::pepper_ik::MotionParameters computeWBMotionParameters(const humoto::pepper_mpc::ModelState& model_state)
            {
                humoto::pepper_ik::MotionParameters ik_motion_parameters;
                ik_motion_parameters.base_com_position_.x()    = model_state.base_state_.position_.x();
                ik_motion_parameters.base_com_position_.y()    = model_state.base_state_.position_.y();
                ik_motion_parameters.base_com_position_.z()    = model_state.base_state_.position_.z();
                ik_motion_parameters.base_orientation_rpy_.x() = model_state.base_state_.rpy_.x();
                ik_motion_parameters.base_orientation_rpy_.y() = model_state.base_state_.rpy_.y();
                ik_motion_parameters.base_orientation_rpy_.z() = model_state.base_state_.rpy_.z();
                ik_motion_parameters.body_com_position_.x()    = model_state.body_state_.position_.x();
                ik_motion_parameters.body_com_position_.y()    = model_state.body_state_.position_.y();
                ik_motion_parameters.body_com_position_.z()    = model_state.body_state_.position_.z();

                return(ik_motion_parameters);
            }


        private:
            humoto::config::yaml::Reader                        config_reader_;
            // optimization problem (a stack of tasks / hierarchy)
            humoto::pepper_mpc::ConfigurableOptimizationProblem opt_problem_;

            // parameters of the solver
            humoto::CONTROLLER_HUMOTO_MPC_SOLVER_NAMESPACE::SolverParameters solver_parameters_;
            // a solver which is giong to be used
            humoto::CONTROLLER_HUMOTO_MPC_SOLVER_NAMESPACE::Solver           solver_;
            // solution
            humoto::CONTROLLER_HUMOTO_MPC_SOLVER_NAMESPACE::Solution         solution_;

            humoto::pepper_mpc::RobotParameters         robot_parameters_;
            // model representing the controlled system
            humoto::pepper_mpc::Model                   model_;

            // parameters of the control problem
            humoto::pepper_mpc::MPCParameters           mg_parameters_;
            // control problem, which is used to construct an optimization problem
            humoto::pepper_mpc::MPCforMG                mg_;

            // options for motion
            humoto::pepper_mpc::MotionParameters        mpc_motion_parameters_;
            std::deque<humoto::pepper_mpc::MotionParameters>        mpc_motion_parameters_deque_;
            
            // state of the model
            humoto::pepper_mpc::ModelState                    model_state_;
           
#ifdef CONTROLLER_MPC_HOTSTARTING_ENABLED                
            // hotstarting
            humoto::ActiveSet                                        active_set_guess_;
            humoto::ActiveSet                                        active_set_actual_;
            humoto::CONTROLLER_HUMOTO_MPC_SOLVER_NAMESPACE::Solution old_solution_;
            humoto::Solution                                         solution_guess_;
#endif

    };
}//pepper_controller
