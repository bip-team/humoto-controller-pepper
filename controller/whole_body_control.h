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
     * @brief Whole-body controller for pepper
     */
    class HUMOTO_LOCAL WBController
    {
        public:
            /**
             * @brief Constructor
             *
             * @param[in] config_path
             */
            WBController(const std::string& config_path) : config_reader_          (config_path + "pepper-controller-config.yaml"),
                                                           wbc_parameters_         (config_reader_, true), 
                                                           motion_parameters_      (config_reader_, true, "IKMotionParameters"),
                                                           generalized_coordinates_(config_path + "initial-state-pepper-ik-planar.yaml", true)
            {
                opt_problem_.readConfig(config_reader_, true, "IKOptimizationProblem");
                //solver_parameters_.solution_method_ = humoto::kktsolver::SolverParameters::CONSTRAINT_ELIMINATION_LLT;
                solver_.setParameters(solver_parameters_);
                model_.loadParameters(config_path + "pepper_fixedwheels_roottibia_planar.urdf");
                model_.updateState(generalized_coordinates_);
                
                tag_angular_velocity_.setZero();
            }


            /**
             * @brief Get whole-body motion commands
             *
             * @param[in,out] commands
             * @param[in,out] mpc_time_instants
             * @param[in]     motion_parameters_vector 
             */
            void getWBMotionCommands(std::vector<humoto::pepper_ik::RobotCommand>& commands,
                                     std::vector<double>& mpc_time_instants, 
                                     const std::vector<humoto::pepper_ik::MotionParameters>&  motion_parameters_vector) 
            {
                std::adjacent_difference(mpc_time_instants.begin(), mpc_time_instants.end(), mpc_time_instants.begin());
                
                // angular velocity of a tag
                wbc_.setTagRefAngularVelocity(tag_angular_velocity_);
                
                qiLogInfo("TagAngularVelocity: ") << tag_angular_velocity_[0] << " " << tag_angular_velocity_[1] << " "
                << tag_angular_velocity_[2] << std::endl;
                
                decayTagAngularVelocity(0.3);
                
                commands.resize(mpc_time_instants.size());
                for(std::size_t i = 0; i < mpc_time_instants.size(); ++i)
                {
                    model_.saveCurrentState();
                    for(std::size_t j = 0;; ++j)
                    {
                        if(j == wbc_parameters_.maximal_number_of_iterations_)
                        {
                            HUMOTO_THROW_MSG("Maximal number of IK iterations reached.");
                        }
                        
                        // prepare control problem for new iteration
                        if(wbc_.update(model_, motion_parameters_vector[i]) != humoto::ControlProblemStatus::OK)
                        {
                            HUMOTO_THROW_MSG("Control problem could not be updated.");
                        }

                        // form an optimization problem
                        opt_problem_.form(solution_, model_, wbc_);

                        // solve an optimization problem
                        solver_.solve(solution_, opt_problem_);

                        // extract next model state from the solution and update model
                        generalized_coordinates_ = wbc_.getNextGeneralizedCoordinates(solution_, model_);
                        model_.updateState(generalized_coordinates_);
                        
                        humoto::pepper_ik::MotionParameters motion_parameters_errors;
                        model_.getStateError(motion_parameters_errors, motion_parameters_vector[i]);

                        if(motion_parameters_errors.base_com_position_.norm()
                            + motion_parameters_errors.base_orientation_rpy_.norm()
                            + motion_parameters_errors.body_com_position_.norm() < wbc_parameters_.motion_parameters_tolerance_)
                        {
                            break;
                        }

                        if(solution_.x_.lpNorm<Eigen::Infinity>() < wbc_parameters_.joint_angle_error_tolerance_)
                        {
                            break;
                        }
                    }

                    if(i == 0)
                    {
                        generalized_coords_to_update_ = generalized_coordinates_;
                    }
                    
                    model_.getRobotCommand(commands[i], mpc_time_instants[i]);
                }
            }


            /**
             * @brief Decay tag angular velocity
             *
             * @param[in] decay_factor
             */
            void decayTagAngularVelocity(const double decay_factor)    
            {
                 tag_angular_velocity_ = decay_factor * tag_angular_velocity_;
            }
            

            /**
             * @brief Set tag angular velocity
             *
             * @param[in] tag_angular_velocity
             */
            void setTagAngularVelocity(const std::vector<double>& tag_angular_velocity)
            {
                if(tag_angular_velocity.size() != tag_angular_velocity_.size())
                {
                    HUMOTO_THROW_MSG("Velocity vector sizes must match.");
                }
                
                for(std::size_t i = 0; i < tag_angular_velocity.size(); ++i)
                {
                    tag_angular_velocity_(i) = tag_angular_velocity[i];
                }

            }


            /**
             * @brief Get whole-body model
             *
             * @return Model
             */
            const humoto::pepper_ik::Model<MODEL_FEATURES>& getModel() const
            {
                return(model_);
            }
            
            
            /**
             * @brief Get whole-body model
             *
             * @return Model
             */
            humoto::pepper_ik::Model<MODEL_FEATURES>& getModel()
            {
                return(model_);
            }


            /**
             * @brief Get whole-body controller parameters
             *
             * @return Whole-body controller parameters
             */
            const humoto::pepper_ik::WBCParameters& getWBCParameters() const
            {
                return(wbc_parameters_);
            }
            
            
            /**
             * @brief Get whole-body controller parameters
             *
             * @return Whole-body controller parameters
             */
            humoto::pepper_ik::WBCParameters& getWBCParameters()
            {
                return(wbc_parameters_);
            }
            

            /**
             * @brief Get generalized coordinates of the whole-body model
             *
             * @return generalized coordinates
             */
            const humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>& getModelStateToUpdate() const
            {
                return(generalized_coords_to_update_);
            }


        private:
            humoto::config::Reader              config_reader_;
            // optimization problem (a stack of tasks / hierarchy)
            humoto::pepper_ik::ConfigurableOptimizationProblem<MODEL_FEATURES>  opt_problem_;

            // parameters of the solver
            humoto::kktsolver::SolverParameters solver_parameters_;
            // a solver which is giong to be used
            humoto::kktsolver::Solver           solver_;
            // solution
            humoto::Solution           solution_;
            humoto::Solution           solution_guess_;

            // parameters of the control problem
            humoto::pepper_ik::WBCParameters                           wbc_parameters_;
            // control problem, which is used to construct an optimization problem
            humoto::pepper_ik::WholeBodyController<MODEL_FEATURES>     wbc_;

            // model representing the controlled system
            humoto::pepper_ik::Model<MODEL_FEATURES>                   model_;

            // options for walking
            humoto::pepper_ik::MotionParameters                        motion_parameters_;

            humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>  generalized_coordinates_;
            humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>  generalized_coords_to_update_;
            
            // angular velocity of the tag
            etools::Vector3                                            tag_angular_velocity_;
    };
} //pepper_controller
