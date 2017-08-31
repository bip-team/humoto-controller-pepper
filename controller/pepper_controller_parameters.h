/**
    @file
    @author  Jan Michalczyk
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace pepper_controller
{
    /**
     * @brief Stores parameters for pepper controller
     */
    class HUMOTO_LOCAL PepperControllerParameters : public humoto::config::ConfigurableBase
    {
            #define HUMOTO_CONFIG_SECTION_ID "PepperControllerParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR PepperControllerParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_SCALAR_(n_states_future) \
                HUMOTO_CONFIG_SCALAR_(dcm_iter_per_mpc)\
                HUMOTO_CONFIG_SCALAR_(log_for_simulation)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS
            
            
        public:
            std::size_t n_states_future_;
            std::size_t dcm_iter_per_mpc_;
            bool        log_for_simulation_;


        public:
            /**
             * @brief Default constructor
             */
            PepperControllerParameters()
            {
                setDefaults();
            }


            /**
             * @brief Default parameters of the pepper controller
             */
            void setDefaults()
            {
                n_states_future_        = 1;
                dcm_iter_per_mpc_       = 1;
                log_for_simulation_     = false;
            }


            /**
             * @brief Finalize parameters of the pepper controller
             */
            void finalize()
            {
                if((n_states_future_ > 2) || (n_states_future_ == 0))
                {
                    HUMOTO_THROW_MSG("One or two states in the future are supported.");
                }
            }
    };
} //pepper_controller
