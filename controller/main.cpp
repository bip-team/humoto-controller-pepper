/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <boost/shared_ptr.hpp>

#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>

#include "pepper_controller.h"

// we're in a dll, so export the entry point
#ifdef _WIN32
# define ALCALL __declspec(dllexport)
#else
# define ALCALL
#endif

extern "C"
{
    /**
     * @brief Called when module opens
     */
    ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> broker)
    {
        // init broker with the main broker instance
        // from the parent executable
        AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
        AL::ALBrokerManager::getInstance()->addBroker(broker);
        // create module instances
        AL::ALModule::createModule<pepper_controller::PepperController>(broker, "PepperController");
        return 0;
    }

    /**
     * @brief Called when module exits
     */
    ALCALL int _closeModule(  )
    {
        return 0;
    }
} // extern "C"
