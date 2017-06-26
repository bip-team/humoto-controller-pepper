/**
    @file
    @author Jan Michalczyk 
    @copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <numeric>

#include <alerror/alerror.h>
#include <alcommon/albroker.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>
#include <almemoryfastaccess/almemoryfastaccess.h>

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
#include "humoto/humoto.h"
#include "humoto/kktsolver.h"
#include "humoto/pepper_ik.h"

#include "pepper_controller_config.h"

#include "humoto/pepper_mpc.h"

#define MODEL_FEATURES humoto::pepper_ik::ModelFeatures::FIXED_WHEELS | humoto::pepper_ik::ModelFeatures::ROOT_PLANAR

#include "whole_body_control.h"
#include "mpc_control.h"
#include "pepper_controller_parameters.h"

#ifdef CONTROLLER_LOGGING_ENABLED
    #include "pepper_controller_logger.h"
#endif
