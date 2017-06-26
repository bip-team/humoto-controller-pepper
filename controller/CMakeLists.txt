cmake_minimum_required(VERSION 2.8.3)

project(controller)

find_package(qibuild)

include_directories(SYSTEM "${HUMOTO_INSTALL_DIR}/include/")
include("${HUMOTO_INSTALL_DIR}/share/humoto/HumotoConfig.cmake")
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

set(_srcs 
    pepper_controller_logger.h
    mpc_control.h
    pepper_controller_includes.h
    pepper_controller_parameters.h
    whole_body_control.h
    pepper_controller.h
    pepper_controller.cpp
    pepper_controller_config.h
    main.cpp)

qi_create_lib(peppercontroller SHARED ${_srcs} SUBFOLDER naoqi)
qi_use_lib(peppercontroller ALCOMMON ALPROXIES EIGEN3 BOOST ALMEMORYFASTACCESS)
target_link_libraries(peppercontroller ${HUMOTO_LIBRARIES})
