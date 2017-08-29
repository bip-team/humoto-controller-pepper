cmake_minimum_required(VERSION 2.8.3)

project(controller-pepper)

find_package(qibuild)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
include(pepper_controller OPTIONAL)

set(CONTROLLER_BASE_MOTION_VISION_ENABLED  OFF          CACHE BOOL   "")
set(CONTROLLER_MPC_HOTSTARTING_ENABLED     OFF          CACHE BOOL   "")
set(CONTROLLER_LOGGING_ENABLED             OFF          CACHE BOOL   "")
set(CONTROLLER_HUMOTO_MPC_SOLVER_NAMESPACE "quadprogpp" CACHE STRING "")
set(CONTROLLER_HUMOTO_IK_SOLVER_NAMESPACE  "kktsolver"  CACHE STRING "")

include(ExternalProject)

set(CONTROLLER_HUMOTO_DIR         "${PROJECT_SOURCE_DIR}/humoto")
set(CONTROLLER_HUMOTO_INSTALL_DIR "${PROJECT_BINARY_DIR}/humoto_install")
set(CONTROLLER_HUMOTO_PEPPER_CONFIG_PATH "${CONTROLLER_HUMOTO_INSTALL_DIR}/share/humoto/config/pepper")

if(CONTROLLER_MPC_HOTSTARTING_ENABLED AND CONTROLLER_HUMOTO_MPC_SOLVER_NAMESPACE STREQUAL "quadprogpp") 
    message(FATAL_ERROR "Hotstarting not available for quadprogpp solver. Aborting." )
endif()

configure_file("${PROJECT_SOURCE_DIR}/cmake/pepper_controller_config.cmake.in"
               "${PROJECT_SOURCE_DIR}/controller/pepper_controller_config.h")

#hack to retrieve cross-toolchain dir
get_filename_component(CONTROLLER_HUMOTO_QI_TOOLCHAIN_DIR ${ALDE_CTC_CROSS} DIRECTORY)

set(CONTROLLER_HUMOTO_CMAKE_ARGS
    "-C${PROJECT_SOURCE_DIR}/cmake/humoto.cmake"
    -DCMAKE_VERBOSE_MAKEFILE=ON
    -DCMAKE_BUILD_TYPE=Release
    "-DHUMOTO_QI_TOOLCHAIN_DIR=${CONTROLLER_HUMOTO_QI_TOOLCHAIN_DIR}"
    "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
    "-DCMAKE_INSTALL_PREFIX=${CONTROLLER_HUMOTO_INSTALL_DIR}"
    "-DCMAKE_CXX_FLAGS=-fPIC -Wno-error=conversion -Wno-error=unused-parameter -Wno-unused-parameter")

include(CMakeParseArguments)
include(humoto_add_external_git_project)
humoto_add_external_git_project(TARGET_NAME "humoto"
                                PROJECT_DIR "${CONTROLLER_HUMOTO_DIR}"
                                CMAKE_ARGS   ${CONTROLLER_HUMOTO_CMAKE_ARGS})

set(CONTROLLER_INSTALL_DIR "${PROJECT_BINARY_DIR}/controller_install/")
set(CONTROLLER_CMAKE_ARGS
    -DCMAKE_VERBOSE_MAKEFILE=ON
    -DCMAKE_BUILD_TYPE=Release
    "-DHUMOTO_INSTALL_DIR=${CONTROLLER_HUMOTO_INSTALL_DIR}"
    "-DCMAKE_INSTALL_PREFIX=${CONTROLLER_INSTALL_DIR}"
    "-DCMAKE_CXX_FLAGS=-fPIC -Wno-error=conversion -Wno-error=unused-parameter -Wno-unused-parameter")

ExternalProject_Add(
    controller
    SOURCE_DIR   "${PROJECT_SOURCE_DIR}/controller/"
    DOWNLOAD_DIR "${PROJECT_SOURCE_DIR}/controller/"
    TMP_DIR      "${PROJECT_BINARY_DIR}/controller/tmp"
    STAMP_DIR    "${PROJECT_BINARY_DIR}/controller/stamp"
    BINARY_DIR   "${PROJECT_SOURCE_DIR}/controller/"
    INSTALL_DIR  "${PROJECT_SOURCE_DIR}/controller/"
    CONFIGURE_COMMAND qibuild configure -v ${CONTROLLER_CMAKE_ARGS}
    BUILD_COMMAND qibuild make -v
    INSTALL_COMMAND ""
)

add_dependencies(controller humoto)
