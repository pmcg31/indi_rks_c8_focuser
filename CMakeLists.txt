# define the project name
project(indi-rks-c8-focuser C CXX)
cmake_minimum_required(VERSION 3.1)

include(GNUInstallDirs)

# add our cmake_modules folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# find our required packages
find_package(INDI 1.8 REQUIRED)
find_package(Nova REQUIRED)
find_package(ZLIB REQUIRED)
find_package(GSL REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# these will be used to set the version number in config.h and our driver's xml file
set(CDRIVER_VERSION_MAJOR 1)
set(CDRIVER_VERSION_MINOR 0)

# do the replacement in the config.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

# do the replacement in the driver's xml file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_rks_c8_focuser.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_rks_c8_focuser.xml
)

# set our include directories to look for header files
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${NOVA_INCLUDE_DIR})
include_directories( ${EV_INCLUDE_DIR})
include_directories(include)

include(CMakeCommon)

# tell cmake to build our executable
add_executable(
    indi_rks_c8_focuser
    src/indi_rks_c8_focuser.cpp
    src/HostComms.cpp
    src/FocuserComms.cpp
)

# and link it to these libraries
target_link_libraries(
    indi_rks_c8_focuser
    ${INDI_LIBRARIES}
    ${NOVA_LIBRARIES}
    ${GSL_LIBRARIES}
    Threads::Threads
)

# tell cmake where to install our executable
install(TARGETS indi_rks_c8_focuser RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/indi_rks_c8_focuser.xml
    DESTINATION ${INDI_DATA_DIR}
)