cmake_minimum_required(VERSION 3.20)
project(apexdrive VERSION 1.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Strict Compilation Flags
if(MSVC)
    add_compile_options(/W4 /WX /O2)
else()
    add_compile_options(-O3 -Wall -Wextra -Werror -Wpedantic)
endif()

# Include Directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 1. Host Actuator Library (with Real Linux SocketCAN transport & Platform Auto-Detector)
add_library(apexdrive_host STATIC
    src/host/actuator.cpp
    src/host/can_socket.cpp
    src/core/platform_detector.cpp
)
set_target_properties(apexdrive_host PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "include/apexdrive/host/actuator.hpp;include/apexdrive/host/can_socket.hpp"
)

# 2. Developer CLI Binary
add_executable(apexdrive
    src/cli/main.cpp
)
target_link_libraries(apexdrive PRIVATE apexdrive_host)

# 3. Unit Test Suite (Mathematical & Component Tests)
add_executable(test_suite
    tests/test_suite.cpp
)
target_link_libraries(test_suite PRIVATE apexdrive_host)

# 4. Hardware-Readiness & Closed-Loop Scenarios Test Suite
add_executable(control_scenarios_test
    tests/control_scenarios_test.cpp
)
target_link_libraries(control_scenarios_test PRIVATE apexdrive_host)

# 5. Multi-Joint CAN-FD Real-Time Streaming Example
add_executable(example_can_streaming
    examples/can_hardware_streaming.cpp
)
target_link_libraries(example_can_streaming PRIVATE apexdrive_host)

# Enable CTest
enable_testing()
add_test(NAME ApexDriveUnitTests COMMAND test_suite)
add_test(NAME ApexDriveControlScenarios COMMAND control_scenarios_test)

# ==============================================================================
# INSTALLATION & CPACK DISTRIBUTION (Debian / RPM / Homebrew / Tarball)
# ==============================================================================
include(GNUInstallDirs)

# Install Targets
install(TARGETS apexdrive apexdrive_host
    EXPORT ApexDriveTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/apexdrive/host
)

# Install Header Files
install(DIRECTORY include/apexdrive
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp"
)

# CPack Packaging Configuration
set(CPACK_PACKAGE_NAME "apexdrive")
set(CPACK_PACKAGE_VENDOR "ApexDrive Robotics Team")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "High-Performance Robotics Actuator & Motor Control Engine")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Himan-D <dev@apexdrive.io>")
set(CPACK_DEBIAN_PACKAGE_SECTION "science")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.31), libstdc++6 (>= 11)")
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/Himan-D/apexdrive")

set(CPACK_RPM_PACKAGE_LICENSE "Apache-2.0")
set(CPACK_RPM_PACKAGE_GROUP "Development/Tools")

set(CPACK_GENERATOR "TGZ;DEB;RPM")
include(CPack)
