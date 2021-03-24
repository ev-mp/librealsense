cmake_minimum_required(VERSION 3.4)
project(depth-monitor	VERSION 1.2)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(${PROJECT_NAME} check_depth.cpp)
    target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES})
    include_directories(${PROJECT_NAME} ./../../third-party)
    set_target_properties (${PROJECT_NAME} PROPERTIES
        FOLDER "Sberbank"
    )

    install(TARGETS	${PROJECT_NAME}
        RUNTIME DESTINATION	${CMAKE_INSTALL_PREFIX}/bin
    )
endif()


#set(CHECK_DEPTH_MONITORING "CheckDepthMonitoring")
#set(USE_RS_DLL "${PROJECT_SOURCE_DIR}/realsense2/bin/realsense2.dll")
#set(REALSENSE_LIB_PATH "${PROJECT_SOURCE_DIR}/realsense2/lib")	

# set(CHECK_DEPTH_MONITORING_SOURCES
	# "check_depth.cpp")

# find_library(REALSENSE_LIB
    # NAMES realsense2
    # PATHS "${REALSENSE_LIB_PATH}")

# if(${REALSENSE_LIB} STREQUAL "REALSENSE_LIB-NOTFOUND")
    # message("RealSense libraries not found in '${REALSENSE_LIB_PATH}', set correct REALSENSE_LIB_PATH")
# endif()

# add_executable(${CHECK_DEPTH_MONITORING} ${CHECK_DEPTH_MONITORING_SOURCES})

# target_include_directories(${CHECK_DEPTH_MONITORING} PRIVATE 
	# "${PROJECT_SOURCE_DIR}/realsense2/include/")

# target_include_directories(${CHECK_DEPTH_MONITORING} PRIVATE 
	# "${PROJECT_SOURCE_DIR}/third-party/")

# target_link_libraries(${CHECK_DEPTH_MONITORING} PRIVATE ${REALSENSE_LIB})

# add_custom_command(TARGET ${CHECK_DEPTH_MONITORING} POST_BUILD 
	# COMMAND "${CMAKE_COMMAND}" -E copy 
		# "${USE_RS_DLL}"
		# "$(OutDir)")
