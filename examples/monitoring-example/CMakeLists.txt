cmake_minimum_required(VERSION 3.4)
project(monitoring-example
						VERSION 1.0)

set(MONIT_EXAMPLE "monitoring-example")

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
	add_executable(${PROJECT_NAME} monitoring-example.cpp)
	target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES})
    include_directories(${PROJECT_NAME} ./../../common)
    set_target_properties (${PROJECT_NAME} PROPERTIES
        FOLDER "Sberbank"
    )
		
	install(TARGETS ${PROJECT_NAME}
		RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif()
# if(NOT DEFINED USE_PLATFROM)
    # set(USE_PLATFROM "Win32")
    # message("Using Win32 platform as default")
# endif()

# if("${USE_PLATFROM}" MATCHES "Win32")
	# message("USE_RS_LIBS for x86")
    # set(USE_RS_DLL "${PROJECT_SOURCE_DIR}/realsense2/bin/x86/realsense2.dll")
    # set(REALSENSE_LIB_PATH "${PROJECT_SOURCE_DIR}/realsense2/lib/x86")
# endif()

# if("${USE_PLATFROM}" MATCHES "x64")
	# message("USE_RS_LIBS for x64")
	# set(USE_RS_DLL "${PROJECT_SOURCE_DIR}/realsense2/bin/x64/realsense2.dll")
	# set(REALSENSE_LIB_PATH "${PROJECT_SOURCE_DIR}/realsense2/lib/x64")
# endif()

#set(MONIT_EXAMPLE_SOURCES
#	"example.cpp")

#find_library(REALSENSE_LIB
#    NAMES realsense2
#    PATHS "${REALSENSE_LIB_PATH}")

#if(${REALSENSE_LIB} STREQUAL "REALSENSE_LIB-NOTFOUND")
#    message("RealSense libraries not found in '${REALSENSE_LIB_PATH}', set correct REALSENSE_LIB_PATH")
#endif()

