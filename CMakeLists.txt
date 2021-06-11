cmake_minimum_required(VERSION 2.8.3)
project(bz_robot)


#SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#将git当前版本信息编译到可执行文件中
exec_program(
    "git"
    ${CMAKE_CURRENT_SOURCE_DIR}
    ARGS "log --format='[sha1]:%h [author]:%cn [time]:%ci [commit]:%s [branch]:%d' -1"
    OUTPUT_VARIABLE VERSION_SHA1 )

string(REGEX REPLACE "\#"
           "$" VERSION_SHA1
           ${VERSION_SHA1})

add_definitions( -DGIT_SHA1="${VERSION_SHA1}" )

## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


#FIND_PACKAGE(Boost)
#IF (Boost_FOUND)
#   INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
#   #ADD_DEFINITIONS( "-DHAS_BOOST" )
#ENDIF()

# for print call trace by zhou
add_compile_options(-rdynamic)
add_compile_options(-g -lmcheck)
#add_compile_options(-fopenmp)
#add_compile_options(-fpermissive)
add_compile_options(-lstdc++)
add_compile_options(-fPIC)
#add_compile_options(-O0)
#add_compile_options(-save-temps)
#test link nanomsg
#add_compile_options(-lnanomsg)
#add_compile_options('--copy-dt-needed-entries')
SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /opt/ros/melodic/ /usr)


 # 打印出编译和链接阶段的耗时情况
# set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
# set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CMAKE_COMMAND} -E time")
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    tf
    costmap_2d
    base_local_planner
    pluginlib
    )
find_package(Boost COMPONENTS system filesystem REQUIRED)
#link_directories(${CMAKE_CURRENT_SOURCE_DIR}/third/boost_1_74_0/stage/lib)
#link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/third/boost_1_74_0/stage/lib/libboost_serialization.so)
#link_libraries("/home/zhou/work/bz_robot/third/boost_1_74_0/stage/lib/libboost_serialization.so")
# link_libraries("/usr/local/lib/libipopt.so.3.13.3")
## OPEN MOTION PLANNING LIBRARY
#find_package(OMPL REQUIRED)
# find_package(ompl REQUIRED)
# if(NOT OMPL_FOUND)
#     message(AUTHOR_WARNING,"Open Motion Planning Library not found")
# endif(NOT OMPL_FOUND)

# find_package(Nanomsg REQUIRED)
# if(NOT NNG_FOUND)
#     message(AUTHOR_WARNING,"Nanomsg not found")
# endif()
#find_package(Eigen3)
#INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
find_package(OsqpEigen REQUIRED)
INCLUDE_DIRECTORIES(${OSQPEIGEN_INCLUDE_DIR})
# include_directories( ${NNG_INCLUDE_DIRS} )
# link_directories(${NNG_LIBRARY_DIRS})

# FIND_PACKAGE( OpenMP REQUIRED)
# if(OPENMP_FOUND)
# message("OPENMP FOUND")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif()

find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
include_directories( ${SDL_INCLUDE_DIR}
                     ${SDL_IMAGE_INCLUDE_DIRS} )

#find_package( PCL REQUIRED)
#include_directories( ${PCL_INCLUDE_DIRS} )
#link_directories(${PCL_LIBRARY_DIRS})
#find_package(nlohmann_json REQUIRED)
#include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(include ${OMPL_INCLUDE_DIRS})

#message("CMAKE_CURRENT_SOURCE_DIR:\n${CMAKE_CURRENT_SOURCE_DIR}")
find_package(PythonLibs 2.7)
#link_libraries(${PYTHON_LIBRARIES})
include_directories(${PYTHON_INCLUDE_DIRS})
## HEADER ##

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include /usr/include/pcl-1.8)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/External/blasfeo/lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/External/hpipm/lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/fuzzy/fuzzylite)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/third_libs/)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/third_libs/eigen3)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/third_libs/osqp/include)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/third_libs/osqp-eigen-0.6.2)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/third_libs/osqp-eigen-0.6.2/include)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/common)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/common/timer)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/main)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/main/ros_plugins)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/collision_detector)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/localization)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/map)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/model)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/planner)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/planner/ackermann_wave_front_planner)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/planner/fmm_planner)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/planner/frenet_optimal_planner)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/planner/frenet_polynomial_planner)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/planner/r_star_planner)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/robot)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/robot/laser)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/smoother)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/smoother/gd_smoother)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/smoother/sample_optimizing_smoother)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/smoother/sample_optimizing_smoother/config)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/smoother/sample_optimizing_smoother/data_struct)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/smoother/sample_optimizing_smoother/path_optimizer)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/smoother/sample_optimizing_smoother/reference_path_smoother)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/smoother/sample_optimizing_smoother/solver)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/smoother/sample_optimizing_smoother/tinyspline)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/smoother/sample_optimizing_smoother/tools)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/common)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/fuzzy)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/lqr)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/mpc)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/fnn)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/test)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/test/matplotlib)

#include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/mpc_hpipm)
#include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/mpcc)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/pid)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/task)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/service)
#include_directories (${CMAKE_CURRENT_SOURCE_DIR}/third_libs/)
#include_directories (${CMAKE_CURRENT_SOURCE_DIR}/third_libs/rest_rpc/)
#include_directories (${CMAKE_CURRENT_SOURCE_DIR}/third_libs/rest_rpc/include/)
#include_directories (${CMAKE_CURRENT_SOURCE_DIR}/third_libs/rest_rpc/third/msgpack/include/)

## SRC ##

AUX_SOURCE_DIRECTORY (src DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/common DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/common/timer DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/main DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/main/ros_plugins DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/collision_detector DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/localization DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/map DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/model DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/planner DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/planner/ackermann_wave_front_planner DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/planner/fmm_planner DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/planner/frenet_optimal_planner DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/planner/frenet_polynomial_planner DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/planner/r_star_planner DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/robot DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/robot/laser DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/smoother DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/smoother/gradient_descent_smoother DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/smoother/sample_optimizing_smoother DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/smoother/sample_optimizing_smoother/config DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/smoother/sample_optimizing_smoother/data_struct DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/smoother/sample_optimizing_smoother/path_optimizer DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/smoother/sample_optimizing_smoother/reference_path_smoother DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/smoother/sample_optimizing_smoother/solver DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/smoother/sample_optimizing_smoother/tinyspline DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/smoother/sample_optimizing_smoother/tools DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/tracker DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/tracker/common DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/tracker/lqr DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/tracker/mpc DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fnn DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/tracker/test DIR_SRCS)

#AUX_SOURCE_DIRECTORY (src/modules/tracker/mpc_hpipm DIR_SRCS)
#AUX_SOURCE_DIRECTORY (src/modules/tracker/mpcc DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/modules/tracker/pid DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/task DIR_SRCS)
AUX_SOURCE_DIRECTORY (src/service DIR_SRCS)

AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy/fuzzylite/src DIR_SRCS1)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy/fuzzylite/src/activation DIR_SRCS1)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy/fuzzylite/src/defuzzifier DIR_SRCS1)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy/fuzzylite/src/factory DIR_SRCS1)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy/fuzzylite/src/hedge DIR_SRCS1)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy/fuzzylite/src/imex DIR_SRCS1)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy/fuzzylite/src/norm DIR_SRCS1)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy/fuzzylite/src/rule DIR_SRCS1)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy/fuzzylite/src/term DIR_SRCS1)
AUX_SOURCE_DIRECTORY (src/modules/tracker/fuzzy/fuzzylite/src/variable DIR_SRCS1)


#MESSAGE(STATUS "Src file: ${DIR_SRCS1}")


#GET_DIRECTORY_PROPERTY(output INCLUDE_DIRECTORIES)
#MESSAGE(WARNING ${output})


link_libraries("/usr/local/lib/libipopt.so")
#link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/third_libs/OsqpEigen/libOsqpEigen.so)
link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/third_libs/osqp-eigen-0.6.2/build/libOsqpEigen.so)
link_libraries("/usr/lib/x86_64-linux-gnu/libpcl_common.so")
link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/src/modules/tracker/fuzzy/fuzzylite/release/bin/libfuzzylite.so)

# 动态库
add_library(bz_robot_lib SHARED ${DIR_SRCS})
add_library(fuzzy_lib SHARED ${DIR_SRCS1})
# 静态库
# add_library(bz_robot_lib STATIC ${DIR_SRCS})

#屏蔽PCL，改为手动指定PCL库，解决不能连接libevent库的问题
#find_package( PCL REQUIRED)



target_link_libraries(bz_robot_lib
    #${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
    ${Boost_LIBRARIES}
    -pthread
    #-levent
    #-levent_pthreads
    -lstdc++
    xmlrpc++
    xmlrpc_client++
)




##########################
# applications
##########################
add_executable(bz_robot src/app/bz_robot.cpp)
target_link_libraries(bz_robot
    bz_robot_lib
)
add_executable(test src/modules/tracker/test/test_main.cpp)
target_link_libraries(test
    bz_robot_lib
)
#add_executable(collision_detector_server src/app/collision_detector_server.cpp)
#target_link_libraries(collision_detector_server
#    bz_robot_lib
#)


#add_executable(localization_server src/app/localization_server.cpp)
#target_link_libraries(localization_server
#    bz_robot_lib
#)

#add_executable(map_server src/app/map_server.cpp)
#target_link_libraries(map_server
#    bz_robot_lib
#)

#add_executable(planner_server src/app/planner_server.cpp)
#target_link_libraries(planner_server
#    bz_robot_lib
#)

#add_executable(task_manager src/app/task_manager.cpp)
#target_link_libraries(task_manager
#    bz_robot_lib
#)

#add_executable(robot_server src/app/robot_server.cpp)
#target_link_libraries(robot_server
#    bz_robot_lib
#)

#add_executable(service_server src/app/service_server.cpp)
#target_link_libraries(service_server
#    bz_robot_lib
#)

#add_executable(smoother_server src/app/smoother_server.cpp)
#target_link_libraries(smoother_server
#    bz_robot_lib
#)

#add_executable(task_server src/app/task_server.cpp)
#target_link_libraries(task_server
#    bz_robot_lib
#)


#add_executable(tracker_server src/app/tracker_server.cpp)
#target_link_libraries(tracker_server
#    bz_robot_lib
#)

#include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(${catkin_INCLUDE_DIRS})
# message(${catkin_LIBRARIES})

add_executable(map_to_odom src/app/ros_plugins/map_to_odom.cpp)
target_link_libraries(map_to_odom
    ${catkin_LIBRARIES}
    )

##屏蔽PCL，改为手动指定PCL库，解决不能连接libevent库的问题
##find_package( PCL REQUIRED)
#include_directories(include /usr/include/pcl-1.8)
#link_libraries("/usr/lib/x86_64-linux-gnu/libpcl_common.so")

#add_executable(ros_interface src/app/ros_plugins/ros_interface.cpp)
#target_link_libraries(ros_interface
#     bz_robot_lib
#     #${PCL_LIBRARIES}
#     ${catkin_LIBRARIES}
# )




