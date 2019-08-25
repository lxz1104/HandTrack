cmake_minimum_required(VERSION 2.6)
project(hand-label)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Release)

#设置头文件所在目录
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(ROYALE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/royale/include")

#添加PMD库
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/royale/share")
find_package(royale REQUIRED)
if( royale_FOUND )
    message( STATUS "Found royale: ${ROYALE_INCLUDE_DIR}" )
endif( royale_FOUND )
link_directories(${royale_LIB_DIR})
include_directories(
        ${ROYALE_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/royale/inc
)

#设置cmake模块所在路径
set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" )

#开启Boost库
find_package( Boost REQUIRED COMPONENTS
        system
        filesystem
        log
        thread
        )
if( Boost_FOUND )
    message( STATUS "Found Eigen: ${EIGEN_INCLUDE_DIRS}" )
endif( Boost_FOUND )

#设置cmake模块所在路径
#set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" )
set( Boost_USE_STATIC_LIBS ON )
set( Boost_USE_STATIC ON )

#检查OpenCV库
find_package( OpenCV REQUIRED )
if( OpenCV_FOUND )
    message( STATUS "Found OpenCV: ${OpenCV_INCLUDE_DIRS}" )
endif( OpenCV_FOUND )

find_package( Eigen REQUIRED COMPONENTS
        Dense
        )
if( Eigen_FOUND )
    message( STATUS "Found Eigen: ${EIGEN_INCLUDE_DIRS}" )
endif( Eigen_FOUND )

#添加头文件
include_directories(
        ${EIGEN_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${INCLUDE_DIR}
)

#添加库文件
set(
        DEPENDENCIES
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${royale_LIBS}
)

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

aux_source_directory(./lib SRC)

add_executable(hand-label ${SRC} main.cpp)

COPY_ROYALE_LIBS(hand-label)
target_link_libraries(hand-label ${DEPENDENCIES})
