cmake_minimum_required(VERSION 2.6)
project(hand-label)

set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_BUILD_TYPE Release)

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

#检查PCL库
find_package( PCL REQUIRED )
if( PCL_FOUND )
    message( STATUS "Found PCL: ${PCL_INCLUDE_DIRS}" )
    link_directories(${PCL_LIBRARY_DIRS})
endif( PCL_FOUND )

#开启Boost库
#set( Boost_USE_STATIC_LIBS ON )
#set( Boost_USE_STATIC ON )
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
find_package( Boost REQUIRED COMPONENTS
        system
        filesystem
        log
        thread
        )
if( Boost_FOUND )
    message( STATUS "Found Boost: ${Boost_LIBRARIES}" )
endif( Boost_FOUND )

#添加头文件
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

#添加库文件
set(
        DEPENDENCIES
        ${OpenCV_LIBRARIES}
        ${royale_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
)

add_definitions(
        ${PCL_DEFINITIONS}
)


aux_source_directory(./lib SRC)


add_executable(hand-label ${SRC} main.cpp)

COPY_ROYALE_LIBS(hand-label)
target_link_libraries(hand-label ${DEPENDENCIES})
