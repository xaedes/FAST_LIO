cmake_minimum_required(VERSION 3.8)
# set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_library(
    ${PROJECT_NAME} 
    # SHARED  
    STATIC
    include/${PROJECT_NAME}/ikd-Tree/ikd_Tree.cpp
    src/use_ikfom.cpp
    src/laserMapping.cpp
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC Boost::boost)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
