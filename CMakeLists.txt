cmake_minimum_required(VERSION 3.10)
project(collision_detection VERSION 1.3)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(TARGET_NAME ${PROJECT_NAME})

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(assimp REQUIRED)

# Find required packages using pkg-config
include(FindPkgConfig)
pkg_check_modules(BASE_TYPES REQUIRED base_types)
pkg_check_modules(KDL_PARSER REQUIRED kdl_parser)
pkg_check_modules(URDF REQUIRED urdfdom)
pkg_check_modules(URDFHEADER REQUIRED urdfdom_headers)
pkg_check_modules(SRDFDOM REQUIRED srdfdom)
pkg_check_modules(CCD REQUIRED ccd)
pkg_check_modules(OCTOMAP REQUIRED octomap)
pkg_check_modules(FCL REQUIRED fcl)
pkg_check_modules(BASE_LOGGING REQUIRED base_logging)

# If FCL is found, configure it
if(FCL_FOUND)
    message(STATUS "FCL VERSION ${FCL_VERSION}")
    message(STATUS "FCL INCLUDE DIRS ${FCL_INCLUDE_DIRS}")
    message(STATUS "FCL LIBRARY DIRS ${FCL_LIBRARY_DIRS}")

    # Include FCL headers and set library directories
    include_directories(${FCL_INCLUDE_DIRS})
    link_directories(${FCL_LIBRARY_DIRS})
else()
    message(WARNING "FCL not found")
endif()

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
if(CLANG_FORMAT_EXECUTABLE)
    file(GLOB_RECURSE CPP_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")
    file(GLOB_RECURSE HPP_FILES "${CMAKE_SOURCE_DIR}/include/collision_detection/*.hpp")

    # Combine the lists of files
    set(FORMAT_FILES ${CPP_FILES} ${HPP_FILES})

    add_custom_target(format
      COMMAND ${CLANG_FORMAT_EXECUTABLE} -i ${FORMAT_FILES}
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      COMMENT "Running clang-format ..."
    )
else()
    message(WARNING "clang-format not found. Formatting target will not be available.")
endif()

# Define the abstract_collision_detection library
add_library(abstract_collision_detection SHARED
    src/abstract/AbstractCollisionDetection.cpp
)

# Include directories for dependencies
set(COLLISION_LIB_INCLUDES 
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${ASSIMP_INCLUDE_DIRS}
    ${KDL_PARSER_INCLUDE_DIRS}
    ${URDF_INCLUDE_DIRS} 
    ${URDFHEADER_INCLUDE_DIRS} 
    ${SRDFDOM_INCLUDE_DIRS} 
    ${OCTOMAP_INCLUDE_DIRS} 
    ${FCL_INCLUDE_DIRS}
    ${BASE_TYPES_INCLUDE_DIRS} 
    ${BASE_LOGGING_INCLUDE_DIRS}
    ${CCD_INCLUDE_DIRS}
)

# Set libraries to link with
set(COLLISION_DEP_LIBS
    Threads::Threads
    Eigen3::Eigen
    ${ASSIMP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY} 
    ${Boost_FILESYSTEM_LIBRARY} 
    ${KDL_PARSER_LIBRARIES} 
    ${URDF_LIBRARIES} 
    ${URDFHEADER_LIBRARIES}
    ${SRDFDOM_LIBRARIES} 
    ${OCTOMAP_LIBRARIES} 
    ${FCL_LIBRARIES}
    ${BASE_TYPES_LIBRARIES}
    ${BASE_LOGGING_LIBRARIES}
    ${CCD_LIBRARIES}
)

target_link_libraries(abstract_collision_detection PUBLIC ${COLLISION_DEP_LIBS}
)

target_include_directories(abstract_collision_detection PUBLIC ${COLLISION_LIB_INCLUDES}
)

# Define the fcl_wrapper library if FCL is found
if(FCL_FOUND)
    add_library(fcl_wrapper SHARED
        src/fcl_wrapper/FCLCollisionDetection.cpp
        src/fcl_wrapper/CollisionObjectAssociatedData.cpp
    )

    target_link_libraries(fcl_wrapper PUBLIC 
        abstract_collision_detection
        ${CCD_LIBRARIES}
        ${COLLISION_DEP_LIBS}
    )

    target_include_directories(fcl_wrapper PUBLIC 
        ${CCD_INCLUDE_DIRS}
        ${COLLISION_LIB_INCLUDES}
    )
endif()

# Define the final collision_detection library
add_library(collision_detection SHARED
    src/CollisionFactory.cpp
)

target_link_libraries(collision_detection
    fcl_wrapper
    abstract_collision_detection
)

# Set package information
set(TARGET_NAME ${PROJECT_NAME})
set(PROJECT_DESCRIPTION "Collection of collision detection libraries")

# Enable all warnings for space-qualification
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion -Wshadow -Wfloat-equal -Wnull-dereference -Wformat -Wcast-align -Wpointer-arith -Wunused-result")

# Package configuration
list(APPEND PKGCONFIG_REQUIRES kdl_parser urdfdom urdfdom_headers srdfdom octomap fcl base_types base_logging)
list(JOIN PKGCONFIG_REQUIRES " " PKGCONFIG_REQUIRES_STR)
configure_file("${CMAKE_SOURCE_DIR}/${PROJECT_NAME}.pc.in" "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc" @ONLY)

# Install headers and libraries
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME} DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(TARGETS abstract_collision_detection DESTINATION lib)
install(TARGETS fcl_wrapper DESTINATION lib)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

add_executable(test_collision_detection test/test_collision_detection.cpp)

target_include_directories(test_collision_detection PRIVATE ${MOTIONPLANNER_LIB_INCLUDES})

target_link_libraries(test_collision_detection PRIVATE
    ${TARGET_NAME}
    ${MOTIONPLANNER_DEP_LIBS}
)

add_test(NAME CollisionDetectionTest COMMAND test_collision_detection)

add_executable(test_space_qualification test/test_space_qualification.cpp)

target_include_directories(test_space_qualification PRIVATE ${MOTIONPLANNER_LIB_INCLUDES})
target_include_directories(test_space_qualification SYSTEM PRIVATE ${COLLISION_LIB_THIRD_PARTY_INCLUDES})

target_link_libraries(test_space_qualification PRIVATE
    ${TARGET_NAME}
    ${MOTIONPLANNER_DEP_LIBS}
)

add_test(NAME SpaceQualificationTest COMMAND test_space_qualification)
