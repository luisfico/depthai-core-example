cmake_minimum_required(VERSION 3.2)

# Add depthai-core dependency
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)
#find_package(depthai CONFIG REQUIRED)
#find_package(depthai REQUIRED PATHS "/home/lc/Dev/depthai-core-example/build/depthai-core/_install")
#find_package(depthai REQUIRED PATHS "/home/lc/Dev/depthai-core-example/build/depthai-core/")

#list ( APPEND CMAKE_MODULE_PATH "/home/lc/Dev/depthai-core-example/depthai-core/cmake") 
#find_package(depthai MODULE)

# Create a project with name 'myapp'
set(TARGET_NAME myapp)
project(${TARGET_NAME})

# Dependencies (optional, only used for example)
find_package(OpenCV REQUIRED)

# PCL library
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Add source files
add_executable("${TARGET_NAME}"
    src/main.cpp
)

# Link with libraries
target_link_libraries(${TARGET_NAME}
    PUBLIC
        depthai::opencv
        ${OpenCV_LIBS} # optional, used for example
        ${PCL_LIBRARIES} # PCL library
)

# Set some errors
if(NOT MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Werror=return-type>)
endif()

# Set compiler features (c++11), and disables extensions (g++)
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_EXTENSIONS OFF)





# Add source files
add_executable(rgb_depth_aligned_ref
    src/rgb_depth_aligned_ref.cpp
)

# Link with libraries
target_link_libraries(rgb_depth_aligned_ref
    PUBLIC
        depthai::opencv
        ${OpenCV_LIBS} # optional, used for example
        ${PCL_LIBRARIES} # PCL library
)

# Set some errors
if(NOT MSVC)
    target_compile_options(rgb_depth_aligned_ref PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Werror=return-type>)
endif()

# Set compiler features (c++11), and disables extensions (g++)
set_property(TARGET rgb_depth_aligned_ref PROPERTY CXX_STANDARD 11)
set_property(TARGET rgb_depth_aligned_ref PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET rgb_depth_aligned_ref PROPERTY CXX_EXTENSIONS OFF)

