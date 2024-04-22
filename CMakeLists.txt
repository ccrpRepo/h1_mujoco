cmake_minimum_required(VERSION 3.14)
project(h1sys)

set(CMAKE_CXX_STANDARD 11)

set(USE_GL 1)
set(BIN_NAME ${PROJECT_NAME})

include(GNUInstallDirs)
set(PACKAGE_NAME matplotlib_cpp)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)

set(CMAKE_BUILD_TYPE "Release")
find_package(Boost)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
include_directories(${Pinocchio_INCLUDE_DIRS})
link_directories(${Pinocchio_LIBRARY_DIRS})

# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/examples>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_11
)
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS matplotlib_cpp
  EXPORT install_targets
)

#Mujoco paths
set(MJPRO_PATH /home/crp/.mujoco/mujoco210)
include_directories(${MJPRO_PATH}/include/mujoco)
include_directories(${MJPRO_PATH}/include)
link_directories(${MJPRO_PATH}/bin)
message(STATUS "MuJoCo path: " ${MJPRO_PATH})

#Finding main mujoco library
if(${USE_GL})
    file(GLOB LIB_MUJOCO ${MJPRO_PATH}/bin/libmujoco[0-9][0-9][0-9].so)
else()
    file(GLOB LIB_MUJOCO ${MJPRO_PATH}/bin/libmujoco[0-9][0-9][0-9]nogl.so)
endif()
#Showing mujoco library found
message(STATUS "MuJoCo lib: " ${LIB_MUJOCO})


include_directories(
    include
)

file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

# add_executable(h1test
# src/h1test.cpp
# ${SRC_LIST}
# )

add_executable(${BIN_NAME}
src/h1sys.cpp
${SRC_LIST}
)
link_libraries(/usr/local/lib/libpinocchio.so)
target_link_libraries(${BIN_NAME} pinocchio::pinocchio)
# target_link_libraries(h1test ${LIB_MUJOCO} pthread)

target_link_libraries(${BIN_NAME} ${LIB_MUJOCO} pthread matplotlib_cpp)

#Standard libraries for GL
target_link_libraries(${PROJECT_NAME} GL GLU)

# target_link_libraries(h1test GL GLU glut )

#Additional libraries from mujoco package
target_link_libraries(${PROJECT_NAME} ${MJPRO_PATH}/bin/libglew.so )

# target_link_libraries(h1test ${MJPRO_PATH}/bin/libglew.so )

target_link_libraries(${PROJECT_NAME} ${MJPRO_PATH}/bin/libglfw.so.3 )

# target_link_libraries(h1test ${MJPRO_PATH}/bin/libglfw.so.3 )
#target_link_libraries(${PROJECT_NAME}_exe libglewegl.so )
#target_link_libraries(${PROJECT_NAME}_exe libglewosmesa.so)

target_include_directories(${PROJECT_NAME} PUBLIC
 include
)

# target_include_directories(h1test PUBLIC
#  include
# )