# Install script for directory: /home/ninjak/Scaricati/ik_tests/blasfeo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/bin/x86_64-linux-gnu-objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/ninjak/Scaricati/ik_tests/build/blasfeo/libblasfeo.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig.cmake"
         "/home/ninjak/Scaricati/ik_tests/build/blasfeo/CMakeFiles/Export/cmake/blasfeoConfig.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES "/home/ninjak/Scaricati/ik_tests/build/blasfeo/CMakeFiles/Export/cmake/blasfeoConfig.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES "/home/ninjak/Scaricati/ik_tests/build/blasfeo/CMakeFiles/Export/cmake/blasfeoConfig-debug.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_block_size.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_common.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_aux.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_aux_ext_dep.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_aux_ext_dep_ref.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_aux_old.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_aux_ref.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_aux_test.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_blas.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_blas_api.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_blasfeo_api.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_blasfeo_api_ref.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_d_kernel.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_i_aux_ext_dep.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_m_aux.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_naming.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_processor_features.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_aux.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_aux_ext_dep.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_aux_ext_dep_ref.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_aux_old.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_aux_ref.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_aux_test.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_blas.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_blas_api.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_blasfeo_api.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_blasfeo_api_ref.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_s_kernel.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_stdlib.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_target.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_timing.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/blasfeo_v_aux_ext_dep.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/d_blas.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/d_blas_64.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/s_blas.h"
    "/home/ninjak/Scaricati/ik_tests/blasfeo/include/s_blas_64.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ninjak/Scaricati/ik_tests/build/blasfeo/examples/cmake_install.cmake")

endif()

