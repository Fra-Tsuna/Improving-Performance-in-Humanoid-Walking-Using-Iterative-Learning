# Install script for directory: /home/ninjak/Scaricati/ik_tests/hpipm

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/ninjak/Scaricati/ik_tests/build/hpipm/libhpipm.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/hpipmConfig.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/hpipmConfig.cmake"
         "/home/ninjak/Scaricati/ik_tests/build/hpipm/CMakeFiles/Export/cmake/hpipmConfig.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/hpipmConfig-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/hpipmConfig.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES "/home/ninjak/Scaricati/ik_tests/build/hpipm/CMakeFiles/Export/cmake/hpipmConfig.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES "/home/ninjak/Scaricati/ik_tests/build/hpipm/CMakeFiles/Export/cmake/hpipmConfig-debug.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_aux_mem.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_aux_string.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_common.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_cast_qcqp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_cond.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_cond_aux.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_cond_qcqp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_core_qp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_core_qp_ipm_aux.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qcqp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qcqp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qcqp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qcqp_res.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qcqp_sol.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qcqp_utils.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qp_kkt.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qp_res.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qp_sol.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_dense_qp_utils.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qcqp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qcqp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qcqp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qcqp_res.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qcqp_sol.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qcqp_utils.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qp_kkt.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qp_red.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qp_res.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qp_sol.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_ocp_qp_utils.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_part_cond.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_part_cond_qcqp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_sim_erk.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_sim_rk.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_tree_ocp_qp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_tree_ocp_qp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_tree_ocp_qp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_tree_ocp_qp_kkt.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_tree_ocp_qp_res.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_d_tree_ocp_qp_sol.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_m_dense_qp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_m_dense_qp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_m_ocp_qp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_m_ocp_qp_ipm_hard.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_m_ocp_qp_kkt.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_cast_qcqp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_cond.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_cond_aux.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_cond_qcqp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_core_qp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_core_qp_ipm_aux.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qcqp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qcqp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qcqp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qcqp_res.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qcqp_sol.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qcqp_utils.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qp_kkt.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qp_res.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qp_sol.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_dense_qp_utils.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qcqp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qcqp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qcqp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qcqp_res.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qcqp_sol.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qcqp_utils.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qp_kkt.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qp_red.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qp_res.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qp_sol.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_ocp_qp_utils.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_part_cond.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_part_cond_qcqp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_sim_erk.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_sim_rk.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_tree_ocp_qp.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_tree_ocp_qp_dim.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_tree_ocp_qp_ipm.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_tree_ocp_qp_kkt.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_tree_ocp_qp_res.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_s_tree_ocp_qp_sol.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_scenario_tree.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_target.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_timing.h"
    "/home/ninjak/Scaricati/ik_tests/hpipm/include/hpipm_tree.h"
    )
endif()

