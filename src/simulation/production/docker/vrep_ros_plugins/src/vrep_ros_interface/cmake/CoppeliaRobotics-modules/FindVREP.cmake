cmake_minimum_required(VERSION 2.8.12)
include(CMakeParseArguments)

if(NOT VREP_ROOT)
    if(NOT DEFINED ENV{VREP_ROOT})
        if(EXISTS "${CMAKE_SOURCE_DIR}/../../programming/include")
            get_filename_component(VREP_PROGRAMMING ${CMAKE_SOURCE_DIR} PATH)
            get_filename_component(VREP_ROOT ${VREP_PROGRAMMING} PATH)
        else()
            message(FATAL_ERROR "Cannot find V-REP installation. Please set the VREP_ROOT environment variable to point to the root of your V-REP installation.")
        endif()
    else()
        set(VREP_ROOT "$ENV{VREP_ROOT}")
    endif()
endif()

file(TO_CMAKE_PATH "${VREP_ROOT}" VREP_ROOT)

if(EXISTS "${VREP_ROOT}/programming/include" AND EXISTS "${VREP_ROOT}/programming/common")
    set(VREP_INCLUDE "${VREP_ROOT}/programming/include")
    set(VREP_COMMON "${VREP_ROOT}/programming/common")
    if(NOT EXISTS "${VREP_INCLUDE}/v_repLib.h")
        message(FATAL_ERROR "Cannot find v_repLib.h in ${VREP_INCLUDE}.")
    endif()
    if(NOT EXISTS "${VREP_INCLUDE}/v_repConst.h")
        message(FATAL_ERROR "Cannot find v_repConst.h in ${VREP_INCLUDE}.")
    endif()
    if(NOT EXISTS "${VREP_COMMON}/v_repLib.cpp")
        message(FATAL_ERROR "Cannot find v_repLib.cpp in ${VREP_COMMON}.")
    endif()
    if(NOT VREP_FIND_QUIETLY)
        message(STATUS "Found V-REP installation at ${VREP_ROOT}.")
    endif()

    set(VREP_VERSION_CHECK_SRC "${CMAKE_BINARY_DIR}/vrep_version_check.cpp")
    set(VREP_VERSION_CHECK_BIN "${CMAKE_BINARY_DIR}/vrep_version_check")
    file(WRITE ${VREP_VERSION_CHECK_SRC} "
#include <iostream>
#include <v_repConst.h>
int main() {
    char sep = ';';
    std::cout
        << VREP_PROGRAM_VERSION_NB/10000 << sep
        << VREP_PROGRAM_VERSION_NB/100%100 << sep
        << VREP_PROGRAM_VERSION_NB%100 << sep
        << VREP_PROGRAM_REVISION_NB << sep
        << 0 << std::endl;
}
    ")
    if(NOT VREP_FIND_QUIETLY)
        message(STATUS "Checking V-REP header version...")
    endif()
    try_run(VREP_VERSION_RUN_RESULT VREP_VERSION_COMPILE_RESULT ${VREP_VERSION_CHECK_BIN} ${VREP_VERSION_CHECK_SRC} CMAKE_FLAGS -DINCLUDE_DIRECTORIES=${VREP_INCLUDE} RUN_OUTPUT_VARIABLE VREP_VERSION_CHECK_OUTPUT)
    if(${VREP_VERSION_COMPILE_RESULT})
        if(${VREP_VERSION_RUN_RESULT} EQUAL 0)
            list(GET VREP_VERSION_CHECK_OUTPUT 0 VREP_VERSION_MAJOR)
            list(GET VREP_VERSION_CHECK_OUTPUT 1 VREP_VERSION_MINOR)
            list(GET VREP_VERSION_CHECK_OUTPUT 2 VREP_VERSION_PATCH)
            list(GET VREP_VERSION_CHECK_OUTPUT 3 VREP_VERSION_TWEAK)
            set(VREP_VERSION_COUNT 4)
            list(GET VREP_VERSION_CHECK_OUTPUT 3 VREP_REVISION)
            set(VREP_VERSION "${VREP_VERSION_MAJOR}.${VREP_VERSION_MINOR}.${VREP_VERSION_PATCH}.${VREP_REVISION}")
            set(VREP_VERSION_STR "${VREP_VERSION_MAJOR}.${VREP_VERSION_MINOR}.${VREP_VERSION_PATCH} rev${VREP_REVISION}")
            if(NOT VREP_FIND_QUIETLY)
                message(STATUS "V-REP headers version ${VREP_VERSION_STR}")
            endif()
            if(DEFINED VREP_FIND_VERSION)
                if(${VREP_VERSION} VERSION_LESS ${VREP_FIND_VERSION})
                    message(FATAL_ERROR "Found V-REP version ${VREP_VERSION} but ${VREP_FIND_VERSION} required.")
                endif()
            endif()
        else()
            message(FATAL_ERROR "Failed to run V-REP version check program")
        endif()
    else()
        message(FATAL_ERROR "Failed to compile V-REP version check program")
    endif()

    set(VREP_FOUND TRUE)
else()
    if(VREP_FIND_REQUIRED)
        message(FATAL_ERROR "The specified VREP_ROOT dir does not point to a valid V-REP installation.")
    endif()
endif()

set(VREP_EXPORTED_SOURCES "${VREP_COMMON}/v_repLib.cpp")

if(WIN32)
    add_definitions(-DWIN_VREP)
    add_definitions(-DNOMINMAX)
    add_definitions(-Dstrcasecmp=_stricmp)
    set(VREP_LIBRARIES shlwapi)
elseif(UNIX AND NOT APPLE)
    if(NOT VREP_FIND_QUIETLY)
        message(STATUS "Checking if V-REP has Qt3D...")
    endif()
    execute_process(COMMAND ldd ${VREP_ROOT}/libv_rep.so OUTPUT_VARIABLE VREP_LIB_DEPS)
    if(${VREP_LIB_DEPS} MATCHES Qt3D)
        if(NOT VREP_FIND_QUIETLY)
            message(STATUS "V-REP has Qt3D")
        endif()
        set(VREP_QT3D TRUE)
    else()
        if(NOT VREP_FIND_QUIETLY)
            message(STATUS "V-REP has no Qt3D")
        endif()
        set(VREP_QT3D FALSE)
    endif()
    add_definitions(-DLIN_VREP)
    set(VREP_LIBRARIES "")
elseif(UNIX AND APPLE)
    if(NOT VREP_FIND_QUIETLY)
        message(STATUS "Checking if V-REP has Qt3D...")
    endif()
    execute_process(COMMAND otool -L ${VREP_ROOT}/vrep.app/Contents/MacOS/libv_rep.dylib OUTPUT_VARIABLE VREP_LIB_DEPS)
    if(${VREP_LIB_DEPS} MATCHES Qt3D)
        if(NOT VREP_FIND_QUIETLY)
            message(STATUS "V-REP has Qt3D")
        endif()
        set(VREP_QT3D TRUE)
    else()
        if(NOT VREP_FIND_QUIETLY)
            message(STATUS "V-REP has no Qt3D")
        endif()
        set(VREP_QT3D FALSE)
    endif()
    add_definitions(-DMAC_VREP)
    set(VREP_LIBRARIES "")
endif()

function(VREP_GENERATE_STUBS GENERATED_OUTPUT_DIR)
    cmake_parse_arguments(VREP_GENERATE_STUBS "" "XML_FILE;LUA_FILE" "" ${ARGN})
    if("${VREP_GENERATE_STUBS_LUA_FILE}" STREQUAL "")
        add_custom_command(OUTPUT ${GENERATED_OUTPUT_DIR}/stubs.cpp ${GENERATED_OUTPUT_DIR}/stubs.h
            COMMAND python ${CMAKE_SOURCE_DIR}/external/v_repStubsGen/generate.py --xml-file ${VREP_GENERATE_STUBS_XML_FILE} --gen-all ${GENERATED_OUTPUT_DIR}
            DEPENDS ${VREP_GENERATE_STUBS_XML_FILE})
    else()
        add_custom_command(OUTPUT ${GENERATED_OUTPUT_DIR}/stubs.cpp ${GENERATED_OUTPUT_DIR}/stubs.h ${GENERATED_OUTPUT_DIR}/lua_calltips.cpp
            COMMAND python ${CMAKE_SOURCE_DIR}/external/v_repStubsGen/generate.py --xml-file ${VREP_GENERATE_STUBS_XML_FILE} --lua-file ${VREP_GENERATE_STUBS_LUA_FILE} --gen-all ${GENERATED_OUTPUT_DIR}
            DEPENDS ${VREP_GENERATE_STUBS_XML_FILE})
    endif()
    set(VREP_EXPORTED_SOURCES ${VREP_EXPORTED_SOURCES} "${GENERATED_OUTPUT_DIR}/stubs.cpp")
endfunction(VREP_GENERATE_STUBS)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
    find_package(Git)
    if(GIT_FOUND)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE "BUILD_GIT_VERSION"
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE)
    else()
        set(BUILD_GIT_VERSION "unknown")
    endif()
endif()

