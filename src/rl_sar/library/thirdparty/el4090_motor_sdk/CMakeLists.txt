cmake_minimum_required(VERSION 3.15)
project(MasterStack)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(BUILD_SHARED_LIBS OFF)

# 添加第三方库 (SOEM)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/soem")
    add_subdirectory(soem)
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/soem")
    add_subdirectory(thirdparty/soem)
else()
    message(FATAL_ERROR "SOEM library not found in expected locations")
endif()

# 包含目录
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/soem")
    set(SOEM_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/soem/soem
        ${CMAKE_CURRENT_SOURCE_DIR}/soem/osal
        ${CMAKE_CURRENT_SOURCE_DIR}/soem/osal/linux
        ${CMAKE_CURRENT_SOURCE_DIR}/soem/oshw
        ${CMAKE_CURRENT_SOURCE_DIR}/soem/oshw/linux
    )
else()
    set(SOEM_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/soem/soem
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/soem/osal
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/soem/osal/linux
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/soem/oshw
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/soem/oshw/linux
    )
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SOEM_INCLUDE_DIRS}
)

# 数学库
find_library(MATH_LIB m)

# 创建 motor_lib (for rl_sar integration)
add_library(el4090_motor_control
    motor_control.c
    math_ops.c
    transmit.cpp
    command.cpp
)

target_include_directories(el4090_motor_control PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SOEM_INCLUDE_DIRS}
)

target_link_libraries(el4090_motor_control PUBLIC
    soem
    Threads::Threads
)

if(MATH_LIB)
    target_link_libraries(el4090_motor_control PUBLIC ${MATH_LIB})
endif()

# Python bindings (optional)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
if(BUILD_PYTHON_BINDINGS)
    # 处理传入的 PYTHON_EXECUTABLE
    if(DEFINED PYTHON_EXECUTABLE)
        set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE}")
    endif()

    # 查找 Python 和 pybind11
    find_package(Python3 COMPONENTS Interpreter Development)
    find_package(pybind11 CONFIG)

    if(Python3_FOUND AND pybind11_FOUND)
        # 添加 cpp-readline if needed
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cpp-readline")
            add_subdirectory(thirdparty/cpp-readline)
            
            # 创建 Python 模块
            pybind11_add_module(pyethercat pyethercat.cpp)
            target_link_libraries(pyethercat PRIVATE el4090_motor_control cpp-readline)
        endif()
    else()
        message(WARNING "Python3 or pybind11 not found, skipping Python bindings")
    endif()
endif()

# 可执行文件 (optional, for testing)
option(BUILD_CPP_EXECUTABLE "Build main.cpp executable" OFF)
if(BUILD_CPP_EXECUTABLE)
    get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} PATH)
    if(EXISTS "${PARENT_DIR}/main.cpp")
        add_executable(master_stack_test ${PARENT_DIR}/main.cpp)
        target_link_libraries(master_stack_test PRIVATE el4090_motor_control)
    endif()
endif()