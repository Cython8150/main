#set(GENERATE OFF CACHE BOOL "Generate wrappers for objects and methods" FORCE)
cmake_minimum_required(VERSION 3.5)
project(zmqRemoteApi-client-cpp3)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        message(FATAL_ERROR "Environment variable COPPELIASIM_ROOT_DIR is not set")
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.1.0.0 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GENERATE OFF CACHE BOOL "Generate wrappers for objects and methods (requires CoppeliaSim to be running).")
set(GENERATE_INCLUDE_OBJECTS "" CACHE STRING "Include only given objects (e.g.: \"sim,simIK\") or leave blank to include all objects in wrapper code generation.")
set(GENERATE_EXCLUDE_OBJECTS "simEigen,simB0,simRemoteApi,simQML,simOMPL,simUI,simIM,simIGL,simURDF,simSkel,simBWF,simCmd,simSubprocess,simURLDrop,simWS,simZMQ" CACHE STRING "Exclude given objects (e.g.: \"simUI,simIM\") from being generated. Note: option GENERATE_INCLUDE_OBJECTS, if different from \"\", has precedence over this.")
set(GENERATE_EXCLUDE_METHODS "sim.test,sim.auxFunc,sim.getStringSignal,sim.getInt32Signal,sim.getFloatSignal" CACHE STRING "Exclude given methods (e.g.: \"sim.test,simUI.foo\") from being generated.")

# include(FetchContent)

# FetchContent_Declare(jsoncons
#     GIT_REPOSITORY https://github.com/danielaparker/jsoncons
#     SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/jsoncons
# )
# FetchContent_GetProperties(jsoncons)
# if(NOT jsoncons_POPULATED)
#     FetchContent_Populate(jsoncons)
#     #add_subdirectory(${jsoncons_SOURCE_DIR} ${jsoncons_BINARY_DIR} EXCLUDE_FROM_ALL)
# endif()

# set(CPPZMQ_BUILD_TESTS OFF CACHE BOOL "" FORCE)
# FetchContent_Declare(cppzmq
#     GIT_REPOSITORY https://github.com/zeromq/cppzmq
#     SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cppzmq
# )
# FetchContent_GetProperties(cppzmq)
# if(NOT cppzmq_POPULATED)
#     FetchContent_Populate(cppzmq)
#     add_subdirectory(${cppzmq_SOURCE_DIR} ${cppzmq_BINARY_DIR} EXCLUDE_FROM_ALL)
# endif()

set(jsoncons_SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/jsoncons)
set(cppzmq_SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cppzmq)
add_subdirectory(${cppzmq_SOURCE_DIR} ${cppzmq_BINARY_DIR} EXCLUDE_FROM_ALL)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)

add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
    COMMAND
        ${CMAKE_COMMAND} -E env
        PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}/../python/src"
        ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/get_raw_calltips.py"
        "${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/get_raw_calltips.py"
)
add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
    COMMAND
        ${CMAKE_COMMAND} -E env
        PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}/../python/src"
        ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/get_constants.py"
        "${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/get_constants.py"
)

set(generatedFiles)
file(GLOB templateFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/templates/ ${CMAKE_CURRENT_SOURCE_DIR}/templates/*)
foreach(templateFile ${templateFiles})
    add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}"
        COMMAND
            ${Python3_EXECUTABLE}
            "${COPPELIASIM_INCLUDE_DIR}/simStubsGen/external/pycpp/pycpp.py"
            -p "calltips_json=${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
            -p "constants_json=${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
            -p "include_objects=${GENERATE_INCLUDE_OBJECTS}"
            -p "exclude_objects=${GENERATE_EXCLUDE_OBJECTS}"
            -p "exclude_methods=${GENERATE_EXCLUDE_METHODS}"
            -i "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}"
            -o "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}"
            -P "${COPPELIASIM_PYTHON_DIR}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/../../tools"
            -P "${CMAKE_CURRENT_SOURCE_DIR}"
        DEPENDS
            "${COPPELIASIM_INCLUDE_DIR}/simStubsGen/external/pycpp/pycpp.py"
            "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}"
            "${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
            "${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
            "${CMAKE_CURRENT_SOURCE_DIR}/cpp_utils.py"
            "${COPPELIASIM_PYTHON_DIR}/calltip.py"
            "${COPPELIASIM_PYTHON_DIR}/calltip.lark"
    )
    list(APPEND generatedFiles "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}")
endforeach()
add_custom_target(generate_code DEPENDS ${generatedFiles})

add_library(RemoteAPIClient STATIC RemoteAPIClient.cpp)
if(GENERATE)
    add_dependencies(RemoteAPIClient generate_code)
else()
    foreach(templateFile ${templateFiles})
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${templateFile}")
            message(FATAL_ERROR "File ${CMAKE_CURRENT_SOURCE_DIR}/${templateFile} is missing")
        endif()
    endforeach()
endif()
target_compile_definitions(RemoteAPIClient PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS)
target_include_directories(RemoteAPIClient PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/jsoncons/include)
if(GENERATE)
    target_include_directories(RemoteAPIClient BEFORE PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/generated)
    set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/generated/RemoteAPIObjects.h)
    set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/generated/RemoteAPIObjects.cpp)
else()
    set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/RemoteAPIObjects.h)
    set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/RemoteAPIObjects.cpp)
endif()
target_link_libraries(RemoteAPIClient PUBLIC cppzmq)

add_executable(example example.cpp)
target_link_libraries(example PRIVATE RemoteAPIClient)

add_executable(synchronousImageTransmission synchronousImageTransmission.cpp)
target_link_libraries(synchronousImageTransmission PRIVATE RemoteAPIClient)

add_executable(bubbleRobClient bubbleRobClient.cpp)
target_link_libraries(bubbleRobClient PRIVATE RemoteAPIClient)

#add_executable(testConnection testConnection.cpp)
#target_link_libraries(testConnection PRIVATE RemoteAPIClient)

#find_package(OpenCV COMPONENTS core imgproc highgui)
find_package(OpenCV COMPONENTS 
    core 
    imgproc 
    highgui 
    imgcodecs
    calib3d
    features2d
    videoio
)
add_executable(testGrab testGrab.cpp find_handle.cpp control.cpp visual_module.cpp)
target_include_directories(testGrab PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(testGrab PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(get_picture_for_calib get_picture_for_calib.cpp visual_module.cpp find_handle.cpp control.cpp)
target_include_directories(get_picture_for_calib PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(get_picture_for_calib PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(camera_calibration camera_calibration.cpp)
target_include_directories(camera_calibration PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_calibration PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(joints_control_trackbar joints_control_trackbar.cpp find_handle.cpp visual_module.cpp)
target_include_directories(joints_control_trackbar PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(joints_control_trackbar PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(eye_in_hand_calibration eye_in_hand_calibration.cpp)
target_include_directories(eye_in_hand_calibration PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(eye_in_hand_calibration PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(123 123.cpp find_handle.cpp visual_module.cpp control.cpp)
target_include_directories(123 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(123 PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(ik IK.cpp)
target_include_directories(ik PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ik PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(inHandClib inHandClib.cpp)
target_include_directories(inHandClib PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(inHandClib PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(cmik cmik.cpp find_handle.cpp)
target_include_directories(cmik PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cmik PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(456 456.cpp)
target_include_directories(456 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(456 PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(789 789.cpp)
target_include_directories(789 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(789 PRIVATE RemoteAPIClient ${OpenCV_LIBS})

add_executable(to_hand_calibration to_hand_calibration.cpp)
target_include_directories(to_hand_calibration PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(to_hand_calibration PRIVATE RemoteAPIClient ${OpenCV_LIBS})

#find_package(OpenCV COMPONENTS core imgproc highgui)
if(OpenCV_FOUND)
    add_executable(opencv opencv.cpp)
    target_include_directories(opencv PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(opencv PRIVATE RemoteAPIClient)
    target_link_libraries(opencv PRIVATE ${OpenCV_LIBS})
endif()

find_package(Qt5 COMPONENTS Core Widgets Gui)
if(Qt5_FOUND)
    add_executable(qt qt.cpp)
    target_link_libraries(qt PRIVATE RemoteAPIClient)
    target_link_libraries(qt PRIVATE Qt5::Widgets)
endif()
