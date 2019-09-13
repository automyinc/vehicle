cmake_minimum_required(VERSION 2.6)
project(automy-vehicle)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -pthread")

file(GLOB GENERATED_SOURCE_FILES generated/src/*.cpp)

add_library(automy_vehicle SHARED
	${GENERATED_SOURCE_FILES}
	src/UbloxReceiver.cpp
	src/GPIO_PPS_Receiver.cpp
)

target_include_directories(automy_vehicle
	PUBLIC include generated/include
	PRIVATE src/
)

target_link_libraries(automy_vehicle automy_basic automy_math vnx_base)

add_executable(example_ublox_receiver example/ublox_receiver.cpp)
target_link_libraries(example_ublox_receiver automy_vehicle)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY generated/include/ DESTINATION include)
install(DIRECTORY interface/ DESTINATION interface/automy/vehicle)

install(TARGETS automy_vehicle DESTINATION lib)
