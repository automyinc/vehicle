cmake_minimum_required(VERSION 2.6)
project(automy-vehicle)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -pthread")

include_directories(
	/usr/include/automy/
	/usr/local/include/automy/
)

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

install(DIRECTORY include/ DESTINATION include/automy)
install(DIRECTORY generated/include/ DESTINATION include/automy)
install(DIRECTORY interface/ DESTINATION interface/automy/vehicle)

install(TARGETS automy_vehicle DESTINATION lib)
