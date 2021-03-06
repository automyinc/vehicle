#!/bin/bash

SRC=$1

cp $SRC/interface/Odometry.vni interface/
cp $SRC/interface/GPS_Info.vni interface/
cp $SRC/interface/IMU_Info.vni interface/
cp $SRC/interface/PPS_Signal.vni interface/
cp $SRC/interface/UBX_Packet.vni interface/
cp $SRC/interface/VehicleInfo.vni interface/
cp $SRC/interface/VehicleDimensions.vni interface/
cp $SRC/interface/WheelSpeed.vni interface/

cp $SRC/modules/UbloxReceiver.vni modules/
cp $SRC/modules/GPIO_PPS_Receiver.vni modules/

cp $SRC/include/automy/vehicle/UbloxReceiver.h include/automy/vehicle/
cp $SRC/include/automy/vehicle/GPIO_PPS_Receiver.h include/automy/vehicle/

cp $SRC/src/UbloxReceiver.cpp src/
cp $SRC/src/GPIO_PPS_Receiver.cpp src/
cp -r $SRC/src/ublox src/

