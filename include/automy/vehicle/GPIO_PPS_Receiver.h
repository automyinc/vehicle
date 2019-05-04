/*
 * GPIO_PPS_Receiver.h
 *
 *  Created on: Jan 25, 2018
 *      Author: mad
 */

#ifndef VEHICLE_INCLUDE_VEHICLE_GPIO_PPS_RECEIVER_H_
#define VEHICLE_INCLUDE_VEHICLE_GPIO_PPS_RECEIVER_H_

#include <automy/vehicle/GPIO_PPS_ReceiverBase.hxx>


namespace automy {
namespace vehicle {

class GPIO_PPS_Receiver : public GPIO_PPS_ReceiverBase {
public:
	GPIO_PPS_Receiver(const std::string& _vnx_name);
	
protected:
	void main() override;
	
};


} // vehicle
} // automy

#endif /* VEHICLE_INCLUDE_VEHICLE_GPIO_PPS_RECEIVER_H_ */
