/*
 * SensorsAndActuators.h
 *
 *  Created on: Aug 11, 2014
 *      Author: antonio
 */

#ifndef SENSORSANDACTUATORS_H_
#define SENSORSANDACTUATORS_H_

/* Argos3 objects for robot components: actuators and sensors. (plug-ins) */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>

using namespace argos;

class SensorsAndActuators {
public:
	/* robot actuator and sensor components */
	CCI_DifferentialSteeringActuator *steeringActuator; // controls the robot's motor speeds
	CCI_FootBotProximitySensor       *proximitySensor;  // detects nearby objects to prevent collision
	CCI_FootBotMotorGroundSensor     *groundSensor;     // detects food items & nest (color changes)
	CCI_FootBotLightSensor           *lightSensor;      // detects nest-light for navigation control

	SensorsAndActuators();
	~SensorsAndActuators();
};

#endif /* SENSORSANDACTUATORS_H_ */
