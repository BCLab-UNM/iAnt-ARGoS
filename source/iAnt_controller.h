#ifndef IANT_CONTROLLER_H_
#define IANT_CONTROLLER_H_

#include "iAnt_data.h"
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray3.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

#include <algorithm>    // std::reverse
#include <fstream>     
#include <vector>
#include <iterator>
#include <iostream>
#include <string>
#include <stdio.h>
#include <ctype.h>

using namespace argos;
using namespace std;

/*****
 * The brains of the iAnt robot. This controller object is a component of
 * each robot that is placed in the arena for experiments. The implementation
 * of the iAnt Central Place Foraging Algorithm (CPFA) is in this code.
 *****/
class iAnt_controller : public CCI_Controller {

    public:

        /* constructor and destructor */
        iAnt_controller();
        ~iAnt_controller() {}

        /* CCI_Controller Inherited Functions */
        void Init(TConfigurationNode& node);
        void ControlStep();
        void Reset();

        /* public helper functions */
        void       SetData(iAnt_data* dataPointer);
        bool       IsHoldingFood();
        iAnt_data* GetData();
        CVector2   GetPosition();
        CRadians   GetHeading();

        /* public variables */
        // extern int        levels;

    private:

        /* foot-bot components: sensors and actuators */
        CCI_PositioningSensor*            compass;
        CCI_DifferentialSteeringActuator* motorActuator;
        CCI_FootBotProximitySensor*       proximitySensor;

        /* data pipeline to qt_user_functions */
        Real       RobotForwardSpeed;
        Real       RobotTurningSpeed;

        /* robot internal variables & statistics */
        CRandom::CRNG*   RNG;
        iAnt_data*       data;
        CVector2         target;
        CVector2         newTarget;
        vector<char>     pattern;
        vector<char>     tempPattern;
        int              levels;
        float            stepSize;
        bool             isHoldingFood;
        bool             goingHome;
        CRange<CRadians> AngleToleranceInRadians;
        CRange<CRadians> Tolerance;
    
        /* movement functions */
        CDegrees angleInDegrees;
        void SetTargetN(char x);
        void SetTargetS(char x);
        void SetTargetE(char x);
        void SetTargetW(char x);
    
        /* movement helper functions */
        CVector2 GetTarget();
        void ApproachTheTarget();
        void ApproachTheTarget(CVector2 myTarget);
        void GetTargets();
        void ReadFile();
        void CopyPatterntoTemp();
        bool TargetHit();

        /*moving states */
        void SetHoldingFood();
        
    
};      

#endif /* IANT_CONTROLLER_H_ */