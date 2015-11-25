/*****
 * This file contains the Algorithm for DSA (Determinstic Spider Algorithm)
 * This file is extension of the matlab version created by Matthew Fricke and Linh Tran
 * on 8/11/15. This version interprets patterns for multiple robots preforming
 * the DSA algorithm. 
 * Authors: Linh Tran and Matthew Fricke
 * Date: 09/29/15
 *****/

#ifndef DSA_CONTROLLER_H_
#define DSA_CONTROLLER_H_

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray3.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <source/iAnt_loop_functions.h>

/*
#include <algorithm>    // std::reverse
#include <fstream>     
#include <vector>
#include <iterator>
#include <iostream>
#include <string>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
*/

using namespace argos;
using namespace std;

class iAnt_loop_functions;

/*****
 * The brains of the iAnt robot. This controller object is a component of
 * each robot that is placed in the arena for experiments. The implementation
 * of the iAnt Central Place Foraging Algorithm (CPFA) is in this code.
 *****/
class DSA_controller : public CCI_Controller, public CLoopFunctions {

    public:

        /* constructor and destructor */
        DSA_controller();
        virtual ~DSA_controller() {}

        /* CCI_Controller Inherited Functions */
        void Init(TConfigurationNode& node);
        void ControlStep();
        void Reset();
        void GetPattern(string ith_Pattern);

        /* public helper functions */
        bool       IsHoldingFood();
        CVector2   GetPosition();
        CVector3   GetStartPosition();
        CRadians   GetHeading();

        /* robot intial start pause */
        bool Stop();
        void SetStop(size_t stop_time_in_seconds);

        /* set robots paths */
        void SetRobotPath(string path);

        /* DSA path creation functions */
		size_t generatePattern(int N_circuits, int N_robots);
		int    calcDistanceToTravel(int i_robot, int i_circuit, int N_robots, char direction);
		void   writePatternToFile(vector<char>&, int N_robots);
		void   addDirectionToPattern(char direction);
		void   printPath(vector<char>&);

        int getNumberOfSpirals();

    private:

        /* foot-bot components: sensors and actuators */
        CCI_PositioningSensor*            compass;
        CCI_DifferentialSteeringActuator* motorActuator;
        CCI_FootBotProximitySensor*       proximitySensor;

        /* data pipeline to qt_user_functions */
        Real       RobotForwardSpeed;
        Real       RobotTurningSpeed;
        size_t     NumberOfRobots;
        size_t     NumberOfSpirals;

        /* iAnt DSA state variable */
        enum DSA { SEARCHING = 1, RETURNING = 2 } DSA;

        /* robot internal variables & statistics */
        CRandom::CRNG*       RNG;
        iAnt_loop_functions& loopFunctions;
        vector<CRay3>        myTrail;
        CVector2             target;
        size_t				 collisionCounter;
        CVector2             newTarget;
        CVector3             startPosition;
        vector<char>         pattern;
        vector<char>         tempPattern;
        vector<string>       rPattern;
        int                  levels;
        float                stepSize;
        bool                 isHoldingFood;
        bool                 goingHome;
        CRange<CRadians>     AngleToleranceInRadians;
        CRange<CRadians>     Tolerance;
        size_t               stopTimeStep;
        size_t               collisionDelay;
    
        /* movement functions */
        CDegrees angleInDegrees;
        void SetTargetN(char x);
        void SetTargetS(char x);
        void SetTargetE(char x);
        void SetTargetW(char x);
    
        /* movement helper functions */
        CVector2 GetTarget();
        CRadians GetCollisionHeading();
        bool     IsCollisionDetected();
        void ApproachTheTarget();
        void ApproachTheTarget(CVector2 myTarget);

        void GetTargets();
        void CopyPatterntoTemp();
        bool TargetHit();

        /*moving states */
        void SetHoldingFood(); 
};      

#endif /* DSA_CONTROLLER_H_ */