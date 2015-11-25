#ifndef DSA_CONTROLLER_H
#define DSA_CONTROLLER_H

#include <source/iAntBase/iAntBaseController.h>
#include <source/DSA/DSA_loop_functions.h>

using namespace argos;
using namespace std;

class DSA_loop_functions;

class DSA_controller : public iAntBaseController {

    public:

        DSA_controller();

        // CCI_Controller inheritence functions
        void Init(TConfigurationNode& node);
        void ControlStep();
        void Reset();

        bool   IsHoldingFood();
        void   GetPattern(string ith_Pattern);
        void   SetRobotPath(string path);
		size_t generatePattern(int N_circuits, int N_robots);
		int    calcDistanceToTravel(int i_robot, int i_circuit, int N_robots, char direction);
		void   writePatternToFile(vector<char>&, int N_robots);
		void   addDirectionToPattern(char direction);
		void   printPath(vector<char>&);

		void SetLoopFunctions(DSA_loop_functions* lf) { loopFunctions = lf; }

    private:

        size_t NumberOfRobots;
        size_t NumberOfSpirals;

        /* iAnt DSA state variable */
        enum DSA { SEARCHING = 1, RETURN_TO_NEST = 2, RETURN_TO_SEARCH = 3 } DSA;

        /* robot internal variables & statistics */
        CRandom::CRNG*      RNG;
        DSA_loop_functions* loopFunctions;

        CVector2            ReturnPosition;

        vector<CRay3>       myTrail;
        CColor              TrailColor;

        Real                SearcherGap;
        Real                FoodDistanceTolerance;
        size_t				collisionCounter;
        CVector2            newTarget;
        CVector3            startPosition;
        vector<char>        pattern;
        vector<char>        tempPattern;
        vector<string>      rPattern;
        int                 levels;
        bool                isHoldingFood;
        bool                goingHome;
        bool                ResetReturnPosition;
        CRange<CRadians>    AngleToleranceInRadians;
        CRange<CRadians>    Tolerance;
        size_t              stopTimeStep;
        size_t              collisionDelay;
    
        /* movement functions */
        CDegrees angleInDegrees;

        void SetTargetN(char x);
        void SetTargetS(char x);
        void SetTargetE(char x);
        void SetTargetW(char x);
    
        /* movement helper functions */
        void GetTargets();
        void CopyPatterntoTemp();
        bool TargetHit();
        void SetHoldingFood(); 

};

#endif /* DSA_CONTROLLER_H */