#ifndef CPFA_LOOP_FUNCTIONS_H
#define CPFA_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/CPFA/CPFA_controller.h>

class CPFA_loop_functions : public argos::CLoopFunctions {

    friend class CPFA_controller;
    friend class CPFA_qt_user_functions;

    public:

        CPFA_loop_functions();

        void Init(argos::TConfigurationNode &t_tree);
        void Reset();
        void PreStep();
        void PostStep();
        bool IsExperimentFinished();
        void PostExperiment();
        argos::CColor GetFloorColor(const argos::CVector2 &c_pos_on_floor);

        /* public helper functions */
        void UpdatePheromoneList();
        void SetFoodDistribution();

    protected:

        argos::CRandom::CRNG* RNG;

        size_t MaxSimTime;
        size_t ResourceDensityDelay;
        size_t RandomSeed;
        size_t SimCounter;
        size_t MaxSimCounter;
        size_t VariableFoodPlacement;
        size_t OutputData;
        size_t DrawDensityRate;
        size_t DrawIDs;
        size_t DrawTrails;
        size_t DrawTargetRays;
        size_t FoodDistribution;
        size_t FoodItemCount;
        size_t NumberOfClusters;
        size_t ClusterWidthX;
        size_t ClusterLengthY;
        size_t PowerRank;

        /* CPFA variables */
        argos::Real ProbabilityOfSwitchingToSearching;
        argos::Real ProbabilityOfReturningToNest;
        argos::CRadians UninformedSearchVariation;
        argos::Real RateOfInformedSearchDecay;
        argos::Real RateOfSiteFidelity;
        argos::Real RateOfLayingPheromone;
        argos::Real RateOfPheromoneDecay;

        /* physical robot & world variables */
        argos::Real FoodRadius;
        argos::Real FoodRadiusSquared;
        argos::Real NestRadius;
        argos::Real NestRadiusSquared;
        argos::Real NestElevation;
        argos::Real SearchRadiusSquared;

        /* list variables for food & pheromones */
        std::vector<argos::CVector2> FoodList;
        std::vector<argos::CColor>   FoodColoringList;
        std::vector<argos::CVector2> FidelityList;
        std::vector<iAntPheromone>   PheromoneList;
        std::vector<argos::CRay3>    TargetRayList;
        argos::CRange<argos::Real>   ForageRangeX;
        argos::CRange<argos::Real>   ForageRangeY;

        argos::CVector2 NestPosition;

    private:

        /* private helper functions */
        void RandomFoodDistribution();
        void ClusterFoodDistribution();
        void PowerLawFoodDistribution();
        bool IsOutOfBounds(argos::CVector2 p, size_t length, size_t width);
        bool IsCollidingWithNest(argos::CVector2 p);
        bool IsCollidingWithFood(argos::CVector2 p);

};

#endif /* CPFA_LOOP_FUNCTIONS_H */