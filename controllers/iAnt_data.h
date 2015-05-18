#ifndef IANT_DATA_H_
#define IANT_DATA_H_

#include "iAnt_pheromone.h"
#include <vector>
#include <argos3/core/utility/math/vector2.h>

using namespace std;
using namespace argos;

class iAnt_data {
    public:

        iAnt_data();

        /* iAnt simulation data */
        size_t simTime;
        size_t ticks_per_second;
        size_t trailDensityRate;
        bool   drawTrails;

        /* CPFA variables and settings */
        Real     probabilityOfSwitchingToSearching;
        Real     probabilityOfReturningToNest;
        CRadians uninformedSearchVariation;
        Real     rateOfInformedSearchDecay;
        Real     rateOfSiteFidelity;
        Real     rateOfLayingPheromone;
        Real     rateOfPheromoneDecay;

        /* iAnt_controller data */
        Real                   turnProbability;
        Real                   pushProbability;
        Real                   pullProbability;
        Real                   waitProbability;
        Real                   nestRadius;
        Real                   nestRadiusSquared;
        Real                   nestElevation;
        CRange<Real>           forageRangeX;
        CRange<Real>           forageRangeY;
        CVector2               nestPosition;
        vector<CVector2>       foodList;
        vector<CVector2>       fidelityList;
        vector<iAnt_pheromone> pheromoneList;
};

#endif /* IANT_DATA_H_ */
