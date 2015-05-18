#include "iAnt_data.h"

iAnt_data::iAnt_data() :

    simTime(0),
    ticks_per_second(16),
    trailDensityRate(12),
    drawTrails(true),

    probabilityOfSwitchingToSearching(0.99999),
    probabilityOfReturningToNest(0.00001),
    uninformedSearchVariation(0.231256126),
    rateOfInformedSearchDecay(0.01),
    rateOfSiteFidelity(10.0),
    rateOfLayingPheromone(10.0),
    rateOfPheromoneDecay(0.01),

    turnProbability(0.1),
    pushProbability(0.5),
    pullProbability(0.1),
    waitProbability(0.1),

    nestRadius(0.25),
    nestRadiusSquared(0.0625),
    nestElevation(0.01),

    forageRangeX(-2.25, 2.25),
    forageRangeY(-2.25, 2.25)
{}
