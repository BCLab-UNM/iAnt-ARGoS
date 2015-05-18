#include "iAnt_data.h"

iAnt_data::iAnt_data() :

    simTime(0),
    ticks_per_second(16),
    trailDensityRate(12),
    drawTrails(true),

    /* CPFA parameters */
    probabilityOfSwitchingToSearching(0.9999),
    probabilityOfReturningToNest(0.0001),
    uninformedSearchVariation(0.231256126),
    rateOfInformedSearchDecay(0.01),
    rateOfSiteFidelity(0.0),
    rateOfLayingPheromone(0.0),
    rateOfPheromoneDecay(0.025),

    robotSpeed(16.0),
    turnProbability(0.1),
    pushProbability(0.5),
    pullProbability(0.1),
    waitProbability(0.1),
    distanceTolerance(0.01),
    searchStepSize(0.08),
    searchRadiusSquared(1.0),

    nestRadius(0.25),
    nestRadiusSquared(0.0625),
    nestElevation(0.01),

    angleTolerance(CRadians(-0.261799388), CRadians(0.261799388)),
    forageRangeX(-2.25, 2.25),
    forageRangeY(-2.25, 2.25)
{}
