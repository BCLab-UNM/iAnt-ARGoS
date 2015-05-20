#include "iAnt_data.h"

/*****
 *
 *****/
iAnt_data::iAnt_data() :

    SimTime(0),
    TicksPerSecond(16),
    RandomSeed(1337),
    VariableSeed(0),
    OutputData(0),
    SimCounter(0),
    TrailDensityRate(12),
    DrawTrails(0),
    FoodDistribution(0),

    FoodItemCount(256),
    NumberOfClusters(4),
    ClusterWidthX(8),
    ClusterLengthY(8),
    PowerRank(4),

    ProbabilityOfSwitchingToSearching(0.99999),
    ProbabilityOfReturningToNest(0.00001),
    UninformedSearchVariation(0.231256126),
    RateOfInformedSearchDecay(0.05),
    RateOfSiteFidelity(10.0),
    RateOfLayingPheromone(10.0),
    RateOfPheromoneDecay(0.05),

    TurnProbability(0.1),
    PushProbability(0.5),
    PullProbability(0.1),
    WaitProbability(0.1),

    NestRadius(0.25),
    NestRadiusSquared(0.0625),
    NestElevation(0.01),

    FoodRadius(0.05),
    FoodRadiusSquared(0.0025),

    ForageRangeX(-2.25, 2.25),
    ForageRangeY(-2.25, 2.25)

{}

/*****
 *
 *****/
void iAnt_data::UpdatePheromoneList() {
 
   vector<iAnt_pheromone> new_p_list;

	for(size_t i = 0; i < PheromoneList.size(); i++) {

        PheromoneList[i].Update(SimTime / TicksPerSecond);

        if(PheromoneList[i].IsActive() == true) {
            new_p_list.push_back(PheromoneList[i]);
        }
    }

    PheromoneList = new_p_list;
}

/*****
 *
 *****/
void iAnt_data::SetFoodDistribution() {
    switch(FoodDistribution) {
        case 0:
            RandomFoodDistribution();
            break;
        case 1:
            ClusterFoodDistribution();
            break;
        case 2:
            PowerLawFoodDistribution();
            break;
        default:
            LOGERR << "ERROR: Invalid food distribution in XML file.\n";
    }
}

/*****
 *
 *****/
void iAnt_data::RandomFoodDistribution() {
    FoodList.clear();

    CVector2 placementPosition;

    for(size_t i = 0; i < FoodItemCount; i++) {
        placementPosition.Set(RNG->Uniform(ForageRangeX),
                              RNG->Uniform(ForageRangeY));

        while(IsOutOfBounds(placementPosition, 1, 1)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX),
                                  RNG->Uniform(ForageRangeY));
        }

        FoodList.push_back(placementPosition);
    }
}

/*****
 *
 *****/
void iAnt_data::ClusterFoodDistribution() {
}

/*****
 *
 *****/
void iAnt_data::PowerLawFoodDistribution() {
}

/*****
 *
 *****/
bool iAnt_data::IsOutOfBounds(CVector2 p, size_t length, size_t width) {
    CVector2 placementPosition = p;

    Real foodOffset   = 3.0 * FoodRadius;
    Real widthOffset  = 3.0 * FoodRadius * (Real)width;
    Real lengthOffset = 3.0 * FoodRadius * (Real)length;

    Real x_min = p.GetX() - FoodRadius;
    Real x_max = p.GetX() + FoodRadius + widthOffset;

    Real y_min = p.GetY() - FoodRadius;
    Real y_max = p.GetY() + FoodRadius + lengthOffset;

    if((x_min < (ForageRangeX.GetMin() + FoodRadius)) ||
       (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
       (y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
       (y_max > (ForageRangeY.GetMax() - FoodRadius))) {
        return true;
    }

    for(size_t j = 0; j < length; j++) {
        for(size_t k = 0; k < width; k++) {
            if(IsCollidingWithFood(placementPosition)) return true;
            if(IsCollidingWithNest(placementPosition)) return true;
            placementPosition.SetX(placementPosition.GetX() + foodOffset);
        }

        placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
        placementPosition.SetY(placementPosition.GetY() + foodOffset);
    }

    return false;
}

/*****
 *
 *****/
bool iAnt_data::IsCollidingWithNest(CVector2 p) {
    Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
    Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

    return ((p - NestPosition).SquareLength() < NRPB_squared);
}

/*****
 *
 *****/
bool iAnt_data::IsCollidingWithFood(CVector2 p) {
    Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
    Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

    for(size_t i = 0; i < FoodList.size(); i++) {
        if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
    }

    return false;
}
