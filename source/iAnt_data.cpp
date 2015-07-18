#include "iAnt_data.h"

/*****
 *
 *****/
iAnt_data::iAnt_data() :
    SimTime(0),
    MaxSimTime(57600),
    TicksPerSecond(16),
    ResourceDensityDelay(0),
    RandomSeed(1337),
    RNG(NULL),
    SimCounter(0),
    MaxSimCounter(1),
    VariableSeed(0),
    OutputData(0),
    FoodDistribution(0),

    FoodItemCount(256),
    NumberOfClusters(4),
    ClusterWidthX(8),
    ClusterLengthY(8),
    PowerRank(4),

    NestRadius(0.25),
    NestRadiusSquared(0.0625),
    NestElevation(0.01),

    FoodRadius(0.05),
    FoodRadiusSquared(0.0025),

    ForageRangeX(-10.0, 10.0),
    ForageRangeY(-10.0, 10.0),
    
    DrawTargetRays(1)
{}

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
    Real     foodOffset  = 3.0 * FoodRadius;
    size_t   foodToPlace = NumberOfClusters * ClusterWidthX * ClusterLengthY;
    CVector2 placementPosition;

    FoodItemCount = foodToPlace;

    for(size_t i = 0; i < NumberOfClusters; i++) {
        placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

        while(IsOutOfBounds(placementPosition, ClusterLengthY, ClusterWidthX)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
        }

        for(size_t j = 0; j < ClusterLengthY; j++) {
            for(size_t k = 0; k < ClusterWidthX; k++) {
                FoodList.push_back(placementPosition);
                placementPosition.SetX(placementPosition.GetX() + foodOffset);
            }

            placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
            placementPosition.SetY(placementPosition.GetY() + foodOffset);
        }
    }
}

/*****
 *
 *****/
void iAnt_data::PowerLawFoodDistribution() {
    Real   foodOffset     = 3.0 * FoodRadius;
    size_t foodPlaced     = 0;
    size_t powerLawLength = 1;
    size_t maxTrials      = 200;
    size_t trialCount     = 0;

    vector<size_t> powerLawClusters;
    vector<size_t> clusterSides;
    CVector2       placementPosition;

    for(size_t i = 0; i < PowerRank; i++) {
        powerLawClusters.push_back(powerLawLength * powerLawLength);
        powerLawLength *= 2;
    }

    for(size_t i = 0; i < PowerRank; i++) {
        powerLawLength /= 2;
        clusterSides.push_back(powerLawLength);
    }

    for(size_t h = 0; h < powerLawClusters.size(); h++) {
        for(size_t i = 0; i < powerLawClusters[h]; i++) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

            while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
                trialCount++;
                placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

                if(trialCount > maxTrials) {
                    LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
                    break;
                }
            }

            for(size_t j = 0; j < clusterSides[h]; j++) {
                for(size_t k = 0; k < clusterSides[h]; k++) {
                    foodPlaced++;
                    FoodList.push_back(placementPosition);
                    placementPosition.SetX(placementPosition.GetX() + foodOffset);
                }

                placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
                placementPosition.SetY(placementPosition.GetY() + foodOffset);
            }
        }
    }

    FoodItemCount = foodPlaced;
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
