#include "CPFA_loop_functions.h"

CPFA_loop_functions::CPFA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
    MaxSimTime(3600 * GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick()),
    ResourceDensityDelay(0),
    RandomSeed(GetSimulator().GetRandomSeed()),
    SimCounter(0),
    MaxSimCounter(1),
    VariableFoodPlacement(0),
    OutputData(0),
    DrawDensityRate(4),
    DrawIDs(1),
    DrawTrails(1),
    DrawTargetRays(0),
    FoodDistribution(2),
    FoodItemCount(256),
    NumberOfClusters(4),
    ClusterWidthX(8),
    ClusterLengthY(8),
    PowerRank(4),
    ProbabilityOfSwitchingToSearching(0.0),
    ProbabilityOfReturningToNest(0.0),
    UninformedSearchVariation(0.0),
    RateOfInformedSearchDecay(0.0),
    RateOfSiteFidelity(0.0),
    RateOfLayingPheromone(0.0),
    RateOfPheromoneDecay(0.0),
    FoodRadius(0.05),
    FoodRadiusSquared(0.0025),
    NestRadius(0.25),
    NestRadiusSquared(0.0625),
    NestElevation(0.01),
    SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius))
{}

void CPFA_loop_functions::Init(argos::TConfigurationNode &node) {

    argos::CDegrees USV_InDegrees;

    argos::TConfigurationNode CPFA_node = argos::GetNode(node, "CPFA");
    argos::GetNodeAttribute(CPFA_node, "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
    argos::GetNodeAttribute(CPFA_node, "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
    argos::GetNodeAttribute(CPFA_node, "UninformedSearchVariation",         USV_InDegrees);
    argos::GetNodeAttribute(CPFA_node, "RateOfInformedSearchDecay",         RateOfInformedSearchDecay);
    argos::GetNodeAttribute(CPFA_node, "RateOfSiteFidelity",                RateOfSiteFidelity);
    argos::GetNodeAttribute(CPFA_node, "RateOfLayingPheromone",             RateOfLayingPheromone);
    argos::GetNodeAttribute(CPFA_node, "RateOfPheromoneDecay",              RateOfPheromoneDecay);

    UninformedSearchVariation = ToRadians(USV_InDegrees);

    // calculate the forage range and compensate for the robot's radius of 0.085m
    argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
    argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085;
    argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085;
    ForageRangeX.Set(-rangeX, rangeX);
    ForageRangeY.Set(-rangeY, rangeY);

    // Send a pointer to this loop functions object to each controller.
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    argos::CSpace::TMapPerType::iterator it;

    for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        iAntBaseController& c = dynamic_cast<iAntBaseController&>(footBot.GetControllableEntity().GetController());
        CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);

        c2.SetLoopFunctions(this);
	}

	SetFoodDistribution();
}

void CPFA_loop_functions::Reset() {
	if(VariableFoodPlacement == 0) {
		RNG->Reset();
	}

	GetSpace().Reset();
	GetSpace().GetFloorEntity().Reset();
	MaxSimCounter = SimCounter;
	SimCounter = 0;

	FoodList.clear();
	FoodColoringList.clear();
	PheromoneList.clear();
	FidelityList.clear();
	TargetRayList.clear();

	// SetFoodDistribution();
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    argos::CSpace::TMapPerType::iterator it;

    for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        iAntBaseController& c = dynamic_cast<iAntBaseController&>(footBot.GetControllableEntity().GetController());
        CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);

        MoveEntity(footBot.GetEmbodiedEntity(), c2.GetStartPosition(), argos::CQuaternion(), false);
	}
}

void CPFA_loop_functions::PreStep() {
	UpdatePheromoneList();

	if(GetSpace().GetSimulationClock() > ResourceDensityDelay) {
		for(size_t i = 0; i < FoodColoringList.size(); i++) {
			FoodColoringList[i] = argos::CColor::BLACK;
		}
	}

	if(FoodList.size() == 0) {
		FidelityList.clear();
		TargetRayList.clear();
		PheromoneList.clear();
	}
}

void CPFA_loop_functions::PostStep() {
	// nothing... yet...
}

bool CPFA_loop_functions::IsExperimentFinished() {
    bool isFinished = false;

    if(FoodList.size() == 0 || GetSpace().GetSimulationClock() >= MaxSimTime) {
        isFinished = true;
    }

    if(isFinished == true && MaxSimCounter > 1) {
        size_t newSimCounter = SimCounter + 1;
        size_t newMaxSimCounter = MaxSimCounter - 1;

        PostExperiment();
        Reset();

        SimCounter    = newSimCounter;
        MaxSimCounter = newMaxSimCounter;
        isFinished    = false;
    }

    return isFinished;
}

void CPFA_loop_functions::PostExperiment() {}

argos::CColor CPFA_loop_functions::GetFloorColor(const argos::CVector2 &c_pos_on_floor) {
	return argos::CColor::WHITE;
}

/*****
 *
 *****/
void CPFA_loop_functions::UpdatePheromoneList() {
 
    std::vector<iAntPheromone> new_p_list;
    argos::Real t = GetSpace().GetSimulationClock() / GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick();

    for(size_t i = 0; i < PheromoneList.size(); i++) {

        PheromoneList[i].Update(t);

        if(PheromoneList[i].IsActive() == true) {
            new_p_list.push_back(PheromoneList[i]);
        }
    }

    PheromoneList = new_p_list;
}

/*****
 *
 *****/
void CPFA_loop_functions::SetFoodDistribution() {
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
            argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
    }
}

/*****
 *
 *****/
void CPFA_loop_functions::RandomFoodDistribution() {
    FoodList.clear();

    argos::CVector2 placementPosition;

    for(size_t i = 0; i < FoodItemCount; i++) {
        placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

        while(IsOutOfBounds(placementPosition, 1, 1)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
        }

        FoodList.push_back(placementPosition);
        FoodColoringList.push_back(argos::CColor::BLACK);
    }
}

/*****
 *
 *****/
void CPFA_loop_functions::ClusterFoodDistribution() {

    argos::Real     foodOffset  = 3.0 * FoodRadius;
    size_t          foodToPlace = NumberOfClusters * ClusterWidthX * ClusterLengthY;
    size_t          foodPlaced = 0;
    argos::CVector2 placementPosition;

    FoodItemCount = foodToPlace;

    for(size_t i = 0; i < NumberOfClusters; i++) {
        placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

        while(IsOutOfBounds(placementPosition, ClusterLengthY, ClusterWidthX)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
        }

        for(size_t j = 0; j < ClusterLengthY; j++) {
            for(size_t k = 0; k < ClusterWidthX; k++) {

                foodPlaced++;
                /*
                #include <argos3/plugins/simulator/entities/box_entity.h>

                string label("my_box_");
                label.push_back('0' + foodPlaced++);

                CBoxEntity *b = new CBoxEntity(label,
                                               CVector3(placementPosition.GetX(), placementPosition.GetY(), 0.0),
                                               CQuaternion(),
                                               true,
                                               CVector3(0.1, 0.1, 0.001),
                                               1.0);
                AddEntity(*b);
                */

                FoodList.push_back(placementPosition);
                FoodColoringList.push_back(argos::CColor::BLACK);
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
void CPFA_loop_functions::PowerLawFoodDistribution() {
    argos::Real foodOffset     = 3.0 * FoodRadius;
    size_t      foodPlaced     = 0;
    size_t      powerLawLength = 1;
    size_t      maxTrials      = 200;
    size_t      trialCount     = 0;

    std::vector<size_t> powerLawClusters;
    std::vector<size_t> clusterSides;
    argos::CVector2     placementPosition;

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
                    argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
                    break;
                }
            }

            for(size_t j = 0; j < clusterSides[h]; j++) {
                for(size_t k = 0; k < clusterSides[h]; k++) {
                    foodPlaced++;
                    FoodList.push_back(placementPosition);
                    FoodColoringList.push_back(argos::CColor::BLACK);
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
bool CPFA_loop_functions::IsOutOfBounds(argos::CVector2 p, size_t length, size_t width) {
    argos::CVector2 placementPosition = p;

    argos::Real foodOffset   = 3.0 * FoodRadius;
    argos::Real widthOffset  = 3.0 * FoodRadius * (argos::Real)width;
    argos::Real lengthOffset = 3.0 * FoodRadius * (argos::Real)length;

    argos::Real x_min = p.GetX() - FoodRadius;
    argos::Real x_max = p.GetX() + FoodRadius + widthOffset;

    argos::Real y_min = p.GetY() - FoodRadius;
    argos::Real y_max = p.GetY() + FoodRadius + lengthOffset;

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
bool CPFA_loop_functions::IsCollidingWithNest(argos::CVector2 p) {
    argos::Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
    argos::Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

    return ((p - NestPosition).SquareLength() < NRPB_squared);
}

/*****
 *
 *****/
bool CPFA_loop_functions::IsCollidingWithFood(argos::CVector2 p) {
    argos::Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
    argos::Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

    for(size_t i = 0; i < FoodList.size(); i++) {
        if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
    }

    return false;
}

REGISTER_LOOP_FUNCTIONS(CPFA_loop_functions, "CPFA_loop_functions")