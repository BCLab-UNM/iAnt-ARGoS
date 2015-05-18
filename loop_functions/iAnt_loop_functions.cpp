#include "iAnt_loop_functions.h"

iAnt_loop_functions::iAnt_loop_functions() {}

void iAnt_loop_functions::Init(TConfigurationNode& node) {
    /* initialize objects and variables */
    RNG = CRandom::CreateRNG("argos");

    /* set the nest position */
    data.nestPosition = CVector2(0.0, 0.0);

    /* set the reference to the ticks per second of the simulation */
    CPhysicsEngine* pEngine = &GetSimulator().GetPhysicsEngine("default");
    data.ticks_per_second = pEngine->GetInverseSimulationClockTick();

    /* store the iAnts in a more friendly, human-readable structure */
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator it;
    for(it = footbots.begin(); it != footbots.end(); it++) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());
        iAnts.push_back(&c);
        c.SetData(&data); // give each iAnt a pointer to the iAnt_data object
    }
}

void iAnt_loop_functions::PreStep() {
    data.simTime++;

    ////////////////////////////////////////////////////////////////////////////

    vector<iAnt_pheromone> new_p_list;

	for(size_t i = 0; i < data.pheromoneList.size(); i++) {
        data.pheromoneList[i].Update(data.simTime/data.ticks_per_second);
        if(data.pheromoneList[i].IsActive() == true) {
            new_p_list.push_back(data.pheromoneList[i]);
        }
    }

    data.pheromoneList = new_p_list;

    ////////////////////////////////////////////////////////////////////////////

    if(data.foodList.size() <= 64) {

        Real x = 0.0, y = 0.0;
        size_t attempts = 0, clusterSize = (size_t)RNG->Uniform(CRange<int>(0, 5));
        bool isPlaced = false;
        CVector2 newFood;

        while(isPlaced == false) {
            attempts++;
            isPlaced = true;
            x = RNG->Uniform(data.forageRangeX);
            y = RNG->Uniform(data.forageRangeY);

            for(size_t i = 0; i < data.foodList.size(); i++) {
                if((CVector2(x, y) - data.foodList[i]).SquareLength() < data.distanceTolerance + 0.05)
                    isPlaced = false;
            }

            if((CVector2(x, y) - data.nestPosition).SquareLength() < data.nestRadiusSquared + 0.05)
                isPlaced = false;
            else if(attempts > 10) isPlaced = true;
        }

        newFood = CVector2(x, y);

        for(size_t i = 0; i < clusterSize; i++) {
            x = newFood.GetX();

            for(size_t j = 0; j < clusterSize; j++) {
                x += 3.0 * 0.05;

                if(x <= data.forageRangeX.GetMax() && x >= data.forageRangeX.GetMin() &&
                   y <= data.forageRangeY.GetMax() && y >= data.forageRangeY.GetMin() &&
                   (CVector2(x,y) - data.nestPosition).SquareLength() > data.nestRadiusSquared + 0.05)
                    data.foodList.push_back(CVector2(x, y));
            }

            y += 3.0 * 0.05;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
}

REGISTER_LOOP_FUNCTIONS(iAnt_loop_functions, "iAnt_loop_functions")
