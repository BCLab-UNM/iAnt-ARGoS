#include "iAnt_qt_user_functions.h"

// constructor
iAnt_qt_user_functions::iAnt_qt_user_functions() :
    foodRadius(0.0),
    nestRadius(0.0)
{
    // register and connect these drawing functions to Argos
    RegisterUserFunction<iAnt_qt_user_functions, CFootBotEntity>(&iAnt_qt_user_functions::DrawFood);
}

// draw function for adding graphics to a foot-bot
void iAnt_qt_user_functions::DrawFood(CFootBotEntity& entity) {
    // foot-bot controller object
    iAnt_controller& c = dynamic_cast<iAnt_controller&>(entity.GetControllableEntity().GetController());

    foodRadius = c.GetFoodRadius();
    nestRadius = c.GetNestRadius();

    nestPosition = c.GetNestPosition();
    foodPositions = c.GetFoodPositions();

    vector<CVector2> p = c.GetPheromonePositions();

    for(int i = 0; i < p.size(); i++) {
        pheromonePositions.push_back(p[i]);
    }

    fidelityPositions.push_back(c.FidelityPosition());

    // if the foot-bot has a food item, draw it on top of the foot-bot
    if(c.IsHoldingFood()) {
        // these hard-wired values are valid for the foot-bot only,
        // and must be changed if different robot graphics are used
        DrawCylinder(0.05f, 0.025f, CVector3(0.0f, 0.0f, 0.3f), CColor::BLACK);
    }
}

void iAnt_qt_user_functions::DrawInWorld() {
    // draw the nest
    DrawCircle(nestRadius,
               CVector3(nestPosition.GetX(), nestPosition.GetY(), 0.001f),
               CColor::GRAY80);

    // draw the food items
    for(int i = 0; i < foodPositions.size(); i++) {
        DrawCircle(foodRadius,
          CVector3(foodPositions[i].GetX(), foodPositions[i].GetY(), 0.002f),
                   CColor::BLACK);
    }

    if(foodPositions.size() > 0) {
        // draw the pheromone positions
        for(int i = 0; i < pheromonePositions.size(); i++) {
            DrawCircle(foodRadius,
              CVector3(pheromonePositions[i].GetX(), pheromonePositions[i].GetY(), 0.003f),
                       CColor::RED);
        }

        // draw the fidelity positions
        for(int i = 0; i < fidelityPositions.size(); i++) {
            if(fidelityPositions[i] != nestPosition) {
                DrawCircle(foodRadius,
                           CVector3(fidelityPositions[i].GetX(), fidelityPositions[i].GetY(), 0.004f),
                           CColor::BLUE);
            }
        }
    }

    pheromonePositions.clear();
    fidelityPositions.clear();
}

// macro required for registering this class with Argos
REGISTER_QTOPENGL_USER_FUNCTIONS(iAnt_qt_user_functions, "iAnt_qt_user_functions")
