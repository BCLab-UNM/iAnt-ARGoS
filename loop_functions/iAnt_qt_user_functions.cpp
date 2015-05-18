#include "iAnt_qt_user_functions.h"

// constructor
iAnt_qt_user_functions::iAnt_qt_user_functions() :
    foodRadius(0.0),
    nestRadius(0.0),
    loopFunctions(NULL)
{
    // register and connect these drawing functions to Argos
    RegisterUserFunction<iAnt_qt_user_functions, CFootBotEntity>
                        (&iAnt_qt_user_functions::DrawFood);
}

void iAnt_qt_user_functions::UpdateDrawInWorldData(iAnt_controller& c) {
    loopFunctions = c.GetLoopFunctions();
    foodRadius    = c.GetFoodRadius();
    nestRadius    = c.GetNestRadius();
    nestPosition  = c.GetNestPosition();
}


// draw function for adding graphics to a foot-bot
void iAnt_qt_user_functions::DrawFood(CFootBotEntity& entity) {
    iAnt_controller& c = dynamic_cast<iAnt_controller&>
                         (entity.GetControllableEntity().GetController());

    UpdateDrawInWorldData(c);

    if(c.IsHoldingFood() == true) {
#ifdef __APPLE__
        DrawCylinder(CVector3(0.0f, 0.0f, 0.3f), CQuaternion(), 0.05f, 0.025f, CColor::BLACK);
#else
        DrawCylinder(0.05f, 0.025f, CVector3(0.0f, 0.0f, 0.3f), CColor::BLACK);
#endif
    }
}

void iAnt_qt_user_functions::DrawInWorld() {
    foodPositions      = loopFunctions->GetFoodPositions();
    pheromonePositions = loopFunctions->GetPheromonePositions();
    fidelityPositions  = loopFunctions->GetFidelityPositions();

    CVector3 np(nestPosition.GetX(), nestPosition.GetY(), 0.001f);
    Real height = foodRadius / 2.0;

#ifdef __APPLE__
    CQuaternion q;
#endif

    // draw the nest
#ifdef __APPLE__
    DrawCircle(np, q, nestRadius, CColor::GRAY80);
#else
    DrawCircle(nestRadius, np, CColor::GRAY80);
#endif
    
    if(foodPositions.size() > 0) {
        Real x, y;
        CVector3 p3d;

        // draw the food items
        for(size_t i = 0; i < foodPositions.size(); i++) {
            x = foodPositions[i].GetX();
            y = foodPositions[i].GetY();
            p3d = CVector3(x, y, 0.0);

#ifdef __APPLE__
            DrawCylinder(p3d, q, foodRadius, height, CColor::BLACK);
#else
            DrawCylinder(foodRadius, height, p3d, CColor::BLACK);
#endif
        }

        // draw the pheromone positions
        for(size_t i = 0; i < pheromonePositions.size(); i++) {
            if(pheromonePositions[i] != nestPosition) {
                x = pheromonePositions[i].GetX();
                y = pheromonePositions[i].GetY();
                p3d = CVector3(x, y, 0.0);

#ifdef __APPLE__
                DrawCylinder(p3d, q, foodRadius, height, CColor::RED);
#else
//                DrawCylinder(foodRadius, height, p3d, CColor::RED);
#endif
            }
        }

        // draw the fidelity positions
        for(size_t i = 0; i < fidelityPositions.size(); i++) {
            if(fidelityPositions[i] != nestPosition) {
                x = fidelityPositions[i].GetX();
                y = fidelityPositions[i].GetY();
                p3d = CVector3(x, y, 0.0);

#ifdef __APPLE__
                DrawCylinder(p3d, q, foodRadius, height, CColor::BLUE);
#else
                DrawCylinder(foodRadius, height, p3d, CColor::BLUE);
#endif
            }
        }
    }
}

// macro required for registering this class with Argos
REGISTER_QTOPENGL_USER_FUNCTIONS(iAnt_qt_user_functions,
                                 "iAnt_qt_user_functions")
