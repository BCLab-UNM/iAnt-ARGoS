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
        //DrawCylinder(0.05f, 0.025f, CVector3(0.0f, 0.0f, 0.3f), CColor::BLACK);
        DrawCylinder(CVector3(0.0f, 0.0f, 0.3f), CQuaternion(), 0.05f, 0.025f, CColor::BLACK);
    }
}

void iAnt_qt_user_functions::DrawInWorld() {
    foodPositions      = loopFunctions->GetFoodPositions();
    pheromonePositions = loopFunctions->GetPheromonePositions();
    fidelityPositions  = loopFunctions->GetFidelityPositions();

    CVector3 np(nestPosition.GetX(), nestPosition.GetY(), 0.001f);
    Real height = foodRadius / 2.0;
    CQuaternion q;

    // draw the nest
    /* works on linux, not on Mac (beta 28 of argos) */
    //DrawCircle(nestRadius, np, CColor::GRAY80);
    /* new declaration for Mac (beta 30 of argos) */
    DrawCircle(np, q, nestRadius, CColor::GRAY80);
    
    if(foodPositions.size() > 0) {
        Real x, y;
        CVector3 p3d;

        // draw the food items
        for(size_t i = 0; i < foodPositions.size(); i++) {
            x = foodPositions[i].GetX();
            y = foodPositions[i].GetY();
            p3d = CVector3(x, y, 0.0);

            //DrawCylinder(foodRadius, height, const CVector3(p3d), CColor::BLACK);
            DrawCylinder(p3d, q, foodRadius, height, CColor::BLACK);
        }

        // draw the pheromone positions
        for(size_t i = 0; i < pheromonePositions.size(); i++) {
            if(pheromonePositions[i] != nestPosition) {
                x = pheromonePositions[i].GetX();
                y = pheromonePositions[i].GetY();
                p3d = CVector3(x, y, 0.0);
                
                //DrawCylinder(foodRadius, height, const CVector3(p3d), CColor::RED);
                DrawCylinder(p3d, q, foodRadius, height, CColor::RED);
            }
        }

        // draw the fidelity positions
        for(size_t i = 0; i < fidelityPositions.size(); i++) {
            if(fidelityPositions[i] != nestPosition) {
                x = fidelityPositions[i].GetX();
                y = fidelityPositions[i].GetY();
                p3d = CVector3(x, y, 0.0);

                //DrawCylinder(foodRadius, height, const CVector3(p3d), CColor::BLUE);
                DrawCylinder(p3d, q, foodRadius, height, CColor::BLUE);
            }
        }
    }
}

// macro required for registering this class with Argos
REGISTER_QTOPENGL_USER_FUNCTIONS(iAnt_qt_user_functions,
                                 "iAnt_qt_user_functions")