#include "iAnt_qt_user_functions.h"
#include <source/iAnt_loop_functions.h>

/*****
 * Constructor: In order for drawing functions in this class to be used by
 * ARGoS it must be registered using the RegisterUserFunction function.
 *****/
iAnt_qt_user_functions::iAnt_qt_user_functions() :
    loopFunctions(dynamic_cast<iAnt_loop_functions&>(CSimulator::GetInstance().GetLoopFunctions()))
{
    RegisterUserFunction<iAnt_qt_user_functions, CFootBotEntity>(&iAnt_qt_user_functions::DrawOnRobot);
    RegisterUserFunction<iAnt_qt_user_functions, CFloorEntity>(&iAnt_qt_user_functions::DrawOnArena);
}

/*****
 *
 *****/
void iAnt_qt_user_functions::DrawOnRobot(CFootBotEntity& entity) {
    iAnt_controller& c = dynamic_cast<iAnt_controller&>(entity.GetControllableEntity().GetController());

    if(c.IsHoldingFood() == true) {
        DrawCylinder(CVector3(0.0, 0.0, 0.3), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::BLACK);
    }
}
 
/*****
 *
 *****/
void iAnt_qt_user_functions::DrawOnArena(CFloorEntity& entity) {
    DrawFood();
    DrawFidelity();
    DrawPheromones();
    DrawNest();

    if(loopFunctions.DrawTargetRays == 1) DrawTargetRays();
}

/*****
 * This function is called by the DrawOnArena(...) function. If the iAnt_data
 * object is not initialized this function should not be called.
 *****/
void iAnt_qt_user_functions::DrawNest() {

    /* 2d cartesian coordinates of the nest */
    Real x_coordinate = loopFunctions.NestPosition.GetX();
    Real y_coordinate = loopFunctions.NestPosition.GetX();

    /* required: leaving this 0.0 will draw the nest inside of the floor */
    Real elevation = loopFunctions.NestElevation;

    /* 3d cartesian coordinates of the nest */
    CVector3 nest_3d(x_coordinate, y_coordinate, elevation);

    /* Draw the nest on the arena. */
    DrawCircle(nest_3d, CQuaternion(), loopFunctions.NestRadius, CColor::GRAY50);
}

/*****
 *
 *****/
void iAnt_qt_user_functions::DrawFood() {

    Real x, y;

    for(size_t i = 0; i < loopFunctions.FoodList.size(); i++) {
        x = loopFunctions.FoodList[i].GetX();
        y = loopFunctions.FoodList[i].GetY();
        DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, loopFunctions.FoodColoringList[i]);
    }
}

/*****
 *
 *****/
void iAnt_qt_user_functions::DrawFidelity() {

    Real x, y;

    for(size_t i = 0; i < loopFunctions.FidelityList.size(); i++) {
        x = loopFunctions.FidelityList[i].GetX();
        y = loopFunctions.FidelityList[i].GetY();
        DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::CYAN);
    }
}

/*****
 *
 *****/
void iAnt_qt_user_functions::DrawPheromones() {

    Real x, y, weight;
    vector<CVector2> trail;
    CColor trailColor = CColor::GREEN, pColor = CColor::GREEN;

    for(size_t i = 0; i < loopFunctions.PheromoneList.size(); i++) {
        x = loopFunctions.PheromoneList[i].GetLocation().GetX();
        y = loopFunctions.PheromoneList[i].GetLocation().GetY();

        if(loopFunctions.DrawTrails == 1) {
            trail  = loopFunctions.PheromoneList[i].GetTrail();
            weight = loopFunctions.PheromoneList[i].GetWeight();

            if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
                pColor = trailColor = CColor::GREEN;
            else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
                pColor = trailColor = CColor::YELLOW;
            else                                      // [   5.0% ,  0.0% ]
                pColor = trailColor = CColor::RED;

            CRay3 ray;
            size_t j = 0;

            for(j = 1; j < trail.size(); j++) {
                ray = CRay3(CVector3(trail[j - 1].GetX(), trail[j - 1].GetY(), 0.01),
                            CVector3(trail[j].GetX(), trail[j].GetY(), 0.01));
                DrawRay(ray, trailColor, 1.0);
            }

            DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, pColor);
        } else {
            weight = loopFunctions.PheromoneList[i].GetWeight();

            if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
                pColor = CColor::GREEN;
            else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
                pColor = CColor::YELLOW;
            else                                      // [   5.0% ,  0.0% ]
                pColor = CColor::RED;

            DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, pColor);
        }
    }
}

void iAnt_qt_user_functions::DrawTargetRays() {
    for(size_t i = 0; i < loopFunctions.TargetRayList.size(); i++) {
        DrawRay(loopFunctions.TargetRayList[i], CColor::BLUE);
    }

    loopFunctions.TargetRayList.clear();
}

REGISTER_QTOPENGL_USER_FUNCTIONS(iAnt_qt_user_functions, "iAnt_qt_user_functions")