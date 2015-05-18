#include "iAnt_qt_user_functions.h"

/*****
 * Constructor: In order for drawing functions in this class to be used by
 * ARGoS it must be registered using the RegisterUserFunction function.
 *****/
iAnt_qt_user_functions::iAnt_qt_user_functions() :
    data(NULL)
{
    RegisterUserFunction<iAnt_qt_user_functions, CFootBotEntity>(&iAnt_qt_user_functions::DrawOnRobot);
    RegisterUserFunction<iAnt_qt_user_functions, CFloorEntity>(&iAnt_qt_user_functions::DrawOnArena);
}

void iAnt_qt_user_functions::DrawOnRobot(CFootBotEntity& entity) {
    iAnt_controller& c = dynamic_cast<iAnt_controller&>(entity.GetControllableEntity().GetController());

    if(data == NULL) data = c.GetData();

    if(c.IsHoldingFood() == true) {
        DrawCylinder(CVector3(0.0, 0.0, 0.3), CQuaternion(), 0.05, 0.025, CColor::BLACK);
    }
}
 
void iAnt_qt_user_functions::DrawOnArena(CFloorEntity& entity) {
    if(data != NULL) {
        DrawNest();
        DrawFood();
        DrawFidelity();
        DrawPheromones();
    }
}

/*****
 * This function is called by the DrawOnArena(...) function. If the iAnt_data
 * object is not initialized this function should not be called.
 *****/
void iAnt_qt_user_functions::DrawNest() {
    /* if the iAnt_data object is null, we cannot draw the nest */
    if(data == NULL) return;

    /* 2d cartesian coordinates of the nest */
    Real x_coordinate = data->nestPosition.GetX();
    Real y_coordinate = data->nestPosition.GetX();

    /* required: leaving this 0.0 will draw the nest inside of the floor */
    Real elevation = data->nestElevation;

    /* 3d cartesian coordinates of the nest */
    CVector3 nest_3d(x_coordinate, y_coordinate, elevation);

    /* Draw the nest on the arena. */
    DrawCircle(nest_3d, CQuaternion(), data->nestRadius, CColor::GRAY50);
}

void iAnt_qt_user_functions::DrawFood() {
    /* if the iAnt_data object is null, we cannot draw the nest */
    if(data == NULL) return;

    Real x, y;

    for(size_t i = 0; i < data->foodList.size(); i++) {
        x = data->foodList[i].GetX();
        y = data->foodList[i].GetY();
        DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), 0.05, 0.025, CColor::BLACK);
    }
}

void iAnt_qt_user_functions::DrawFidelity() {
    /* if the iAnt_data object is null, we cannot draw the nest */
    if(data == NULL) return;

    Real x, y;

    for(size_t i = 0; i < data->fidelityList.size(); i++) {
        x = data->fidelityList[i].GetX();
        y = data->fidelityList[i].GetY();
        DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), 0.05, 0.025, CColor::CYAN);
    }
}

void iAnt_qt_user_functions::DrawPheromones() {
    /* if the iAnt_data object is null, we cannot draw the nest */
    if(data == NULL) return;

    Real x, y, weight;
    vector<CVector2> trail;
    CColor trailColor;

    for(size_t i = 0; i < data->pheromoneList.size(); i++) {
        x = data->pheromoneList[i].GetLocation().GetX();
        y = data->pheromoneList[i].GetLocation().GetY();
        DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), 0.05, 0.025, CColor::PURPLE);

        if(data->drawTrails == true) {
            trail  = data->pheromoneList[i].GetTrail();
            weight = data->pheromoneList[i].GetWeight();

            if(weight > 0.475 && weight <= 1.0)       // [ 100.0% , 47.5% )
                trailColor = CColor::GREEN;
            else if(weight > 0.05 && weight <= 0.475) // [  47.5% ,  5.0% )
                trailColor = CColor::YELLOW;
            else                                      // [   5.0% ,  0.0% ]
                trailColor = CColor::RED;

            for(size_t j = 0; j < trail.size(); j++) {
                x = trail[j].GetX();
                y = trail[j].GetY();

                DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), 0.05, 0.025, trailColor);
            }
        }
    }
}

REGISTER_QTOPENGL_USER_FUNCTIONS(iAnt_qt_user_functions, "iAnt_qt_user_functions")
