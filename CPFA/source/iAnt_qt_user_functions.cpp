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

/*****
 *
 *****/
void iAnt_qt_user_functions::DrawOnRobot(CFootBotEntity& entity) {
    iAnt_controller& c = dynamic_cast<iAnt_controller&>(entity.GetControllableEntity().GetController());

    if(data == NULL) data = c.GetData();

    if(c.IsHoldingFood() == true) {
        DrawCylinder(CVector3(0.0, 0.0, 0.3), CQuaternion(), data->FoodRadius, 0.025, CColor::BLACK);
    }
}
 
/*****
 *
 *****/
void iAnt_qt_user_functions::DrawOnArena(CFloorEntity& entity) {
    if(data != NULL) {
        DrawFood();
        DrawFidelity();
        DrawPheromones();
        DrawNest();

        if(data->DrawTargetRays == 1) DrawTargetRays();
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
    Real x_coordinate = data->NestPosition.GetX();
    Real y_coordinate = data->NestPosition.GetY();

    /* required: leaving this 0.0 will draw the nest inside of the floor */
    Real elevation = data->NestElevation;

    /* 3d cartesian coordinates of the nest */
    CVector3 nest_3d(x_coordinate, y_coordinate, elevation);

    /* Draw the nest on the arena. */
    DrawCircle(nest_3d, CQuaternion(), data->NestRadius, CColor::GRAY50);
}

/*****
 *
 *****/
void iAnt_qt_user_functions::DrawFood() {
    /* if the iAnt_data object is null, we cannot draw the nest */
    if(data == NULL) return;

    Real x, y;

    for(size_t i = 0; i < data->FoodList.size(); i++) {
        x = data->FoodList[i].GetX();
        y = data->FoodList[i].GetY();
        DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), data->FoodRadius, 0.025, data->FoodColoringList[i]);
    }
}

/*****
 *
 *****/
void iAnt_qt_user_functions::DrawFidelity() {
    /* if the iAnt_data object is null, we cannot draw the nest */
    if(data == NULL) return;

    Real x, y;

    /*for(size_t i = 0; i < data->FidelityList.size(); i++) {
        x = data->FidelityList[i].GetX();
        y = data->FidelityList[i].GetY();
	*/ //qilu 07/17
	for(map<string, CVector2>::iterator it=data->FidelityList.begin(); it!=data->FidelityList.end(); ++it){ //qilu 07/17
		x = it->second.GetX(); 
		y = it->second.GetY(); 
        DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), data->FoodRadius, 0.025, CColor::CYAN);
    }
}

/*****
 *
 *****/
void iAnt_qt_user_functions::DrawPheromones() {
    /* if the iAnt_data object is null, we cannot draw the nest */
    if(data == NULL) return;

    Real x, y, weight;
    vector<CVector2> trail;
    CColor trailColor = CColor::GREEN, pColor = CColor::GREEN;

    for(size_t i = 0; i < data->PheromoneList.size(); i++) {
        x = data->PheromoneList[i].GetLocation().GetX();
        y = data->PheromoneList[i].GetLocation().GetY();

        if(data->DrawTrails == 1) {
            trail  = data->PheromoneList[i].GetTrail();
            weight = data->PheromoneList[i].GetWeight();

            if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
                pColor = trailColor = CColor::GREEN;
            else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
                pColor = trailColor = CColor::YELLOW;
            else                                      // [   5.0% ,  0.0% ]
                pColor = trailColor = CColor::RED;

            /*
            for(size_t j = 0; j < trail.size(); j++) {
                x = trail[j].GetX();
                y = trail[j].GetY();

                DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), data->FoodRadius, 0.025, trailColor);
            }
            */

            CRay3 ray;

            for(size_t j = 1; j < trail.size(); j++) {
                ray = CRay3(CVector3(trail[j - 1].GetX(), trail[j - 1].GetY(), 0.01),
                            CVector3(trail[j].GetX(), trail[j].GetY(), 0.01));
                DrawRay(ray, trailColor, 1.0);
            }

            DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), data->FoodRadius, 0.025, pColor);
        } else {
            weight = data->PheromoneList[i].GetWeight();

            if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
                pColor = CColor::GREEN;
            else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
                pColor = CColor::YELLOW;
            else                                      // [   5.0% ,  0.0% ]
                pColor = CColor::RED;

            DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), data->FoodRadius, 0.025, pColor);
        }
    }
}

void iAnt_qt_user_functions::DrawTargetRays() {
    for(size_t i = 0; i < data->TargetRayList.size(); i++) {
        DrawRay(data->TargetRayList[i], CColor::BLUE);
    }

    data->TargetRayList.clear();
}

REGISTER_QTOPENGL_USER_FUNCTIONS(iAnt_qt_user_functions, "iAnt_qt_user_functions")
