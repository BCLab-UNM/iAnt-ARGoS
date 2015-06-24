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
}
 
/*****
 *
 *****/
void iAnt_qt_user_functions::DrawOnArena(CFloorEntity& entity) {
    if(data != NULL) {
        DrawNest();
        DrawFood();
        //////////////////ADDED///////////////////////
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
    Real y_coordinate = data->NestPosition.GetX();

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
        DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), data->FoodRadius, 0.025, CColor::BLACK);
    }
}
//////////////////ADDED////////////////////////////
void iAnt_qt_user_functions::DrawTargetRays() {
    for(size_t i = 0; i < data->TargetRayList.size(); i++) {
        DrawRay(data->TargetRayList[i], CColor::BLUE);
    }
    
    //data->TargetRayList.clear();
}


REGISTER_QTOPENGL_USER_FUNCTIONS(iAnt_qt_user_functions, "iAnt_qt_user_functions")
