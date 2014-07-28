#include "_iAnt_qt_user_functions.h"

// constructor
iAnt_qt_user_functions::iAnt_qt_user_functions() {
    // register and connect this drawing function to Argos
    RegisterUserFunction<iAnt_qt_user_functions, CFootBotEntity>(&iAnt_qt_user_functions::Draw);
}

// draw function for adding graphics to a foot-bot
void iAnt_qt_user_functions::Draw(CFootBotEntity& entity) {
    // foot-bot controller object
    iAnt_controller& controller = dynamic_cast<iAnt_controller&>(entity.GetControllableEntity().GetController());

    // if the foot-bot has a food item, draw it on top of the foot-bot
    if(controller.isHoldingFood()) {
        // these hard-wired values are valid for the foot-bot only,
        // and must be changed if different robot graphics are used
        DrawCylinder(0.1f, 0.05f, CVector3(0.0f, 0.0f, 0.3f), CColor::BLACK);
    }
}

// macro required for registering this class with Argos
REGISTER_QTOPENGL_USER_FUNCTIONS(iAnt_qt_user_functions, "iAnt_qt_user_functions")
