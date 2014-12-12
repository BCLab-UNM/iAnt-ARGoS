#ifndef IANT_QT_USER_FUNCTIONS_H_
#define IANT_QT_USER_FUNCTIONS_H_

#include <controllers/iAnt_controller/iAnt_controller.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class iAnt_qt_user_functions : public CQTOpenGLUserFunctions {

public:
    // constructor
    iAnt_qt_user_functions();

    // destructor
    virtual ~iAnt_qt_user_functions() {}

    // draw function for adding graphics to a foot-bot
    void Draw(CFootBotEntity& entity);

};

#endif // IANT_QT_USER_FUNCTIONS_H_
