#ifndef IANT_QT_USER_FUNCTIONS_H_
#define IANT_QT_USER_FUNCTIONS_H_

#include <controllers/iAnt_controller.h>
#include <loop_functions/iAnt_loop_functions.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class iAnt_qt_user_functions : public CQTOpenGLUserFunctions {

private:

    Real                 foodRadius;
    Real                 nestRadius;
    vector<CVector2>     fidelityPositions;
    vector<CVector2>     pheromonePositions;
    vector<CVector2>     foodPositions;
    CVector2             nestPosition;
    iAnt_loop_functions *loopFunctions;

    void                 UpdateDrawInWorldData(iAnt_controller& c);

public:

    iAnt_qt_user_functions();
    ~iAnt_qt_user_functions() {}

    void DrawFood(CFootBotEntity& entity);
    void DrawInWorld();
};

#endif // IANT_QT_USER_FUNCTIONS_H_
