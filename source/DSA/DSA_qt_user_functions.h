#ifndef DSA_QT_USER_FUNCTIONS_H
#define DSA_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/DSA/DSA_loop_functions.h>

using namespace argos;
using namespace std;

class DSA_controller;
class DSA_loop_functions;

class DSA_qt_user_functions : public CQTOpenGLUserFunctions {

    public:

        /* constructor and destructor functions */
        DSA_qt_user_functions();

        /* interface functions between QT and ARGoS */
        void DrawOnRobot(CFootBotEntity& entity);
        void DrawOnArena(CFloorEntity& entity);

    private:

        /* private helper drawing functions */
        void DrawNest();
        void DrawFood();
        void DrawTargetRays();

        DSA_loop_functions& loopFunctions;

};

#endif /* DSA_QT_USER_FUNCTIONS_H */