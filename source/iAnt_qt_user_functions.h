#ifndef IANT_QT_USER_FUNCTIONS_H_
#define IANT_QT_USER_FUNCTIONS_H_

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/ray3.h>

using namespace argos;
using namespace std;

class iAnt_controller;
class iAnt_loop_functions;

/*****
 * The iAnt_qt_user_functions class is used to draw food, nest, and pheromone
 * objects onto the arena during experiments using the GUI.
 *****/
class iAnt_qt_user_functions : public CQTOpenGLUserFunctions {

    public:

        /* constructor and destructor functions */
        iAnt_qt_user_functions();

        /* interface functions between QT and ARGoS */
        void DrawOnRobot(CFootBotEntity& entity);
        void DrawOnArena(CFloorEntity& entity);

    private:

        /* private helper drawing functions */
        void DrawNest();
        void DrawFood();
        void DrawFidelity();
        void DrawPheromones();
        void DrawTargetRays();

        iAnt_loop_functions& loopFunctions;
};

#endif /* IANT_QT_USER_FUNCTIONS_H_ */
