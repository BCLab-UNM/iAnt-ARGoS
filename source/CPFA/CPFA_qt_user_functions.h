#ifndef CPFA_QT_USER_FUNCTIONS_H
#define CPFA_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/ray3.h>
#include <source/CPFA/CPFA_loop_functions.h>

using namespace std;
using namespace argos;

class CPFA_loop_functions;

class CPFA_qt_user_functions : public argos::CQTOpenGLUserFunctions {

	public:

		CPFA_qt_user_functions();

        /* interface functions between QT and ARGoS */
        void DrawOnRobot(argos::CFootBotEntity& entity);
        void DrawOnArena(argos::CFloorEntity& entity);

    private:

        /* private helper drawing functions */
        void DrawNest();
        void DrawFood();
        void DrawFidelity();
        void DrawPheromones();
        void DrawTargetRays();

        CPFA_loop_functions& loopFunctions;

};

#endif /* CPFA_QT_USER_FUNCTIONS_H */