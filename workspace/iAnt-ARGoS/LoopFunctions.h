#ifndef LOOPFUNCTIONS_H_
#define LOOPFUNCTIONS_H_

#include "PheromoneList.h"
#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class LoopFunctions {

private:

	PheromoneList pList;
	size_t frame;

public:

	LoopFunctions() :
		frame(0)
	{}

	~LoopFunctions() {}

	/**
     * Executes user-defined initialization logic.
     * The default implementation of this method does nothing.
     * @param t_tree The <tt>&lt;loop_functions&gt;</tt> XML configuration tree.
     * @see Reset()
     * @see Destroy()
     */
    void Init(TConfigurationNode& t_tree) {}

    /**
     * Executes user-defined reset logic.
     * This method should restore the state of the simulation at it was right
     * after Init() was called.
     * The default implementation of this method does nothing.
     * @see Init()
     */
    void Reset() {
    	frame = 0;
    }

    /**
     * Executes user-defined destruction logic.
     * This method should undo whatever is done in Init().
     * The default implementation of this method does nothing.
     * @see Init()
     */
    void Destroy() {}

    /**
     * Executes user-defined logic right before a simulation step is executed.
     * The default implementation of this method does nothing.
     * @see PostStep()
     */
    void PreStep() {
    	frame++;
    }

    /**
     * Executes user-defined logic right after a simulation step is executed.
     * The default implementation of this method does nothing.
     * @see PreStep()
     */
    void PostStep() {}

    /**
     * Returns <tt>true</tt> if the experiment is finished.
     * This method allows the user to specify experiment-specific ending conditions.
     * The default implementation of this method returns always <tt>false</tt>.
     * This means that the only ending conditions for an experiment are time limit
     * expiration or GUI shutdown.
     * @return <tt>true</tt> if the experiment is finished.
     * @see CSimulator::IsExperimentFinished()
     */
    bool IsExperimentFinished() {
       return false;
    }

    /**
     * Executes user-defined logic when the experiment finishes.
     * This method is called within CSimulator::IsExperimentFinished()
     * as soon as its return value evaluates to <tt>true</tt>. This
     * method is executed before Destroy().
     * You can use this method to perform final calculations at the
     * end of an experiment.
     * The default implementation of this method does nothing.
     */
    void PostExperiment() {
    }

    /**
     * Returns the color of the floor in the specified point.
     * This function is called if the floor entity was configured to take the loop functions
     * as source. The floor color is used by the ground sensors to calculate their readings,
     * and by the graphical visualization to create a texture to display on the arena floor.
     * @param c_pos_on_floor The position on the floor.
     * @return The color of the floor in the specified point.
     * @see CFloorEntity
     * @see CGroundSensorEquippedEntity
     * @see CGroundRotZOnlySensor
     */
    CColor GetFloorColor(const CVector2& c_pos_on_floor) {
       return CColor::BLACK;
    }

};

#endif /* LOOPFUNCTIONS_H_ */
