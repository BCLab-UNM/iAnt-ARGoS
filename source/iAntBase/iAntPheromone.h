#ifndef IANT_PHEROMONE_H
#define IANT_PHEROMONE_H

#include <argos3/core/utility/math/vector2.h>

/*****
 * Implementation of the iAnt Pheromone object used by the iAnt CPFA. iAnts build and maintain a list of these pheromone waypoint objects to use during
 * the informed search component of the CPFA algorithm.
 *****/
class iAntPheromone {

    public:

        /* constructor function */
		iAntPheromone(argos::CVector2 newLocation, std::vector<argos::CVector2> newTrail, argos::Real newTime, argos::Real newDecayRate);

        /* public helper functions */
        void                         Update(argos::Real time);
        void                         Deactivate();
		argos::CVector2              GetLocation();
        std::vector<argos::CVector2> GetTrail();
		argos::Real                  GetWeight();
        bool                         IsActive();

	private:

        /* pheromone position variables */
		argos::CVector2              location;
        std::vector<argos::CVector2> trail;

        /* pheromone component variables */
		argos::Real lastUpdated;
		argos::Real decayRate;
		argos::Real weight;
		argos::Real threshold;
};

#endif /* IANT_PHEROMONE_H */