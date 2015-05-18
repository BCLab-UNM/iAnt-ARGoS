#ifndef IANT_PHEROMONE_H_
#define IANT_PHEROMONE_H_

#include <argos3/core/utility/math/vector2.h>

using namespace argos;
using namespace std;

/*****
 * Implementation of the iAnt Pheromone object used by the iAnt CPFA.
 * iAnts build and maintain a list of these pheromone waypoint objects
 * to use during the informed search component of the CPFA algorithm.
 *****/
class iAnt_pheromone {

	public:

        /* constructor function */
		iAnt_pheromone(CVector2         newLocation,
                       vector<CVector2> newTrail,
                       Real             newTime,
                       Real             newDecayRate);

        /* public helper functions */
        void             Update(Real time);
		CVector2         GetLocation();
        vector<CVector2> GetTrail();
		Real             GetWeight();
        bool             IsActive();

	private:

        /* pheromone position variables */
		CVector2         location;
        vector<CVector2> trail;

        /* pheromone component variables */
		Real lastUpdated;
		Real decayRate;
		Real weight;
		Real threshold;
};

#endif /* IANT_PHEROMONE_H_ */
