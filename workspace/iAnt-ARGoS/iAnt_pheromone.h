#ifndef IANT_PHEROMONE_H_
#define IANT_PHEROMONE_H_

#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class iAnt_pheromone {
	public:
		iAnt_pheromone();
		iAnt_pheromone(const iAnt_pheromone &copy);
		iAnt_pheromone(CVector2 loc, long int tick, double decay, double w = 1.0);
		~iAnt_pheromone();

		void     Update(long int time); // decay at time given with rate of change lambda
		void     Reset(CVector2 loc, long int tick, double w = 1.0); // reset the pheromone
		void     SetDecay(double decay); // change the decay rate
		bool     IsActive(); // is the pheromone valid and active?
		double   Strength(); // pheromone strength or weight
		CVector2 Location(); // pheromone location
		void     Set(iAnt_pheromone newPheromone); // set pheromone = newPheromone
	private:
		CVector2 location;
		long int lastUpdated;
		double   decayRate;
		double   weight;
		double   threshold;
		bool     isActive;
};

#endif /* IANT_PHEROMONE_H_ */
