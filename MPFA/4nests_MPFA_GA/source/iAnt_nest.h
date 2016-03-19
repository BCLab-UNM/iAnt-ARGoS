#ifndef IANT_NEST_H_
#define IANT_NEST_H_

#include <argos3/core/utility/math/vector2.h>
//#include <argos3/core/utility/math/ray3.h>
#include "iAnt_pheromone.h"
using namespace argos;
using namespace std;

/*****
 * Implementation of the iAnt nest object used by the iAnt MPFA. iAnts
 * build and maintain a list of these nest objects.
 *****/
class iAnt_nest {

	public:
		iAnt_nest();
		iAnt_nest(CVector2 location);
		       
		vector<iAnt_pheromone> PheromoneList;
        /* constructor function */
		
		/* public helper functions */
        CVector2		GetLocation();
        void		SetLocation();
        
	private:

        CVector2        nestLocation;
        
};

#endif /* IANT_NEST_H_ */
