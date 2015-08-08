#include "iAnt_nest.h"

/*****
 * The iAnt nest needs to keep track of four things:
 *
 * [1] location
 * [2] nest id 
 * [3] site fidelity
 * [4] pheromone trails
 *
 *****/
	iAnt_nest::iAnt_nest(){}
	iAnt_nest::iAnt_nest(CVector2   location)
{
    /* required initializations */
	nestLocation    = location;
    PheromoneList.clear();
}

/*****
 *****/

/*****
 * Return the nest's location.
 *****/
CVector2 iAnt_nest::GetLocation() {
    return nestLocation;
}

void iAnt_nest::SetLocation() {
    nestLocation=CVector2(0.0, 0.0);
}

        
        
