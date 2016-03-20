#include "iAnt_pheromone.h"

/*****
 * The iAnt pheromone needs to keep track of four things:
 *
 * [1] location of the waypoint
 * [2] a trail to the nest
 * [3] simulation time at creation
 * [4] pheromone rate of decay
 *
 * The remaining variables always start with default values.
 *****/
iAnt_pheromone::iAnt_pheromone(CVector2         newLocation,
                               vector<CVector2> newTrail,
                               Real             newTime,
                               Real             newDecayRate)
{
    /* required initializations */
	location    = newLocation;
    trail       = newTrail;
	lastUpdated = newTime;
	decayRate   = newDecayRate;

    /* standardized initializations */
	weight      = 1.0;
	threshold   = 0.001;
}

/*****
 * The pheromones slowly decay and eventually become inactive. This simulates
 * the effect of a chemical pheromone trail that dissipates over time.
 *****/
void iAnt_pheromone::Update(Real time) {
    /* pheromones experience exponential decay with time */
    weight *= exp(-decayRate * (time - lastUpdated)); //qilu 03/13/2016 the exponential decay is based on the initial. This is correct, since 'lastUpdated' is updated every time, and (time - lastUpdated) is a unit time.
    lastUpdated = time;
}

/*****
 * Return the pheromone's location.
 *****/
CVector2 iAnt_pheromone::GetLocation() {
    return location;
}

/*****
 * Return the trail between the pheromone and the nest.
 *****/
vector<CVector2> iAnt_pheromone::GetTrail() {
    return trail;
}

/*****
 * Return the weight, or strength, of this pheromone.
 *****/
Real iAnt_pheromone::GetWeight() {
	return weight;
}

/*****
 * Is the pheromone active and usable?
 * TRUE:  weight >  threshold : the pheromone is active
 * FALSE: weight <= threshold : the pheromone is not active
 *****/
bool iAnt_pheromone::IsActive() {
	return (weight > threshold);
}
