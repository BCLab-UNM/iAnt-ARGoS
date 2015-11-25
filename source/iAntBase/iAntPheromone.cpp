#include "iAntPheromone.h"

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
iAntPheromone::iAntPheromone(argos::CVector2              newLocation,
                             std::vector<argos::CVector2> newTrail,
                             argos::Real                  newTime,
                             argos::Real                  newDecayRate)
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
void iAntPheromone::Update(argos::Real time) {
    /* pheromones experience exponential decay with time */
    weight *= exp(-decayRate * (time - lastUpdated));
    lastUpdated = time;
}

/*****
 * Turns off a pheromone and makes it inactive.
 *****/
void iAntPheromone::Deactivate() {
    weight = 0.0;
}

/*****
 * Return the pheromone's location.
 *****/
argos::CVector2 iAntPheromone::GetLocation() {
    return location;
}

/*****
 * Return the trail between the pheromone and the nest.
 *****/
std::vector<argos::CVector2> iAntPheromone::GetTrail() {
    return trail;
}

/*****
 * Return the weight, or strength, of this pheromone.
 *****/
argos::Real iAntPheromone::GetWeight() {
	return weight;
}

/*****
 * Is the pheromone active and usable?
 * TRUE:  weight >  threshold : the pheromone is active
 * FALSE: weight <= threshold : the pheromone is not active
 *****/
bool iAntPheromone::IsActive() {
	return (weight > threshold);
}
