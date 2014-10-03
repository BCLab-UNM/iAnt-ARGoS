#include "iAnt_pheromone.h"

iAnt_pheromone::iAnt_pheromone() :
	lastUpdated(0),
	decayRate(0.0),
	weight(0.0),
	threshold(0.0),
	isActive(false)
{}

iAnt_pheromone::iAnt_pheromone(const iAnt_pheromone &copy) {
	location    = copy.location;
	lastUpdated = copy.lastUpdated;
	decayRate   = copy.decayRate;
	weight      = copy.weight;
	threshold   = copy.threshold;
	isActive    = copy.isActive;
}

iAnt_pheromone::iAnt_pheromone(CVector2 loc, long int tick, double decay, double w) {
	location    = loc;
	decayRate   = decay;
	weight      = w;
	threshold   = w * 0.001;
	lastUpdated = tick;
	isActive    = true;
}

iAnt_pheromone::~iAnt_pheromone() {
	// unused
}

// Returns decay of quantity at time given rate of change lambda
void iAnt_pheromone::Update(long int time) {
    weight      *= exp(-decayRate * (time - lastUpdated));
    lastUpdated  = time;

    if(weight <= threshold) isActive = false;
}

// reset the pheromone
void iAnt_pheromone::Reset(CVector2 loc, long int tick, double w) {
	location    = loc;
	weight      = w;
	threshold   = w * 0.001;
	lastUpdated = tick;
	isActive    = true;
}

// change the decay rate
void iAnt_pheromone::SetDecay(double decay) {
	decayRate = decay;
}

// is the pheromone valid?
bool iAnt_pheromone::IsActive() {
	return isActive;
}

// pheromone strength or weight
double iAnt_pheromone::Strength() {
	return weight;
}

// pheromone location
CVector2 iAnt_pheromone::Location() {
	return location;
}

// set this pheromone to another pheromone's values
void iAnt_pheromone::Set(iAnt_pheromone newPheromone) {
	location    = newPheromone.location;
	lastUpdated = newPheromone.lastUpdated;
	decayRate   = newPheromone.decayRate;
	weight      = newPheromone.weight;
	threshold   = newPheromone.threshold;
	isActive    = newPheromone.isActive;
}
