#include "iAnt_pheromone.h"

using namespace argos;

static inline float exponentialDecay(float quantity, float time, float lambda) {
	return (quantity * exp(-lambda * time));
}

iAnt_pheromone::iAnt_pheromone() :
	lastUpdated(0),
	decayRate(0.0),
	weight(1.0),
	threshold(0.001),
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
	threshold   = 0.001;
	lastUpdated = tick;
	isActive    = true;
}

iAnt_pheromone::~iAnt_pheromone() {
	// unused
}

// Returns decay of quantity at time given rate of change lambda
void iAnt_pheromone::Update(long int time) {
	// divide by 8 to compensate for 16 frames per second
    weight      = exponentialDecay(weight, (time - lastUpdated)/8.0, decayRate);
    lastUpdated = time;

    if(weight <= threshold) isActive = false;
}

// reset the pheromone
void iAnt_pheromone::Reset(CVector2 loc, long int tick, double w) {
	location    = loc;
	weight      = w;
	threshold   = 0.001;
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

// simTime pheromone was last updated at
long int iAnt_pheromone::LastUpdated() {
    return lastUpdated;
}

// pheromone decay rate
double iAnt_pheromone::DecayRate() {
    return decayRate;
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

double iAnt_pheromone::Weight() {
	return weight;
}

void iAnt_pheromone::PrintPheromone() {
    /*
    LOG << "*****\n";
	LOG << "location: "    << location    << endl;
	LOG << "decayRate: "   << decayRate   << endl;
	LOG << "weight: "      << weight      << endl;
	LOG << "threshold: "   << threshold   << endl;
	LOG << "lastUpdated: " << lastUpdated << endl;
	LOG << "isActive: "    << isActive    << endl;
    LOG << "*****\n";
     */
}
