#include "PheromoneList.h"

/* Add the new pheromone waypoint (pw) into the pheromone waypoint list (pwList). */
void PheromoneList::addPheromoneWaypoint(PheromoneWaypoint pw) {
	pwList.push_back(pw);
}

/* Cycle through the  pheromone waypoint list and return
 * the first (and oldest active) waypoint from the list. */
PheromoneWaypoint PheromoneList::getPheromoneWaypoint() {
	/* Cycle through the "vector" of "PheromoneWaypoint"s. */
	for(size_t i = 0; i < pwList.size(); i++) {
		/* Return the first active (and oldest) waypoint. */
		if(pwList[i].isActive()) {
			return pwList[i];
		}
	}

	/* Return a NULL pheromone waypoint when none are active. */
	return PheromoneWaypoint();
}

/* Cycle through all of the pheromone waypoints and update their exponential decay. Ignore
 * any deactivated pheromones and add the rest to an updated vector of active pheromones. */
void PheromoneList::update(size_t time) {
	/* Don't even bother if the list vector has no elements. */
	if(pwList.size() > 0) {
		/* Temporary storage for the new vector for post-update. */
		vector<PheromoneWaypoint> newList;

		/* Go through every currently active pheromone waypoint. */
		for(size_t i = 0; i < pwList.size(); i++) {
			/* Update the waypoint, and if it is still viable, add it to the temporary storage. */
			if(pwList[i].update(time)) {
				newList.push_back(pwList[i]);
			}
		}

		/* Reset the pheromone waypoint list in order to remove deactivated pheromones. */
		this->pwList = newList;
	}
}
