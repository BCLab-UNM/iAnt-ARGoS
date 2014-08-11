/*
 * FoodData.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: antonio
 */

#include "FoodData.h"

FoodData::FoodData() :
	hasFood(false),
	totalFoodCollected(0)
{}

FoodData::~FoodData()
{}

bool FoodData::HasFood() {
	return hasFood;
}

bool FoodData::HasPheromone() {
	return pheromone.isActive();
}

CVector2 FoodData::GetPheromone() {
	/* TODO change "1" to actual/correct time update */
	pheromone.update(1);
	return pheromone.getLocation();
}
