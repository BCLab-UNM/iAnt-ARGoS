/*
 * iAnt_food_type.h
 *
 *  Created on: Apr 2, 2015
 *      Author: safeeulbashir
 */

#ifndef IANT_FOOD_TYPE_H_
#define IANT_FOOD_TYPE_H_
class iAnt_food_type{
public:
	argos::Real getCollectionTime() const {
		return collectionTime;
	}

	void setCollectionTime(argos::Real collectionTime) {
		this->collectionTime = collectionTime;
	}

	int getDistributionType() const {
		return DistributionType;
	}

	int getAntId() {
		return this->antId;
	}

	void setDistributionType(int distributionType) {
		DistributionType = distributionType;
	}

	int getPileId() const {
		return PileId;
	}

	int getPickUpTime(){
		return this->PickUpTime;
	}

	void setPileId(int pileId) {
		PileId = pileId;
	}

	const argos::CVector2& getPosition() const {
		return position;
	}

	void setPosition(const argos::CVector2& position) {
		this->position = position;
	}

	void setAntId(int antId) {
		this->antId = antId;
	}
	void setPickUpTime(int PickUpTime){
		this->PickUpTime=PickUpTime;
	}

private:
	int PileId;
	argos::CVector2 position;
	int DistributionType;
	argos::Real collectionTime;
	int antId;
	int PickUpTime;
};



#endif /* IANT_FOOD_TYPE_H_ */
