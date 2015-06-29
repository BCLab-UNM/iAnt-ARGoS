#include "iAnt_controller.h"
#include <stdlib.h>
/*****
 * Initialize most basic variables and objects here. Most of the setup should
 * be done in the Init(...) function instead of here where possible.
 *****/
iAnt_controller::iAnt_controller() :
    compass(NULL),
    motorActuator(NULL),
    proximitySensor(NULL),
    data(NULL),
    RobotForwardSpeed(0.0),
    RobotTurningSpeed(0.0)
{}

/*****
 * Initialize the controller via the XML configuration file. ARGoS typically
 * wants objects & variables initialized here instead of in the constructor(s).
 *****/
void iAnt_controller::Init(TConfigurationNode& node) {
    /* Shorter names, please. #This_Is_Not_Java */
    typedef CCI_PositioningSensor            CCI_PS;
    typedef CCI_DifferentialSteeringActuator CCI_DSA;
    typedef CCI_FootBotProximitySensor       CCI_FBPS;

    /* Initialize the robot's actuator and sensor objects. */
    motorActuator   = GetActuator<CCI_DSA>("differential_steering");
    compass         = GetSensor<CCI_PS>   ("positioning");
    proximitySensor = GetSensor<CCI_FBPS> ("footbot_proximity");

    TConfigurationNode iAnt_params = GetNode(node, "iAnt_params");
    GetNodeAttribute(iAnt_params, "RobotForwardSpeed", RobotForwardSpeed);
    GetNodeAttribute(iAnt_params, "RobotTurningSpeed", RobotTurningSpeed);
    GetNodeAttribute(iAnt_params, "AngleToleranceInDegrees", angleInDegrees);

    AngleToleranceInRadians.Set(-ToRadians(angleInDegrees),ToRadians(angleInDegrees));
    
    stepSize = 0.2; /* Assigns the robot's stepSize */
    
    /***** 
     * Initializes the pattern here by reading pattern from
     * a text file. 
    *****/
    ReadFile();
    reverse(pattern.begin(),pattern.end());/* Reverses the pattern */
}

/*****
 * Reads the pattern from a text file.
 *****/
 void iAnt_controller::ReadFile(){
    ifstream inFile;

    /* Specify the file to open. */
    //"DiamondPatternOutPut.txt"
    //"SquarePatternOutPut.txt"
    inFile.open("SquarePatternOutPut.txt");
    string sPattern;

    if(inFile.fail()){
        cerr << "Error in opening file.";
        exit(1);
    }
    
    if (inFile.is_open()){
        while (getline (inFile,sPattern)){
            /*copies the string into a vector<char> */
            copy(sPattern.begin(),
                 sPattern.end(),back_inserter(pattern));
        }
        cout<< "Pattern: "<< sPattern << endl;
       inFile.close();
    }
}

/*****
 * Primary control loop for this controller object. This function will execute
 * the CPFA logic using the CPFA enumeration flag once per frame.
 *****/
void iAnt_controller::ControlStep() {

    CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.02);
    CVector3 target3d(GetTarget().GetX(), GetTarget().GetY(), 0.02);
    CRay3 targetRay(target3d, position3d);
    data->TargetRayList.push_back(targetRay);

    /* Checks if the robot found a food */
    SetHoldingFood();

    /* If it didn't continue in a sprial */
    if(IsHoldingFood() == false){
       GetTargets(); /* Initializes targets positions. */

    } else { /* Check if it is near the nest then set isHoldingFood to false */
        if((GetPosition() - data->NestPosition).SquareLength() < data->NestRadiusSquared) {
            isHoldingFood = false;
        } else ApproachTheTarget(data->NestPosition);
    }
}   
/*****
 * Sets target North of the robot's current target.
 *****/
void iAnt_controller::SetTargetN(char x){
    CVector2 position = GetTarget();
    target = CVector2(position.GetX()+stepSize,position.GetY());
}

/*****
 * Sets target South of the robot's current target.
 *****/
void iAnt_controller::SetTargetS(char x){
    CVector2 position = GetTarget();
    target = CVector2(position.GetX()-stepSize,position.GetY());
}

/*****
 * Sets target East of the robot's current target.
 *****/
void iAnt_controller::SetTargetE(char x){
   CVector2 position = GetTarget();
   target = CVector2(position.GetX(),position.GetY()-stepSize);
}

/*****
 * Sets target West of the robot's current target.
 *****/
void iAnt_controller::SetTargetW(char x){
    CVector2 position = GetTarget();
    target = CVector2(position.GetX(),position.GetY()+stepSize);
}

/*****
 * Controls the robot's motor to go in the correct angle relative to the target.
 *****/
void iAnt_controller::ApproachTheTarget(){

    /* angle of the robot's direction relative to the arena's origin */
    CRadians angle1  = GetHeading();

    /* angle from the target to the robot's position */
    CRadians angle2  = (target - GetPosition()).Angle();

    /* heading = angle1 - angle2 = 0.0 when the robot is facing its target */
    CRadians heading = (angle1 - angle2).SignedNormalize();

    if(heading <= AngleToleranceInRadians.GetMin()) {
        /* turn left */
        motorActuator->SetLinearVelocity(-RobotTurningSpeed, 
                                          RobotTurningSpeed);
    } else if(heading >= AngleToleranceInRadians.GetMax()){
        /* turn right */
        motorActuator->SetLinearVelocity( RobotTurningSpeed, 
                                         -RobotTurningSpeed);
    } else {
        /* go straight */
        motorActuator->SetLinearVelocity(RobotForwardSpeed, 
                                         RobotForwardSpeed);
    }
}

void iAnt_controller::ApproachTheTarget(CVector2 myTarget){

    /* angle of the robot's direction relative to the arena's origin */
    CRadians angle1  = GetHeading();

    /* angle from the target to the robot's position */
    CRadians angle2  = (myTarget - GetPosition()).Angle();

    /* heading = angle1 - angle2 = 0.0 when the robot is facing its target */
    CRadians heading = (angle1 - angle2).SignedNormalize();

    if(heading <= AngleToleranceInRadians.GetMin()) {
        /* turn left */
        motorActuator->SetLinearVelocity(-RobotTurningSpeed, 
                                          RobotTurningSpeed);
    } else if(heading >= AngleToleranceInRadians.GetMax()){
        /* turn right */
        motorActuator->SetLinearVelocity( RobotTurningSpeed, 
                                         -RobotTurningSpeed);
    } else {
        /* go straight */
        motorActuator->SetLinearVelocity(RobotForwardSpeed, 
                                         RobotForwardSpeed);
    }
}
/*****
 *
 *****/
CVector2 iAnt_controller::GetTarget() {
    return target;
}

/*****
 * Helper function that reads vector <char> pattern
 * and sets the target's direction base on the 
 * char at the current vector index.
 *****/
 void iAnt_controller::GetTargets(){

    /* Finds the last direction of the pattern. */
    char direction_last = pattern[pattern.size() - 1]; 
   
    /* If the robot hit target and the patter size >0
       then find the next direction. */
    if(TargetHit() == true && pattern.size() > 0) {
        pattern.pop_back();

        switch(direction_last)
        {
            case 'N':
                SetTargetN('N');
                break;
            case 'S':
                SetTargetS('S');
                break;
            case 'E':
                SetTargetE('E');
                break;
            case 'W':
                SetTargetW('W');
                break;
            default:
                /* ????? */
                motorActuator->SetLinearVelocity(0.0, 0.0);       
        }
    }
    /* If the robot is down traversing the pattern, then return home */
    else if(pattern.size() == 0){
        ApproachTheTarget(data->NestPosition);
    /* Otherwise we continue to approach the target. */  
    } else ApproachTheTarget();
 }

/*****
 * Returns a boolean based on weather the robot is with 0.01 
 * distance tolerance. Declares that the robot had reached 
 * current target.
 *****/
 bool iAnt_controller::TargetHit(){
    CVector2 position = GetPosition();
    bool hit = false;
     
    if((position-target).SquareLength() < 0.01){
        hit = true;
    }
    return hit;
 }

/*****
 * Check if the iAnt is finding food. This is defined as the iAnt being within
 * the distance tolerance of the position of a food item. If the iAnt has found
 * food then the appropriate boolean flags are triggered.
 *****/
void iAnt_controller::SetHoldingFood(){
    /* Is the iAnt already holding food? */
    if(IsHoldingFood() == false) {
        vector <CVector2> newFoodList; 
        size_t i = 0; 

        /* No, the iAnt isn't holding food. Check if we have found food at our
           current position and update the food list if we have. */

        for (i = 0; i < data->FoodList.size(); i++){
            /* We found food! */
            if ((GetPosition()-data->FoodList[i]).SquareLength() < data->FoodRadiusSquared){
                isHoldingFood = true;
            }
            /* Else push the that current food onto the newFoodList. */
            else {
                /* Return this unfound-food position to the list */
                newFoodList.push_back(data->FoodList[i]);
            }
        } 
        data->FoodList = newFoodList;
    }
}
/*****
 * Is this iAnt_controller holding food?
 *     true  = yes
 *     false = no
 *****/
bool iAnt_controller::IsHoldingFood() {
    return isHoldingFood;
}
/*****
 * After pressing the reset button in the GUI, this controller will be set to
 * default factory settings like at the start of a simulation.
 *****/
void iAnt_controller::Reset() {
    CVector2 target = data->NestPosition;
}

/*****
 * iAnt_loop_functions uses this function to set the iAnt_data pointer.
 *****/
void iAnt_controller::SetData(iAnt_data* dataPointer) {
    data = dataPointer;
}

/*****
 * iAnt_qt_user_functions uses this function to get the iAnt_data pointer.
 *****/
iAnt_data* iAnt_controller::GetData() {
    return data;
}

/*****
 * Return the robot's 2D position on the arena.
 *****/
CVector2 iAnt_controller::GetPosition() {
    /* The robot's compass sensor gives us a 3D position. */
    CVector3 position3D = compass->GetReading().Position;
    /* Return the 2D position components of the compass sensor reading. */
    return CVector2(position3D.GetX(), position3D.GetY());
}

/*****
 * Return the angle the robot is facing relative to the arena's origin.
 *****/
CRadians iAnt_controller::GetHeading() {
    /* in ARGoS, the robot's orientation is represented by a quaternion */
    const CCI_PositioningSensor::SReading& sReading = compass->GetReading();
    CQuaternion orientation = sReading.Orientation;

    /* convert the quaternion to euler angles */
    CRadians z_angle, y_angle, x_angle;
    orientation.ToEulerAngles(z_angle, y_angle, x_angle);

    /* the angle to the z-axis represents the compass heading */
    return z_angle;
}

REGISTER_CONTROLLER(iAnt_controller, "iAnt_controller")
