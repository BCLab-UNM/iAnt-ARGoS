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
    
    ///////////////ADDED////////////////////s
    GetNodeAttribute(iAnt_params, "AngleToleranceInDegrees", angleInDegrees);

    AngleToleranceInRadians.Set(-ToRadians(angleInDegrees),ToRadians(angleInDegrees));
    
    pattern.push_back('N');
    pattern.push_back('E');
    pattern.push_back('S');
    pattern.push_back('W');
    
    pattern.push_back('N');
    pattern.push_back('E');
    pattern.push_back('S');
    pattern.push_back('W');

    pattern.push_back('N');    
    pattern.push_back('E');
    pattern.push_back('S');
    pattern.push_back('W');
}

/*****
 * Primary control loop for this controller object. This function will execute
 * the CPFA logic using the CPFA enumeration flag once per frame.
 *****/

void iAnt_controller::ControlStep() {
    // LOG << GetHeading() << endl << '\n';
    
    ///////ADDED///////////////
    CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.02);
    CVector3 target3d(GetTarget().GetX(), GetTarget().GetY(), 0.02);
    CRay3 targetRay(target3d, position3d);
    data->TargetRayList.push_back(targetRay);
    
    char direction = pattern[pattern.size() - 1];

    if(TargetHit() == true && pattern.size() > 0) {
        pattern.pop_back();    // size_t = unsigned int

        switch(direction)
        {
            case 'N':
                //LOG << "NORTH\n";
                SetTargetN('N');
                break;
            case 'S':
                //LOG << "SOUTH\n";
                SetTargetS('S');
                break;
            case 'E':
                //LOG << "EAST\n";
                SetTargetE('E');
                break;
            case 'W':
                //LOG << "WEST\n";
                SetTargetW('W');
                break;
            default:
                //LOG << "SHENANIGANS!\n\n";
                motorActuator->SetLinearVelocity(0.0, 0.0);

        }
        //if(index < pattern.size()) GetTargets(index);
    } else ApproachTheTarget();

}
/*****
 *
 *****/
void iAnt_controller::SetTargetN(char x){
    CVector2 position = GetTarget();
    target = CVector2(position.GetX()+2.5,position.GetY());
    //ApproachTheTarget();
    //indx++;
}

/*****
 *
 *****/
void iAnt_controller::SetTargetS(char x){
    CVector2 position = GetTarget();
    target = CVector2(position.GetX()-2.5,position.GetY());
    //ApproachTheTarget();
   // indx++;
}

/*****
 *
 *****/
void iAnt_controller::SetTargetE(char x){
   CVector2 position = GetTarget();
   target = CVector2(position.GetX(),position.GetY()-2.5);
  // ApproachTheTarget();
   //indx++;
}

/*****
 *
 *****/
void iAnt_controller::SetTargetW(char x){
    CVector2 position = GetTarget();
    target = CVector2(position.GetX(),position.GetY()+2.5);
    //ApproachTheTarget();
    //indx++;
}

/*****
 *
 *****/
void iAnt_controller::ApproachTheTarget(){

    /* angle of the robot's direction relative to the arena's origin */
    CRadians angle1  = GetHeading();

    /* angle from the target to the robot's position */
    CRadians angle2  = (target - GetPosition()).Angle();

    /* heading = angle1 - angle2 = 0.0 when the robot is facing its target */
    CRadians heading = (angle1 - angle2).SignedNormalize();

    //AngleToleranceInRadians.Set(CRadians(-0.09), CRadians(0.09));
    //LOG << "angle1: " << angle1 << endl;
    //LOG << "angle2: " << angle2 << endl;
    //LOG << "heading: " << heading << endl;
    //LOG << AngleToleranceInRadians << endl;

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
 *
 *****/
 void iAnt_controller::GetTargets(int indx){
   
    //for(int i = indx; i<=pattern.size(); i++){
        
        if (TargetHit() == true){
            //switch (pattern[indx]){    
                case 'N':
                    SetTargetN('N');
                    //ApproachTheTarget();
                break;
                case 'E':
                    SetTargetE('E');
                    //ApproachTheTarget();
                break;
                case 'W':
                    SetTargetW('W');
                    //ApproachTheTarget();
                break;
                case 'S':
                    SetTargetS('S');
                    //ApproachTheTarget();
                break;
            }
        }        
    //}
}

/*****
 *
 *****/
 bool iAnt_controller::TargetHit(){
    CVector2 position = GetPosition();
    bool hit = false;
    //cout << "target postion: ";
    //cout << target << '\n';
    if((position-target).SquareLength() < 0.01){
        hit = true;
    }
    return hit;
 }

/*****
 * After pressing the reset button in the GUI, this controller will be set to
 * default factory settings like at the start of a simulation.
 *****/
void iAnt_controller::Reset() {
    ////ADDED//////
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
