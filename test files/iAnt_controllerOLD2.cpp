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
}

/*****
 * Primary control loop for this controller object. This function will execute
 * the CPFA logic using the CPFA enumeration flag once per frame.
 *****/

void iAnt_controller::ControlStep() {
    LOG << GetHeading() << endl << '\n';
    
    /////////ADDED///////////////
    CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.02);
    CVector3 target3d(GetTarget().GetX(), GetTarget().GetY(), 0.02);
    CRay3 targetRay(target3d, position3d);
    data->TargetRayList.push_back(targetRay);

    vector <char> pattern;

    pattern.push_back('N');
    pattern.push_back('E');
    pattern.push_back('S');
    pattern.push_back('S');
    pattern.push_back('S');

    for (unsigned int i = 0; i< pattern.size(); i++)
    {
        CRadians heading = GetHeading().UnsignedNormalize();
        if (pattern[i] == 'N')
        {   //cout << "GoingNorth";
            Tolerance.Set(CRadians::PI-ToRadians(angleInDegrees),
                          CRadians::PI+ToRadians(angleInDegrees)); 
            
           if(north(Tolerance,heading)){
                motorActuator->SetLinearVelocity(RobotForwardSpeed, 
                                                 RobotForwardSpeed);
            }
        }

        else if (pattern[i]=='E')
        {   //cout << "GoingEast";
            Tolerance.Set((CRadians::PI_OVER_TWO*3-ToRadians(angleInDegrees)), 
                          (CRadians::PI_OVER_TWO*3+ToRadians(angleInDegrees)));
            if (east(Tolerance,heading)){
                motorActuator->SetLinearVelocity(RobotForwardSpeed,
                                                 RobotForwardSpeed);
            }
        }
        else if(pattern[i]== 'S')
        {   //cout << "GoingSouth";
            Tolerance.Set((CRadians::PI-ToRadians(angleInDegrees)), 
                          (CRadians::PI+ToRadians(angleInDegrees)));
            if(south(Tolerance,heading)){
                motorActuator->SetLinearVelocity(RobotForwardSpeed,
                                                 RobotForwardSpeed);
            }
        }
        else
        {
            Tolerance.Set(CRadians::PI_OVER_TWO-ToRadians(angleInDegrees),
                         (CRadians::PI_OVER_TWO+ToRadians(angleInDegrees)));
            if(west(Tolerance,heading)){
                motorActuator->SetLinearVelocity(RobotForwardSpeed,
                                                 RobotForwardSpeed);
            }
        }

        // if(north(Tolerance, heading) || south(Tolerance,heading)
        //   ||east(Tolerance, heading) ||  west(Tolerance,heading))
        // {
        //     moveFoward();
        // }
    }

    //[ + + ] move foward
    //motorActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed);
    //[ + - ] rotates to the right
    //motorActuator->SetLinearVelocity(RobotForwardSpeed, -RobotForwardSpeed);
    //[ +  0] spins left
    //motorActuator->SetLinearVelocity(RobotForwardSpeed, 0.0);
    //[ 0  +] spins right
    //motorActuator->SetLinearVelocity(RobotForwardSpeed, 0.0);
}
/*****
 *
 *****/
bool iAnt_controller::north(CRange<CRadians> Tolerance, CRadians head) {
    bool goingNorth = true;
    // cout << "Get min: ";
    // cout << Tolerance.GetMin();
    if(head < Tolerance.GetMin()){
        motorActuator->SetLinearVelocity(RobotTurningSpeed,RobotTurningSpeed);
        goingNorth = false;
    }
    else if (head > Tolerance.GetMax()){
        motorActuator->SetLinearVelocity(RobotTurningSpeed,RobotTurningSpeed);
        goingNorth = false;
    }
    else {
        moveFoward();
    }

    return goingNorth;
}

/*****
 *
 *****/
bool iAnt_controller::south(CRange<CRadians> Tolerance, CRadians head) {
    bool goingSouth = true;
    if(head < Tolerance.GetMin()){
        motorActuator->SetLinearVelocity(-RobotTurningSpeed,RobotTurningSpeed);
        goingSouth = false;
    }
    else if (head > Tolerance.GetMax()) {
        motorActuator->SetLinearVelocity(RobotTurningSpeed,-RobotTurningSpeed);
        goingSouth = false;
    }
    else {
        moveFoward();
    }
    return goingSouth;	
}

/*****
 *
 *****/
bool iAnt_controller::east(CRange<CRadians> Tolerance, CRadians head) {
    bool goingEast = true;
    // cout << "Get min: ";
    // cout << Tolerance.GetMin();
    // cout << "Get max: ";
    // cout << Tolerance.GetMax();
    // cout << "Get head: ";
    // cout << head;
    if(head < Tolerance.GetMin()){
        motorActuator->SetLinearVelocity(RobotTurningSpeed,0.0);
        goingEast = false;
    }
    else if (head> Tolerance.GetMax()){
        motorActuator->SetLinearVelocity(RobotTurningSpeed,0.0);
        goingEast = false;
    }
    else {
        moveFoward();
    }
    return goingEast;
}

// /*****
//  *
//  *****/
bool iAnt_controller::west(CRange<CRadians> Tolerance, CRadians head) {
    bool goingWest = true;
    if(head < Tolerance.GetMin()){
        motorActuator->SetLinearVelocity(0.0,RobotTurningSpeed);
        goingWest = false;
    }
    else if(head > Tolerance.GetMax()){
        motorActuator->SetLinearVelocity(0.0,RobotTurningSpeed);
        goingWest = false;
    }
    else  {
        moveFoward();
    }
	return  goingWest;
}

/*****
 *
 *****/
bool iAnt_controller::moveFoward(){
    bool move = true;
   // cout << "hello move";
    motorActuator->SetLinearVelocity(0.0,0.0);
    UpdateTarget();
    return move;
}

/*****
 * After pressing the reset button in the GUI, this controller will be set to
 * default factory settings like at the start of a simulation.
 *****/
void iAnt_controller::Reset() {
    ////ADDED//////
    target   = data->NestPosition;
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

///////ADDED/////////
CVector2 iAnt_controller::GetTarget() {
    return target;
}

CVector2 iAnt_controller::UpdateTarget(){
    CVector2 curPosition = GetPosition();
    newTarget = target;

    if (curPosition.GetY()-target.GetY() == 0.08){
        newTarget.SetY( target.GetY() + 0.08 ); 
    }
    if (curPosition.GetX()-target.GetX() == 0.08){
        newTarget.SetX( target.GetX() + 0.08);
    }
    if (curPosition.GetY()-target.GetY() == -0.08){
        newTarget.SetX(target.GetY() - 0.08);
    }
    if (curPosition.GetX()-target.GetX() ==-0.08){
        newTarget.SetX(target.GetX() - 0.08);
    }

    //Want to include before UpdateTarget;
    //Refer to line 468 inside SetLocalResourceDensity;
    /*
    if((GetPosition() - target).SquareLength() < 0.01) {
       // target has been reached
    }
    */
    return newTarget;
}
// void iAnt_controller::SetTargetInBounds(CVector2 t) {
//     //* Bound the X value based on the forage range. */
//     if(t.GetX() > data->ForageRangeX.GetMax())
//         t = CVector2(data->ForageRangeX.GetMax(), t.GetY());
    
//     if(t.GetX() < data->ForageRangeX.GetMin())
//         t = CVector2(data->ForageRangeX.GetMin(), t.GetY());
    
//     ///* Bound the Y value based on the forage range. */
//     if(t.GetY() > data->ForageRangeY.GetMax())
//         t = CVector2(t.GetX(), data->ForageRangeY.GetMax());
    
//     if(t.GetY() < data->ForageRangeY.GetMin())
//         t = CVector2(t.GetX(), data->ForageRangeY.GetMin());
    
//     ///* Set the robot's target to the bounded t position. */
//     target = t;
// }

// void iAnt_controller::ApproachTheTarget() {
//     /* angle of the robot's direction relative to the arena's origin */
//     CRadians angle1  = GetHeading();
    
//     /* angle from the target to the robot's position */
//     CRadians angle2  = (target - GetPosition()).Angle();
    
//     /* heading = angle1 - angle2 = 0.0 when the robot is facing its target */
//     CRadians heading = (angle1 - angle2).SignedNormalize();
    
// //    if(IsCollisionDetected() == true) {
// //        collisionDelay = data->SimTime + (data->TicksPerSecond * 2);
// //        
// //        /* turn left */
// //
//     motorActuator->SetLinearVelocity(-RobotTurningSpeed, RobotTurningSpeed);
    
//      if(heading <= AngleToleranceInRadians.GetMin()) {
        
//         /* turn left */
//         motorActuator->SetLinearVelocity(-RobotTurningSpeed, RobotTurningSpeed);

        
//     } else if(heading >= AngleToleranceInRadians.GetMax()) {
        
//         /* turn right */
//         motorActuator->SetLinearVelocity(RobotTurningSpeed, -RobotTurningSpeed);
        
//     } else {
        
//         /* go straight */
//         motorActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed);
//     }
// }

REGISTER_CONTROLLER(iAnt_controller, "iAnt_controller")
