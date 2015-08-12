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
    
    ///////////////ADDED////////////////////
     CDegrees angleInDegrees;
    GetNodeAttribute(iAnt_params, "AngleToleranceInDegrees", angleInDegrees);
    
    AngleToleranceInRadians.Set(-ToRadians(angleInDegrees),
                                ToRadians(angleInDegrees));
    tolerance.Set(-(CRadians::PI_OVER_TWO+x),-(CRadians::PI_OVER_TWO-x));
   

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

    //[ + + ] move foward
    //motorActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed);
    //[ + - ] rotates to the right
    //motorActuator->SetLinearVelocity(RobotForwardSpeed, -RobotForwardSpeed);
    //[ +  0] spins left
    //motorActuator->SetLinearVelocity(RobotForwardSpeed, 0.0);
    //[ 0  +] spins right
    //motorActuator->SetLinearVelocity(RobotForwardSpeed, 0.0);
    
    
	east();
    //west();
    //north();
    //south();
    

    //char* pat = LinearSpiral();
    //out << "pat" << *pat << '\n';
    //free(pat);

    //checkDistanceX();   
    // if(west() == true){ 
    //     motorActuator->SetLinearVelocity(stopMoving(checkDistanceX()), stopMoving(checkDistanceX()));
    //     //motorActuator->SetLinearVelocity(RobotForwardSpeed,RobotForwardSpeed);
    // } 
    // if(west() == true) {
    //     motorActuator->SetLinearVelocity(RobotForwardSpeed,RobotForwardSpeed);
    // }
//    if(east() == true){
//        motorActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed);     
//    }


    //UNCONMMENT
     bool stop=checkDistanceY();
     cout << "stop: " << stop << '\n';
     stopMoving(stop);
     motorActuator->SetLinearVelocity(stopMoving(stop), stopMoving(stop));
     if (stop)
     {    
         east(); 
         motorActuator->SetLinearVelocity(0.0,RobotForwardSpeed);
         if(east() == true) {
             motorActuator->SetLinearVelocity(RobotForwardSpeed,RobotForwardSpeed);
         }
         stop = false;
     } 



    //motorActuator->SetLinearVelocity(stopMoving(stop), stopMoving(stop));
    //north();
    // if(north() == true) { 
    //     //motorActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed); 
    //      motorActuator->SetLinearVelocity(stopMoving(checkDistanceY()), stopMoving(checkDistanceY()));
    // }
    // if(south() == true) {
    //     checkDistanceY();  
    //     motorActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed); 
        
    // } 
    
}
/*****
 *
 *****/
bool iAnt_controller::checkDistanceX() {
    cout << "checkDistanceX\n";
    CVector2 curPosition = GetPosition();
    //cout << "line 87\n";
    cout << curPosition << "curPositionX\n";
    bool stop = false;
    //cout << "nest:" << data ->NestPosition;
    float distanceX = curPosition.GetY()- data->NestPosition.GetY();
    cout << "distance: " << distanceX << "\n";
    if (distanceX > 0.3)
    {   cout << "STOP!";
        stop = true;
    }
    return stop;
}

/*****
 *
 *****/
bool iAnt_controller::checkDistanceY() {
    cout << "checkDistanceY\n";
    CVector2 curPosition = GetPosition();
    cout << "curPositionY" << curPosition << '\n';
    bool stop = false;
    //cout << "curPosition" << curPosition.GetY() << '\n';
    float distanceY = curPosition.GetX()- data->NestPosition.GetX();
    cout << "distance: " << distanceY << "\n";
    if (distanceY > 0.3)
    {   cout << "line 115";
        stop = true;
    }
    return stop;
}
float iAnt_controller::stopMoving(bool stop) {
    cout << "stopMoving\n";
    if (stop == true)
    {
        RobotForwardSpeed = 0.0;
    }
    return RobotForwardSpeed;
 }

/*****
 *
 *****/
bool iAnt_controller::north() {
	cout << "Moving north" <<   '\n';
    bool amIGoingNorth = true;
    CRadians heading = GetHeading().UnsignedNormalize();
    CRadians tolerance(0.09);

    if(heading < CRadians::PI - tolerance) {
        motorActuator->SetLinearVelocity(RobotTurningSpeed, RobotTurningSpeed);
        amIGoingNorth = false;
    } else if(heading > CRadians::PI + tolerance) {
        motorActuator->SetLinearVelocity(RobotTurningSpeed, RobotTurningSpeed);
        amIGoingNorth = false;
    }else {
        motorActuator->SetLinearVelocity(0.0, 0.0);
        amIGoingNorth = true;
    }
    return amIGoingNorth;
}

/*****
 *
 *****/
bool iAnt_controller::south() {
	cout << "Moving south" << '\n';
    bool amIGoingSouth = true;
    CRadians heading = GetHeading().UnsignedNormalize();
    CRadians tolerance(0.09);

    if(heading < CRadians::PI - tolerance) {
        motorActuator->SetLinearVelocity(-RobotTurningSpeed, RobotTurningSpeed);
        amIGoingSouth = false;
    } 
    else if(heading > CRadians::PI + tolerance) {
        motorActuator->SetLinearVelocity(RobotTurningSpeed, -RobotTurningSpeed);
        amIGoingSouth = false;
    }
    else {
        motorActuator->SetLinearVelocity(0.0, 0.0);
        amIGoingSouth = true;
    }

    return amIGoingSouth;
}

/*****
 *
 *****/
bool iAnt_controller::east() {
	cout << "Moving east" << '\n';
    bool amIGoingEast = true;
    CRadians heading = GetHeading().UnsignedNormalize();
    CRadians tolerance(0.09);
    CRadians threePIHalves = (CRadians::PI*3/2);
//    CRadians negPI = (CRadians::PI)*-1;
    if(heading < threePIHalves-tolerance) {
        motorActuator->SetLinearVelocity(RobotTurningSpeed,0.0);
        amIGoingEast = false;
     }else if(heading > threePIHalves+tolerance) {
        motorActuator->SetLinearVelocity(RobotTurningSpeed,0.0);
        amIGoingEast = false;
    }else {
        motorActuator->SetLinearVelocity(0.0, 0.0);
        amIGoingEast = true;
    }
    return amIGoingEast;
}

// /*****
//  *
//  *****/
bool iAnt_controller::west() {
	cout << "Moving west" << '\n';
    bool amIGoingWest = true;
    CRadians heading = GetHeading().UnsignedNormalize();
    CRadians tolerance(0.09);

    if(heading < CRadians::PI/2- tolerance) {
        motorActuator->SetLinearVelocity(0.0, RobotTurningSpeed);
        amIGoingWest = false;
    } else if(heading > CRadians::PI/2 + tolerance) {
        motorActuator->SetLinearVelocity(0.0, RobotTurningSpeed);
        amIGoingWest = false;
    }else {
        motorActuator->SetLinearVelocity(0.0, 0.0);
        amIGoingWest = true;
    }
    return amIGoingWest;
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

void iAnt_controller::SetTargetInBounds(CVector2 t) {
    /* Bound the X value based on the forage range. */
    if(t.GetX() > data->ForageRangeX.GetMax())
        t = CVector2(data->ForageRangeX.GetMax(), t.GetY());
    
    if(t.GetX() < data->ForageRangeX.GetMin())
        t = CVector2(data->ForageRangeX.GetMin(), t.GetY());
    
    /* Bound the Y value based on the forage range. */
    if(t.GetY() > data->ForageRangeY.GetMax())
        t = CVector2(t.GetX(), data->ForageRangeY.GetMax());
    
    if(t.GetY() < data->ForageRangeY.GetMin())
        t = CVector2(t.GetX(), data->ForageRangeY.GetMin());
    
    /* Set the robot's target to the bounded t position. */
    target = t;
}

void iAnt_controller::ApproachTheTarget() {
    /* angle of the robot's direction relative to the arena's origin */
    CRadians angle1  = GetHeading();
    
    /* angle from the target to the robot's position */
    CRadians angle2  = (target - GetPosition()).Angle();
    
    /* heading = angle1 - angle2 = 0.0 when the robot is facing its target */
    CRadians heading = (angle1 - angle2).SignedNormalize();
    
//    if(IsCollisionDetected() == true) {
//        collisionDelay = data->SimTime + (data->TicksPerSecond * 2);
//        
//        /* turn left */
//
    motorActuator->SetLinearVelocity(-RobotTurningSpeed, RobotTurningSpeed);
    
     if(heading <= AngleToleranceInRadians.GetMin()) {
        
        /* turn left */
        motorActuator->SetLinearVelocity(-RobotTurningSpeed, RobotTurningSpeed);
        
    } else if(heading >= AngleToleranceInRadians.GetMax()) {
        
        /* turn right */
        motorActuator->SetLinearVelocity(RobotTurningSpeed, -RobotTurningSpeed);
        
    } else {
        
        /* go straight */
        motorActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed);
    }
}




REGISTER_CONTROLLER(iAnt_controller, "iAnt_controller")
