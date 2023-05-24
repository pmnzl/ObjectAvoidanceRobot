//Running Code States
enum RUNNINGSTATE {SETUP, STRAIGHT, ROTATION, STOP};
RUNNINGSTATE RunningState = SETUP;

//Motor Powers
double speed_val_left = 0;
double speed_val_right = 0;
double speed_val_rotate = 0;
double speed_val_setup = 150;

//Power Allocation
double max_power_staight = 250;
double min_power_straight = 100;
double max_power_side = 250;
double power_difference = 100;
double max_power_rotate = 250;
double min_power_rotate = 20;

//Running Code Variables
int turnCount = 0;
int frontTarget = 180;
int sideTarget = 75; //CHANGED FROM: 80 75 // IF DRIFT TO RIGHT, REDUCE //IF DRIFT TO LEFT, INCREASE. 77
int rotateTarget = 90;
double sideError;
double sideErrorTotal = 0; 
double frontError;
double differenceError;
double differenceErrorTotal = 0; 
double rotateError;

double intDistSide = 40; //For ant-integrator windup // 20, 40
double intDistDiff = 40; // 20, 40, 60, 40 

double sideErrorExit = 2; //For exiting setup
double differenceErrorExit = 2;

//Controller Gains
double F_Kp = 4;  //Forward Controller
double S_Kp = 4; //Side Controller, 7
double S_Ki = 0.2; // 0.3 0.18
double D_Kp = 4; //Difference Controller //2, 5,4 
double D_Ki = 0; //1, 2, 4, 5, 10
double R_Kp = 10;  //Rotation Controller


void TaskMain() {
  Serial.println(RunningState);
  
  //FSM for running all tasks required
  switch (RunningState) {
    case SETUP:
      Setup();
      break;
    case STRAIGHT:
      Straight();
      break;
    case ROTATION:
      Rotation();
      break;
    case STOP:
      stop();
      break;
    default:
      break;
  }  
}

int confirmed = 0;
void Setup() {
  //Aligns the side of the robot with the wall
  //speed_val = speed_val_setup;  //Increases motor speed

  //Error calculation
  sideError = IR_LF.getReading() - (sideTarget);
  //sideErrorTotal += sideError;
  if(abs(sideError) < intDistSide){sideErrorTotal += sideError;} // Anti-integrator windup
  
  differenceError = IR_LF.getReading() - IR_LR.getReading();
  //differenceErrorTotal += differenceError;
  if(abs(differenceErrorTotal) < intDistDiff){differenceErrorTotal += abs(differenceError);} // Anti-integrator windup



  double wall_pre = 4*sideError + 0.01 * sideErrorTotal;

  double diff_pre = 4*differenceError + 0.01 * differenceErrorTotal;

int minimal = 30;

  if (wall_pre < 0) {
        wall_pre -= minimal;
    
    //wall_pre = constrain(wall_pre, -1000, -minimal);
  } else {
       wall_pre += minimal;
    //wall_pre = constrain(wall_pre, minimal, 1000);
  }
  if (diff_pre < 0) {
           diff_pre -= minimal;
//    diff_pre = constrain(diff_pre, -1000, -minimal);
  } else {
           diff_pre += minimal;
//    diff_pre = constrain(diff_pre, minimal, 1000);
  }


  double wallPower = constrain(wall_pre, -100, 100); //250
  double diffPower = constrain(diff_pre, -250, 250);

  double m1 =  - wallPower - diffPower;
  double m2 =  + wallPower - diffPower;
  double m3 =  + wallPower - diffPower;
  double m4 =  - wallPower - diffPower;

  left_font_motor.writeMicroseconds(1500 - wallPower - diffPower);
  left_rear_motor.writeMicroseconds(1500 + wallPower - diffPower);
  right_rear_motor.writeMicroseconds(1500 + wallPower - diffPower);
  right_font_motor.writeMicroseconds(1500 - wallPower - diffPower);



  //if (differenceError > 20) return ccw();
  //if (sideError > 0) return strafe_left();
  if (abs(differenceError) < differenceErrorExit && abs(sideError) < sideErrorExit) { //&& abs(differenceError) < dfferenceErrorExit) { //UNCOMMENTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
     confirmed += 1;
    
 
  } else {
    confirmed = 0;
  }

  if (confirmed == 10){
       RunningState = STRAIGHT;
    sideErrorTotal = 0; 
    differenceErrorTotal = 0; 
  }

 

  //Logic to get robot properly aligned for driving straight
//  if (IR_LF.getReading() > 130 && IR_LR.getReading() > 130) {strafe_left();}
//  else if (sideError > 5 && abs(differenceError) < 10) {strafe_left();}
//  else if (differenceError > 2) {ccw();}
//  else if (differenceError < -2) {cw();}
//  else {
//    //FSM state exit condition
//    stop();
//    RunningState = STRAIGHT;
//  }
}

void Straight() {
  //Implements three different P controllers in order to drive the robot straight with minimal deviation
  //Error calculation
  frontError = IR_FR.getReading() - frontTarget;
  sideError = IR_LF.getReading() - sideTarget;
  differenceError = IR_LF.getReading() - IR_LR.getReading();

  //Controller power calculation
  double power_front = constrain(F_Kp*frontError, min_power_straight, max_power_staight);
  double power_side = constrain(abs(2*S_Kp*sideError + S_Ki*sideError), 0, 500); //+ constrain(D_Kp*differenceError, -1*power_difference, power_difference);

  double base_speed_left = (1750 - constrain(abs(4*sideError), 0, 200));
  double base_speed_right = (1250 + constrain(abs(4*sideError), 0, 200));

  //Motor speeds
  speed_val_left = base_speed_left - sideError * 2;
  speed_val_right = base_speed_right - sideError * 2;

  Serial.print(power_side); Serial.print(" | "); Serial.println(speed_val_right);
  
  if (frontError > 0) {
    SetMotorSpeed(speed_val_left, speed_val_right);
  }
  else {
    //FSM state exit condition
    stop();
    GYRO.setAngle(0);
    GYRO.initialise();
    if (turnCount >= 3) {RunningState = STOP;}
    else {RunningState = ROTATION;}   
  }
}


void Rotation() {
  //Rotation P controller used to rotate the robot 90 degrees
  rotateError = rotateTarget - GYRO.getReading();
  if (rotateError > 5){
    speed_val_rotate = constrain(1500 + R_Kp*rotateError, 1500 + min_power_rotate, 1500 + max_power_rotate);
    SetMotorSpeed(speed_val_rotate, speed_val_rotate);
  }
  else {
    //FSM state exit condition
    turnCount++;

    confirmed = 0;
    sideErrorTotal = 0; 
    differenceErrorTotal = 0; 
    
    RunningState = SETUP;
  }
}


void SetMotorSpeed(double speed_val_left, double speed_val_right) {
  //Function to set robot motor speeds for the left and the right motors
  speed_val_left = constrain(speed_val_left, 750, 2250);
  speed_val_right = constrain(speed_val_right, 750, 2250);

  left_font_motor.writeMicroseconds(speed_val_left);
  left_rear_motor.writeMicroseconds(speed_val_left);
  right_rear_motor.writeMicroseconds(speed_val_right);
  right_font_motor.writeMicroseconds(speed_val_right);
}


void GetSensorReadings() {
  //Use in serial plotter with nothing else printing to serial
  SerialCom->print(IR_FL.getReading());
  SerialCom->print(" ");
  SerialCom->print(IR_FR.getReading());
  SerialCom->print(" ");
  SerialCom->print(IR_LF.getReading());
  SerialCom->print(" ");
  SerialCom->print(IR_LR.getReading());
  SerialCom->print(" ");
  SerialCom->print(ULTRA.getReading());
  SerialCom->print(" ");
  SerialCom->print(GYRO.getReading());
  SerialCom->println();
}
