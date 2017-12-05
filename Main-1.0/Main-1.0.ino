#define IR1 A7//leftir
#define IR2 A6//midir
#define IR3 A5//rightir

#define rLED 4
#define gLED 7
#define bLED 8

#define L_MOTOR 3
#define R_MOTOR 5

#define r_wheel_ir 2
#define l_wheel_ir 1

#define PWM_HIGH_MOTOR 190

const int basespeedR = 112;
const int basespeedL = 110-10; 

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
const int DT = 1;
int offs[5];
int avg = 0;

struct pid { //Note that because we have two different types of distance sensors (Andrew's works a little differently than Jeffrey's we should have two different errors. To stay straight though we can just use one side right?)
  double integral;
  int prev = 0;
  double kp = 0.7; //the ks should be negative to counteract error
  double ki = 0.08;
  double kd = 0.8;
};

struct ir_in{
  int x,y,z,lw,rw;
};

void resetPid(struct pid *e) {
  e->integral = 0;
  e->prev = 0;
}

int ChangeOutput(int input, int changedetected){
  if(changedetected){
      return PWM_HIGH_MOTOR;
    }  
  return input;
}

int getFix(struct pid *e, int error) {
  double d = (error - e->prev)/DT;
  e->integral += error * DT;
  e->prev = error;
  int r = abs(e->kp * error + e->ki * e->integral + e->kd * d);
  //Serial << "GetFix: " << r << " \n";
  return r;
}

inline struct ir_in ir_read(){
  ir_in ir_struct;
  int x = analogRead(IR1);
  int y = analogRead(IR2);
  int z = analogRead(IR3);
  int rw = analogRead(r_wheel_ir);
  int lw = analogRead(l_wheel_ir);
  if(x > offs[0]){
    x = offs[0];
    }
  if(y > offs[1]){
    y = offs[1];
  }
  if(z > offs[2]){
    z = offs[2];
    }
  if (rw > offs[3])
    rw = offs[3];
  if (lw > offs[4])
    lw = offs[4];
  ir_struct.x = map(x, 0, offs[0], 255, 0);
  ir_struct.y = map(y, 0, offs[1], 255, 0);
  ir_struct.z = map(z, 0, offs[2], 255, 0);
  ir_struct.rw = map(rw, 0, offs[3], 255, 0);
  ir_struct.lw = map(lw, 0, offs[4], 255, 0);
  return ir_struct;
}

void blink_5_LEDs(int option){
  if(option == 1){ //blue option
    digitalWrite(L_MOTOR, 150);
    for(int i = 0; i < 5; i++){
     digitalWrite(bLED, HIGH);
     //delay((unsigned long)20);
     digitalWrite(bLED, LOW);
    }
  }
  else if(option == 3){ //red option
    digitalWrite(R_MOTOR,150);
    for(int i = 0; i < 5; i++){
      digitalWrite(rLED, HIGH);
      //delay((unsigned long)20);
      digitalWrite(rLED, LOW);
    }
  }
  else if(option == 2){ //both
    digitalWrite(R_MOTOR, 150);
    digitalWrite(L_MOTOR, 150);
    for(int i = 0; i < 5; i++){
     digitalWrite(bLED, HIGH);
     digitalWrite(rLED, HIGH);
     //delay((unsigned long)20);
     digitalWrite(bLED, LOW);
     digitalWrite(rLED, LOW);
    }
  }
  return;
}

void resetLEDs(){digitalWrite(gLED, LOW); digitalWrite(bLED, LOW); digitalWrite(rLED, LOW);};

void Turn_Perpendicular(){ //Turns left when detects that we're perpendicular to the line. This should hardly go on.
  while(analogRead(IR1) < 40 || analogRead(IR2) < 40 || analogRead(IR3) < 40){
    analogWrite(L_MOTOR, 130);
    analogWrite(R_MOTOR, 0);
    digitalWrite(bLED, HIGH);
    }
    digitalWrite(bLED, LOW);
    analogWrite(L_MOTOR, 0);
    analogWrite(R_MOTOR, 0);
  }

void time_straight(unsigned int t_time){
  for(int i = 0; i < t_time; i++){
     digitalWrite(gLED, HIGH);
     digitalWrite(rLED, LOW);
     digitalWrite(bLED, LOW);
     digitalWrite(R_MOTOR, basespeedR);
     digitalWrite(L_MOTOR, basespeedL);
    }
}

pid lman; pid rman;
int motor_readings1[8]; 
int motor_readings2[8];
int m_increment = 0;
int changedetected = 0;
int changedetected1 = 0;
int lastaction = 0; //0 turn right, 1 turn left
bool detectedexit = 0;
void process(ir_in& ir_struct, int& rmotor, int& lmotor){
  const int tolerance_level = 20;/*
  if(m_increment == 7){
    for(int i = 0; i < 8; i++){
      if(!changedetected && (motor_readings1[i] - motor_readings1[0] > tolerance_level))
        {
          changedetected = 1; //true
          break;
        }
      else 
        changedetected = 0; //false
    }//We should insert a function that jumpstarts the motors, or just add a constant value to the PID instead?
    for(int j = 0; j < 8; j++){
       if(!changedetected1 && (motor_readings2[j] - motor_readings2[0] > tolerance_level))
        {
          changedetected1 = 1; //true
          break;
        }
      else
        changedetected1 = 0; //false
      }
      //blink_5_LEDs((int)changedetected1*2 + (int)changedetected); //Only occurs at the end of the array
    }*/
  changedetected = 0; //debug to stop this portion
  changedetected1 = 0;
  int rError = abs(ir_struct.y - ir_struct.x);
  int lError = abs(ir_struct.y - ir_struct.z);
//  Serial << "x1:  " << ir_struct.x << "  y1:  " << ir_struct.y << "  z1:  " << ir_struct.z << "  \n";  //Previously in loop to test IR
  const int level = 50; //Threshold level. Above 40 means black detected, below that means white.
  const int level_mid = 58;
  if(ir_struct.x > level){
    if(ir_struct.y > level){
      if(ir_struct.z > level && ir_struct.y > level && ir_struct.z > level){
        //We're perpendicular to the line. Turn right until we hit the line again.
          lmotor = 0;
          rmotor = 0;
          digitalWrite(bLED, HIGH);
          digitalWrite(rLED, HIGH);
          digitalWrite(gLED, LOW);
          detectedexit = 1;
        }
       else{
          lastaction = 1;
          digitalWrite(bLED, HIGH);
          digitalWrite(rLED, LOW);
          digitalWrite(gLED, LOW);
          rmotor = constrain(int(basespeedR + getFix(&rman, rError)), basespeedR/2, 220); //ChangeDetected ->1st -> Right Motor
          lmotor = constrain(int(basespeedL - getFix(&lman, lError)), basespeedL/2, 220); //ChangeDetected1 -> 2nd -> Left Motor
        //We should update PID to go to the left a little bit
        }
      }
    else{
          lastaction = 1;
          digitalWrite(bLED, HIGH);
          digitalWrite(rLED, LOW);
          digitalWrite(gLED, LOW);
          rmotor = (constrain(int(basespeedR + getFix(&rman, rError)), basespeedR/2, 220));
          lmotor = (constrain(int(basespeedL - getFix(&lman, lError)), basespeedL/2, 220));
        //We should update PID to go to the left a little bit
      }
    }
  else{
    if(ir_struct.y > 90){
        lastaction = 3;
        digitalWrite(gLED, HIGH);
        digitalWrite(rLED, LOW);
        digitalWrite(bLED, LOW);
        //We're completely in a straight line. Reset pid integral error in order to make PID recalibrated.
        resetPid(&lman); resetPid(&rman);
        rmotor = basespeedR;
        lmotor = basespeedL;
      }
    if(ir_struct.y > level){
      if(ir_struct.z > level + 7){
        lastaction = 0;
          digitalWrite(bLED, LOW);
          digitalWrite(gLED, LOW);
          digitalWrite(rLED, HIGH);
          lmotor = constrain(int(basespeedL + getFix(&lman, lError)), basespeedL/2, 220);
          rmotor = constrain(int(basespeedR - getFix(&rman, rError)), basespeedR/2, 220);
        //TURN RIGHT
        }
      else{
        lastaction = 3;
        digitalWrite(gLED, HIGH);
        digitalWrite(rLED, LOW);
        digitalWrite(bLED, LOW);
        //We're completely in a straight line. Reset pid integral error in order to make PID recalibrated.
        resetPid(&lman); resetPid(&rman);
        rmotor = basespeedR;
        lmotor = basespeedL;
        }  
      }
     else if(ir_struct.z > 30){//Update PID to go to the left a lot. 
      //TURN RIGHT
          lastaction = 0;
          digitalWrite(rLED, HIGH);
          digitalWrite(bLED, LOW);
          digitalWrite(gLED, LOW);
          lmotor = constrain(int(basespeedL + getFix(&lman, lError)), basespeedL/2, 220);
          rmotor = constrain(int(basespeedR - getFix(&rman, rError)), basespeedR/2, 220);
     }
     else{//No line to follow, we finished (Jumpstart and see where we can go after this)
      /*
      if(lastaction == 0){
          digitalWrite(rLED, HIGH);
          digitalWrite(bLED, LOW);
          digitalWrite(gLED, LOW);
          lmotor = constrain(int(basespeedL + 15), basespeedL/2, 220);
          rmotor = basespeedR;
        }
      else if(lastaction == 1){
          digitalWrite(bLED, HIGH);
          digitalWrite(rLED, LOW);
          digitalWrite(gLED, LOW);
          rmotor = constrain(int(basespeedR + 15), basespeedR/2, 220);
          lmotor = basespeedL;
        }
     */
     }  
  }
}
void setup() {
  rman.kd = -0.0115; //kd should always be negative on rman and lman
  rman.ki = 0.001;
  //rman.ki = 0.01;
  //rman.kp = 4.2;
  rman.kp = 1.3;
  lman.ki = 0.001;
  //lman.ki = 0.01;
  lman.kd = -0.0115;
  //lman.kp = 4.0;
  lman.kp = 1.3;
  
  for(int i = 0; i < 8; i++){motor_readings1[i] = 0; motor_readings2[i] = 0;}
  Serial.begin(9600);
  offs[0] = 0;
  offs[1] = 0;
  offs[2] = 0;
  offs[3] = 0;
  offs[4] = 0;
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);

  pinMode(gLED, OUTPUT);
  pinMode(rLED, OUTPUT);
  pinMode(bLED, OUTPUT);

  pinMode(L_MOTOR, OUTPUT);
  pinMode(R_MOTOR, OUTPUT);
  
  int nsamples = 5;
  for(int i = 0; i < nsamples; i++){
    avg += analogRead(IR2);
    offs[0] += analogRead(IR1);
    offs[1] += analogRead(IR2);
    offs[2] += analogRead(IR3);
    offs[3] += analogRead(r_wheel_ir);
    offs[4] += analogRead(l_wheel_ir);
    Serial << "avg:" << avg;
  }
  offs[0] = offs[0]/nsamples;
  offs[1] = offs[1]/nsamples;
  offs[2] = offs[2]/nsamples;
  offs[3] = offs[3]/nsamples;
  offs[4] = offs[4]/nsamples;
  avg = avg/nsamples;
  Serial << "offs" << offs[0] << " " << offs[1] << "\n";

  //Serial << "x1:  " << ir_struct.x << "  y1:  " << ir_struct.y << "  z1:  " << ir_struct.z << "  \n";  //Previously in loop to test IR
}

int rmotor = 0; int lmotor = 0; unsigned tick_cnt = 0;
void loop() {
  ir_in ir_struct = ir_read();
  //Serial << "RW: " << ir_struct.rw << "\n LW: " << ir_struct.lw << "\n";
  if(tick_cnt > 50 && !detectedexit){
   analogWrite(R_MOTOR, rmotor);
   analogWrite(L_MOTOR, lmotor);
   process(ir_struct, rmotor, lmotor);
  }
  else{
    analogWrite(R_MOTOR, LOW);
    analogWrite(L_MOTOR, LOW);
    //time_straight(500);
   }
  if(tick_cnt % 50){ //Only runs every 50th tick count
    motor_readings1[m_increment] = ir_struct.rw;
    motor_readings2[m_increment] = ir_struct.lw;
    if(m_increment < 7)
     m_increment++;
    else
      m_increment = 0;
  }
  tick_cnt++;
 }
