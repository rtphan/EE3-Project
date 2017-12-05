/* Orientation is determined from top-down view */
#define IR1 A7  // Left IR
#define IR2 A6  // Middle IR
#define IR3 A5  // Right IR

#define rLED 4  // Red LED
#define gLED 7  // Green LED
#define bLED 8  // Blue LED

#define L_MOTOR 3 // Left Motor
#define R_MOTOR 5 // Right Motor

/* Motor constants */
const int basespeedR = 112;
const int basespeedL = 100; 

/* Declaration of the << operator */
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

/* Global variables for setup */
const int DT = 1;
int offs[3];
static int avg = 0;

/* Global variables for loop */
static int rmotor = 0, lmotor = 0; 
static unsigned tick_cnt = 0;

/* Holds PID values */
struct pid { 
  double integral;
  int prev = 0;
  double kp = 0.7; //the ks should be negative to counteract error
  double ki = 0.08;
  double kd = 0.8;
};

/* Global variables for process */
pid lman, rman;
static bool detectedexit = 0;

/* Holds IR values */
struct ir_in {
  int x, y, z;
};

/* Resets PID */
void resetPid(struct pid *e) {
  e->integral = 0;
  e->prev = 0;
}

/* I dont really know */
int getFix(struct pid *e, int error) {
  double d = (error - e->prev)/DT;
  e->integral += error * DT;
  e->prev = error;
  int r = abs(e->kp * error + e->ki * e->integral + e->kd * d);
  //Serial << "GetFix: " << r << " \n";
  return r;
}

/* Reads IR values */
inline struct ir_in ir_read(){
  ir_in ir_struct;
  int x = analogRead(IR1);
  int y = analogRead(IR2);
  int z = analogRead(IR3);

  if(x > offs[0])
    x = offs[0];
  if(y > offs[1])
    y = offs[1];
  if(z > offs[2])
    z = offs[2];
  
  ir_struct.x = map(x, 0, offs[0], 255, 0);
  ir_struct.y = map(y, 0, offs[1], 255, 0);
  ir_struct.z = map(z, 0, offs[2], 255, 0);

  return ir_struct;
}

void process(ir_in& ir_struct, int& rmotor, int& lmotor) {
  int rError = abs(ir_struct.y - ir_struct.x);
  int lError = abs(ir_struct.y - ir_struct.z);
  //Serial << "x1:  " << ir_struct.x << "  y1:  " << ir_struct.y << "  z1:  " << ir_struct.z << "  \n";  //Previously in loop to test IR
  
  /* Threshold level (Above 40 means black detected, below that means white) */
  const int level = 50; 
  
  if(ir_struct.x > level) {
    if(ir_struct.y > level) {
      /* Left IR + Middle IR + Right IR : STOP*/
      if(ir_struct.z > level) {
        lmotor = 0;
        rmotor = 0;
        digitalWrite(bLED, HIGH);
        digitalWrite(rLED, HIGH);
        digitalWrite(gLED, LOW);
        detectedexit = 1;
      }
      /* Left IR + Middle IR : LEFT */
      else {
        digitalWrite(bLED, HIGH);
        digitalWrite(rLED, LOW);
        digitalWrite(gLED, LOW);
        rmotor = constrain(int(basespeedR + getFix(&rman, rError)), basespeedR/2, 220);
        lmotor = constrain(int(basespeedL - getFix(&lman, lError)), basespeedL/2, 220);
      }
    }
    /* Left IR : LEFT */
    else {
      digitalWrite(bLED, HIGH);
      digitalWrite(rLED, LOW);
      digitalWrite(gLED, LOW);
      rmotor = (constrain(int(basespeedR + getFix(&rman, rError)), basespeedR/2, 220));
      lmotor = (constrain(int(basespeedL - getFix(&lman, lError)), basespeedL/2, 220));
    }
    }
  else if(ir_struct.y > level + 40) {
    /* Middle IR + Right IR : RIGHT*/
    if(ir_struct.z > level + 7) {
      digitalWrite(bLED, LOW);
      digitalWrite(gLED, LOW);
      digitalWrite(rLED, HIGH);
      lmotor = constrain(int(basespeedL + getFix(&lman, lError)), basespeedL/2, 220);
      rmotor = constrain(int(basespeedR - getFix(&rman, rError)), basespeedR/2, 220);
    }
    /* Middle IR : STRAIGHT */
    else {
      digitalWrite(gLED, HIGH);
      digitalWrite(rLED, LOW);
      digitalWrite(bLED, LOW);
      // Reset pid integral error in order to make PID recalibrated.
      resetPid(&lman); resetPid(&rman);
      rmotor = basespeedR;
      lmotor = basespeedL;
    }  
  }
  /* Right IR : RIGHT */
  else if(ir_struct.z > level - 20) {
    digitalWrite(rLED, HIGH);
    digitalWrite(bLED, LOW);
    digitalWrite(gLED, LOW);
    lmotor = constrain(int(basespeedL + getFix(&lman, lError)), basespeedL/2, 220);
    rmotor = constrain(int(basespeedR - getFix(&rman, rError)), basespeedR/2, 220);
  }
}

void setup() {
  /* Right Motor PID (Used to turn LEFT) */
  rman.kp = 1.3;
  rman.kd = -0.0115; // kd should always be negative on rman and lman
  rman.ki = 0.001;
  
  /* Left Motor PID (Used to turn RIGHT) */
  lman.kp = 1.3;
  lman.kd = -0.0115;
  lman.ki = 0.001;

  /* Set data rate */
  Serial.begin(9600);
  
  /* Pin Configuration */
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(gLED, OUTPUT);
  pinMode(rLED, OUTPUT);
  pinMode(bLED, OUTPUT);
  pinMode(L_MOTOR, OUTPUT);
  pinMode(R_MOTOR, OUTPUT);
  
  /* Get average IR values to reset */
  int nsamples = 3;
  offs[0] = 0;
  offs[1] = 0;
  offs[2] = 0;
  for(int i = 0; i < nsamples; i++) {
    avg += analogRead(IR2);
    offs[0] += analogRead(IR1);
    offs[1] += analogRead(IR2);
    offs[2] += analogRead(IR3);
    //Serial << "avg:" << avg;
  }
  offs[0] = offs[0]/nsamples;
  offs[1] = offs[1]/nsamples;
  offs[2] = offs[2]/nsamples;
  avg = avg/nsamples;
  //Serial << "offs" << offs[0] << " " << offs[1] << "\n";

  /* Test IR sensors */
  //Serial << "x1:  " << ir_struct.x << "  y1:  " << ir_struct.y << "  z1:  " << ir_struct.z << "  \n";  
}

void loop() {
  ir_in ir_struct = ir_read();
  //Serial << "RW: " << ir_struct.rw << "\n LW: " << ir_struct.lw << "\n";
  
  /* Wait so that the IR sensors can read initial values && stop line not detected */
  if(tick_cnt > 50 && !detectedexit) {
    analogWrite(R_MOTOR, rmotor);
    analogWrite(L_MOTOR, lmotor);
    process(ir_struct, rmotor, lmotor);
  }
  /* Used for when stop line was detected */
  else {
    analogWrite(R_MOTOR, LOW);
    analogWrite(L_MOTOR, LOW);
  }
  
  tick_cnt++;
}
