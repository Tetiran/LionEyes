/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

const int BUFFER_SIZE = 100;
char buf[BUFFER_SIZE];
// angle in x plane away from center of camera
double theta_x=0;
// angle in y plane away from center of camera
double theta_y=0;
// angle in rotation of eyes about center of head
double theta_r=0;

int deadband = 15;

int headrollcenter = 90; //93
int headrollrange[] = {80, 105};
int leftyawcenter = 77; //90
int leftpitchcenter = 95; //90
int rightyawcenter = 87; //100
int rightpitchcenter = 115; //105
int neckyawcenter = 130;
int neckpitchcenter = 180;
int headpitchcenter = 55;

int uppereyelidposes[] = {90,25};
int lowereyelisposes[] = {75, 25};

float headrollcommand = headrollcenter;
float leftyawcommand = leftyawcenter;
float leftpitchcommand = leftpitchcenter;
float rightyawcommand = rightyawcenter;
float rightpitchcommand = rightpitchcenter;
float neckyawcommand = neckyawcenter;
float neckpitchcommand = neckpitchcenter;
float headpitchcommand = headpitchcenter;

float headrollprev = headrollcenter;
float leftyawprev = leftyawcenter;
float leftpitchprev = leftyawcenter;
float rightyawprev = rightyawcenter;
float rightpitchprev = rightpitchcenter;
float neckyawprev = neckyawcenter;
float neckpitchprev = neckpitchcenter;
float headpitchprev = headpitchcenter;

int routine = 5;
float prevsmooth = 0.95;
float comsmooth = 1-prevsmooth;
int duration = 4;

float neckposlast = 0;
int pos = 0;    // variable to store the servo position

const int resolution = 13;
const int freq = 50;

const int LYawChannel = 1;
const int RYawChannel = 2;
const int LPitchChannel = 3;
const int RPitchChannel = 4;
const int NeckYawChannel = 5;
const int NeckPitchChannel = 6;
const int HeadRollChannel = 7;
const int HeadPitchChannel = 8;
const int UpperEyelidChannel = 9;
const int LowerEyelidChannel = 10;

const int LYawPin = 21;
const int RYawPin = 22;
const int LPitchPin = 19;
const int RPitchPin = 23;
const int NeckYawPin = 18;
const int NeckPitchPin = 27;
const int HeadRollPin = 33;
const int HeadPitchPin = 32;
const int UpperEyelidPin = 4;
const int LowerEyelidPin = 13;


int blink_state = 0;
long last_blink_time = 0;
long LastHeadRollTime = 0;
float CurrBlinkPercent = 0.0;
float TargetBlinkPercent = 0.0;
float EndBlinkPercent = 0.0;
int BlinkSpeed = 0;
float BlinkStep = 0.0;

float HeadRateLimit = 0.5;

const float BlinkUpdateFreq = 20.0;
const float RollUpdateFreq = 40.0;


int angleToDC(float angle){
     return ((angle - 0.0) * (2.0) * (820.0 - 410.0) / (180.0 - 0.0)) + 205.0;
}

void blink(float ClosePercent, float EndPercent, float step);

void UpdateBlink();

void UpdateRoll();

void setup() {

  ledcSetup(LYawChannel, freq, resolution);
  ledcSetup(RYawChannel, freq, resolution);
  ledcSetup(LPitchChannel, freq, resolution);
  ledcSetup(RPitchChannel, freq, resolution);
  ledcSetup(NeckYawChannel, freq, resolution);
  ledcSetup(NeckPitchChannel, freq, resolution);
  ledcSetup(HeadRollChannel, freq, resolution);
  ledcSetup(HeadPitchChannel, freq, resolution);
  ledcSetup(UpperEyelidChannel, freq, resolution);
  ledcSetup(LowerEyelidChannel, freq, resolution);
  
  ledcAttachPin(LYawPin, LYawChannel);
  ledcAttachPin(RYawPin, RYawChannel);
  ledcAttachPin(LPitchPin, LPitchChannel);
  ledcAttachPin(RPitchPin, RPitchChannel);
  ledcAttachPin(NeckYawPin, NeckYawChannel);
  ledcAttachPin(NeckPitchPin, NeckPitchChannel);
  ledcAttachPin(HeadRollPin, HeadRollChannel);
  ledcAttachPin(HeadPitchPin, HeadPitchChannel);
  ledcAttachPin(UpperEyelidPin, UpperEyelidChannel);
  ledcAttachPin(LowerEyelidPin, LowerEyelidChannel);

  ledcWrite(LYawChannel, angleToDC(leftyawcenter));
  ledcWrite(RYawChannel, angleToDC(rightyawcenter));
  ledcWrite(LPitchChannel, angleToDC(leftpitchcenter));
  ledcWrite(RPitchChannel, angleToDC(rightpitchcenter));
  ledcWrite(NeckYawChannel, angleToDC(neckyawcenter));
  ledcWrite(NeckPitchChannel, angleToDC(neckpitchcenter));
  ledcWrite(HeadRollChannel, angleToDC(headrollcenter));
  ledcWrite(HeadPitchChannel, angleToDC(headpitchcenter));
  ledcWrite(UpperEyelidChannel, angleToDC(uppereyelidposes[0]));
  ledcWrite(LowerEyelidChannel, angleToDC(lowereyelisposes[0]));

  delay(1000);
  Serial.begin(115200);
}

void loop() {
  if(routine == 5){
    while (Serial.available() > 0) {
      // read the incoming bytes:
      int rlen = Serial.readBytesUntil('\0', buf, BUFFER_SIZE);
      Serial.println(buf);
      if (buf[0] == 'x'){
          theta_x = atof(buf+1);
      } else if (buf[0] == 'y'){
          theta_y = atof(buf+1);
      } else if (buf[0] == 'f'){
          theta_r = atof(buf+1);
      }
    }
  }
  if (routine == 11){
    blink(.8, 0.1, .02);
  }

  if(routine==0){
    ledcWrite(LYawChannel, angleToDC(leftyawcenter));
    ledcWrite(RYawChannel, angleToDC(rightyawcenter));
    ledcWrite(LPitchChannel, angleToDC(leftpitchcenter));
    ledcWrite(RPitchChannel, angleToDC(rightpitchcenter));
    ledcWrite(NeckYawChannel, angleToDC(neckyawcenter));
    ledcWrite(NeckPitchChannel, angleToDC(neckpitchcenter));
    ledcWrite(HeadRollChannel, angleToDC(headrollcenter));
    ledcWrite(HeadPitchChannel, angleToDC(headpitchcenter));
    delay(500);
  }
  
  //Set routine
  if(routine==1){
    delay(500);
    leftyawcommand = leftyawcenter+60;
    rightyawcommand = rightyawcenter+60;
    leftpitchcommand = leftpitchcenter+40;
    rightpitchcommand = rightpitchcenter+40;
    headrollcommand = headrollcenter+30;
    headpitchcommand = headpitchcenter+10;
    neckyawcommand = neckyawcenter+20;
    neckpitchcommand = neckpitchcenter+20;
    servocontrol(leftyawcommand,rightyawcommand,leftpitchcommand,rightpitchcommand,headrollcommand,headpitchcommand,neckyawcommand,neckpitchcommand,duration);
    servocontrol(leftyawcenter,rightyawcenter,leftpitchcenter,rightpitchcenter,headrollcenter,headpitchcenter,neckyawcenter,neckpitchcenter,duration);
    routine=2;
  }
  if(routine==2){
    delay(500);
    leftyawcommand = leftyawcenter-60;
    rightyawcommand = rightyawcenter-60;
    leftpitchcommand = leftpitchcenter+40;
    rightpitchcommand = rightpitchcenter+40;
    headrollcommand = headrollcenter-30;
    headpitchcommand = headpitchcenter-10;
    neckyawcommand = neckyawcenter-20;
    neckpitchcommand = neckpitchcenter-20;
    servocontrol(leftyawcommand,rightyawcommand,leftpitchcommand,rightpitchcommand,headrollcommand,headpitchcommand,neckyawcommand,neckpitchcommand,duration);
    servocontrol(leftyawcenter,rightyawcenter,leftpitchcenter,rightpitchcenter,headrollcenter,headpitchcenter,neckyawcenter,neckpitchcenter,duration);
    routine=1;    
  }
  if(routine==3){
    delay(500);
    leftyawcommand = leftyawcenter;
    rightyawcommand = rightyawcenter;
    leftpitchcommand = leftpitchcenter-40;
    rightpitchcommand = rightpitchcenter-40;
    headrollcommand = headrollcenter;
    servocontrol(leftyawcommand,rightyawcommand,leftpitchcommand,rightpitchcommand,headrollcommand,headpitchcommand,neckyawcommand,neckpitchcommand,duration);
    servocontrol(leftyawcenter,rightyawcenter,leftpitchcenter,rightpitchcenter,headrollcenter,headpitchcommand,neckyawcommand,neckpitchcommand,duration);
    routine=4;     
  }
  if(routine==4){
    delay(500);
    leftyawcommand = leftyawcenter;
    rightyawcommand = rightyawcenter;
    leftpitchcommand = leftpitchcenter+40;
    rightpitchcommand = rightpitchcenter+40;
    headrollcommand = headrollcenter;
    servocontrol(leftyawcommand,rightyawcommand,leftpitchcommand,rightpitchcommand,headrollcommand,headpitchcommand,neckyawcommand,neckpitchcommand,duration);
    servocontrol(leftyawcenter,rightyawcenter,leftpitchcenter,rightpitchcenter,headrollcenter,headpitchcommand,neckyawcommand,neckpitchcommand,duration);
    routine=1;     
  }
  if (routine==5){
    leftyawcommand = leftyawcenter;
    rightyawcommand = rightyawcenter;
    neckyawcommand = neckyawcenter;

    if (theta_x > neckposlast + deadband){
      neckposlast = theta_x - deadband;
    } else if (theta_x < neckposlast - deadband){
      neckposlast = theta_x + deadband;
    }
    //Serial.println(neckposlast);
    neckyawcommand = neckyawcenter + neckposlast;
    leftyawcommand = leftyawcenter + (theta_x - neckposlast);
    rightyawcommand = rightyawcenter + (theta_x - neckposlast);
    leftpitchcommand = leftpitchcenter + theta_y;
    rightpitchcommand = rightpitchcenter + theta_y;
    
    //headpitchcommand = headpitchcenter;
    //neckpitchcommand = neckpitchcenter;
    UpdateRoll();
    ledcWrite(LYawChannel, angleToDC(leftyawcommand));
    ledcWrite(RYawChannel, angleToDC(rightyawcommand));
    ledcWrite(LPitchChannel, angleToDC(leftpitchcommand));
    ledcWrite(RPitchChannel, angleToDC(rightpitchcommand));
    //servocontrol(leftyawcommand,rightyawcommand,leftpitchcommand,rightpitchcommand,headrollcommand,headpitchcommand,neckyawcommand,neckpitchcommand,duration);
    /*
    leftyaw.write(leftyawcommand);
    rightyaw.write(rightyawcommand);
    leftpitch.write(leftpitchcommand);
    rightpitch.write(rightpitchcommand);
    headroll.write(headrollcommand);
    headpitch.write(headpitchcommand);
    neckyaw.write(neckyawcommand);
    neckpitch.write(neckpitchcommand);*/

  }
  if(routine == 10){
    static bool state = false;
    static int channel = -1;
    if (Serial.available()> 0){
      int rlen = Serial.readBytesUntil('\n', buf, BUFFER_SIZE);
      buf[rlen] = '\n';
      //Serial.println(buf);
      int val = atoi(buf);
      if (!state){
        Serial.print("selecting channel ");
        Serial.println(val);
        channel = val;
        state = true;
      } else{
        Serial.print("writing value ");
        Serial.print(val);
        Serial.print("on channel ");
        Serial.println(channel);
        ledcWrite(channel, angleToDC(val));
        state = false;
      }
    }
  }
  UpdateBlink();
}

void UpdateRoll(){
  if (LastHeadRollTime + 1000.0/RollUpdateFreq < millis() ){
    LastHeadRollTime = millis();
    float diff = headrollcommand - (headrollcenter + theta_r/2.0);
    if (diff > 0){
      headrollcommand -= HeadRateLimit;
    } else{   
      headrollcommand += HeadRateLimit;
    }
    headrollcommand = constrain(headrollcommand, headrollrange[0], headrollrange[1]);
    Serial.println(headrollcommand);
    ledcWrite(HeadRollChannel, angleToDC(headrollcommand));
  }
}

void UpdateBlink(){
  if(blink_state !=0){
    if (last_blink_time + 1000.0/BlinkUpdateFreq < millis() ){
      last_blink_time = millis();
      float target = 0;
      if(blink_state == 1){
        target = TargetBlinkPercent;
      } else if (blink_state == 2){
        target = EndBlinkPercent;
      } else if (blink_state == 3){
        blink_state = 0;
      }
      if (blink_state == 1 || blink_state == 2){
        float diff = target - CurrBlinkPercent;
        if (diff>0){
          CurrBlinkPercent+= (BlinkStep);
          if (CurrBlinkPercent>=target){
            blink_state++;
          }
        } else{
          CurrBlinkPercent-= (BlinkStep);
          if (CurrBlinkPercent<=target){
            blink_state++;
          }
        }
        ledcWrite(UpperEyelidChannel, angleToDC(CurrBlinkPercent * (uppereyelidposes[1]-uppereyelidposes[0]) + uppereyelidposes[0]));
        ledcWrite(LowerEyelidChannel, angleToDC(CurrBlinkPercent * (lowereyelisposes[1]-lowereyelisposes[0]) + lowereyelisposes[0]));
      }
    }
  }
}

void blink(float ClosePercent, float EndPercent, float step){
  if (blink_state == 0){
    blink_state = 1;
    TargetBlinkPercent = ClosePercent;
    EndBlinkPercent = EndPercent;
    BlinkStep = step;
  }
}

void servocontrol(int leftyawcommand, int rightyawcommand, int leftpitchcommand, int rightpitchcommand,int headrollcommand, int headpitchcommand, int neckyawcommand, int neckpitchcommand, int duration){
     for(int i=1; i<=duration; i += 1){ 
      leftyawprev=leftyawprev*prevsmooth+leftyawcommand*comsmooth;
      rightyawprev=rightyawprev*prevsmooth+rightyawcommand*comsmooth;
      leftpitchprev=leftpitchprev*prevsmooth+leftpitchcommand*comsmooth;
      rightpitchprev=rightpitchprev*prevsmooth+rightpitchcommand*comsmooth;
      //headrollprev=headrollprev*prevsmooth+headrollcommand*comsmooth;
      headpitchprev=headpitchprev*prevsmooth+headpitchcommand*comsmooth;
      //neckyawprev=neckyawprev*prevsmooth+neckyawcommand*comsmooth;
      neckpitchprev=neckpitchprev*prevsmooth+neckpitchcommand*comsmooth;
      //ledcWrite(LYawChannel, angleToDC(leftyawprev));
      //ledcWrite(RYawChannel, angleToDC(rightyawprev));
      //ledcWrite(LPitchChannel, angleToDC(leftpitchprev));
      //ledcWrite(RPitchChannel, angleToDC(rightpitchprev));
      //ledcWrite(NeckYawChannel, angleToDC(neckyawcenter));
      ledcWrite(NeckPitchChannel, angleToDC(neckpitchprev));
      //ledcWrite(HeadRollChannel, angleToDC(headrollprev));
      ledcWrite(HeadPitchChannel, angleToDC(headpitchprev));
      delay(3);
    }
    //ledcWrite(LYawChannel, angleToDC(leftyawcommand));
    //ledcWrite(RYawChannel, angleToDC(rightyawcommand));
    //ledcWrite(LPitchChannel, angleToDC(leftpitchcommand));
    //ledcWrite(RPitchChannel, angleToDC(rightpitchcommand));
    //ledcWrite(HeadRollChannel, angleToDC(headrollcommand));
    ledcWrite(HeadPitchChannel, angleToDC(headpitchcommand));
    //neckyaw.write(neckyawcommand);
    ledcWrite(NeckPitchChannel, angleToDC(neckpitchcommand));
}
