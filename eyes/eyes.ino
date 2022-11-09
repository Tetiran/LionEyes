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

int headrollcenter = 95;
int headrollrange[] = {80, 105};
int leftyawcenter = 77;
int leftpitchcenter = 95;
int rightyawcenter = 87;
int rightpitchcenter = 97;
int neckyawcenter = 137;
int neckpitchcenter = 100;
int headpitchcenter = 55;

int UpperEyeLidPoses[] = {90,25};
int LowerEyeLidPoses[] = {75, 25};

int routine = 5;
float prevsmooth = 0.95;
float comsmooth = 1-prevsmooth;
int duration = 4;

float neckposlast = 0;
int pos = 0;    // variable to store the servo position

const int resolution = 13;
const int freq = 50;

const int LYawChannel = 0;
const int RYawChannel = 1;
const int LPitchChannel = 2;
const int RPitchChannel = 3;
const int NeckYawChannel = 4;
const int NeckPitchChannel = 5;
const int HeadRollChannel = 6;
const int HeadPitchChannel = 7;
const int UpperEyelidChannel = 8;
const int LowerEyelidChannel = 9;

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
long LastYawTime = 0;
float CurrBlinkPercent = 0.0;
float TargetBlinkPercent = 0.0;
float EndBlinkPercent = 0.0;
int BlinkSpeed = 0;
float BlinkStep = 0.0;

float HeadRateLimit = 0.5;
float YawRateLimit = 0.9;

const float BlinkUpdateFreq = 20.0;
const float RollUpdateFreq = 40.0;
const float YawUpdateFreq = 40.0;

long SquintInitTime =0;
#define SQUINTTIME 3000
int EventCode = 0;

#define NUMCONTROLLERS 10
#define ACTIVEVENT 10

float ControlTarget[NUMCONTROLLERS];
float ControlCurrValue[NUMCONTROLLERS];
long ControlLastTime[NUMCONTROLLERS];
float ControlFreq[NUMCONTROLLERS];
float ControlStepSize[NUMCONTROLLERS];
bool ControlActive[NUMCONTROLLERS];



int angleToDC(float angle){
     return ((angle - 0.0) * (2.0) * (820.0 - 410.0) / (180.0 - 0.0)) + 205.0;
}

void blink(float ClosePercent, float EndPercent, float step);

void UpdateRoll();

void SetControlState(int idx, float Target, float Freq, float StepSize);

void UpdateController(int idx);

void UpdateControllers();

bool QueryControllers();

void UpdateYaw(float target);

#define EVENTQUEUESIZE 10
int EventQueue[EVENTQUEUESIZE];

void setup() {

  for(int i=0; i<EVENTQUEUESIZE; i++){
      EventQueue[i] = -1;
  }

  ControlCurrValue[HeadPitchChannel] = headpitchcenter;
  ControlCurrValue[NeckPitchChannel] = neckpitchcenter;
  ControlCurrValue[NeckYawChannel] = neckyawcenter;
  ControlCurrValue[HeadRollChannel] = headrollcenter;
  ControlCurrValue[UpperEyelidChannel] = UpperEyeLidPoses[0];
  ControlCurrValue[LowerEyelidChannel] = LowerEyeLidPoses[0];

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
  ledcWrite(UpperEyelidChannel, angleToDC(UpperEyeLidPoses[0]));
  ledcWrite(LowerEyelidChannel, angleToDC(LowerEyeLidPoses[0]));

  Serial.begin(115200);
}
long last_send = 0;

void loop() {
  /*
  if(last_send+3000<millis()){
    Serial.print("currently in event");
    Serial.println(EventCode);
    last_send=millis();
  }
  */
  
  if(EventCode != 1){
    while (Serial.available() > 0) {
      int rlen = Serial.readBytesUntil('\0', buf, BUFFER_SIZE);
      buf[rlen] = '\n';
      if (buf[0] == 'x'){
          theta_x = atof(buf+1);
      } else if (buf[0] == 'y'){
          theta_y = atof(buf+1);
      } else if (buf[0] == 'f'){
          theta_r = atof(buf+1);
      } else if (buf[0] == 'e'){
          int nextevent = atoi(buf+1);
          Serial.print("got event ");
          Serial.println(nextevent);
          for(int i=0; i < EVENTQUEUESIZE; i++){
            if(EventQueue[i] == -1){
              Serial.print("putting event ");
              Serial.print(nextevent);
              Serial.print(" at index ");
              Serial.println(i);
              EventQueue[i] = nextevent;
              break;
            }
          }
      }
    }
  }

  // set event state
  if(EventCode == 0 || EventCode == ACTIVEVENT){
    if(EventQueue[0] != -1){
      /*
      Serial.println("event queue");
      for(int i=0; i< EVENTQUEUESIZE; i++){
        Serial.println(EventQueue[i]);
      }*/
      EventCode = EventQueue[0];
      Serial.print("switching to event");
      Serial.print(EventCode);
      // shift events forward
      for(int i=0; i < EVENTQUEUESIZE-1; i++){
        EventQueue[i] = EventQueue[i+1];
      }
      // kill last event
      EventQueue[EVENTQUEUESIZE-1]= -1;      
    }
  }

  // eye tracking base case
  if (EventCode >= ACTIVEVENT && EventCode < 1000){

    if (theta_x > neckposlast + deadband){
      neckposlast = theta_x - deadband;
    } else if (theta_x < neckposlast - deadband){
      neckposlast = theta_x + deadband;
    }
    float NeckYawTarget = neckyawcenter + neckposlast;

    ControlCurrValue[LYawChannel] = leftyawcenter + (theta_x - neckposlast);
    ControlCurrValue[RYawChannel] = rightyawcenter + (theta_x - neckposlast);
    ControlCurrValue[LPitchChannel] = leftpitchcenter + theta_y;
    ControlCurrValue[RPitchChannel] = rightpitchcenter + theta_y;

    UpdateRoll();
    UpdateYaw(NeckYawTarget);
    ledcWrite(LYawChannel, angleToDC(ControlCurrValue[LYawChannel]));
    ledcWrite(RYawChannel, angleToDC(ControlCurrValue[RYawChannel]));
    ledcWrite(LPitchChannel, angleToDC(ControlCurrValue[LPitchChannel]));
    ledcWrite(RPitchChannel, angleToDC(ControlCurrValue[RPitchChannel]));
  }

  // blink once fast
  if (EventCode == 11){
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[1], 20, 15);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[1], 20, 15);
      EventCode = 111;
    }
  } else if (EventCode == 111){
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[0], 20, 15);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[0], 20, 15);
      EventCode = 211;
    }
  } else if(EventCode == 211){
    if(QueryControllers()){
      EventCode = ACTIVEVENT;
    }
  }

  // blink twice fast
  if (EventCode == 12){
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[1], 20, 15);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[1], 20, 15);
      EventCode = 112;
    }
  } else if (EventCode == 112){
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[0], 20, 15);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[0], 20, 15);
      EventCode = 212;
    }
  } else if(EventCode == 212){
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[1], 20, 15);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[1], 20, 15);
      EventCode = 312;
    }
  } else if (EventCode == 312){
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[0], 20, 15);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[0], 20, 15);
      EventCode = 412;
    }
  } else if(EventCode == 412){
    if(QueryControllers()){
      EventCode = ACTIVEVENT;
    }
  }

  // blink once slowly
  if (EventCode == 13){
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[1], 20, 5);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[1], 20, 5);
      EventCode = 113;
    }
  } else if (EventCode == 113){
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[0], 20, 5);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[0], 20, 5);
      EventCode = 213;
    }
  } else if(EventCode == 213){
    if(QueryControllers()){
      EventCode = ACTIVEVENT;
    }
  }

  if (EventCode == 14){
    if(QueryControllers()){
      SetControlState(NeckPitchChannel , 50, 60, .3);
      SetControlState(HeadPitchChannel , 110, 60, .3);
      EventCode = 114;
    }
  } else if (EventCode == 114) {
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, (UpperEyeLidPoses[1] + UpperEyeLidPoses[0])/2.0, 20, 3);
      SetControlState(LowerEyelidChannel, (LowerEyeLidPoses[1] + LowerEyeLidPoses[0])/2.0, 20, 3);
      EventCode = 214;
    }
  } else if (EventCode == 214){
    if(QueryControllers()){
      SquintInitTime = millis();
      EventCode = 314;
    }
  } else if (EventCode == 314){
    if((SquintInitTime + SQUINTTIME) < millis()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[0], 20, 3);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[0], 20, 3);
      EventCode = 414;
    }
  } else if (EventCode == 414){
    if(QueryControllers()){
      SetControlState(NeckPitchChannel , neckpitchcenter, 60, .3);
      SetControlState(HeadPitchChannel , headpitchcenter, 60, .3);
      EventCode=  514;
    }
  } else if (EventCode == 514){
    if(QueryControllers()){
      EventCode = ACTIVEVENT;
    }
  }



  // go to sleep end at event 0
  if(EventCode == 2){
    if(QueryControllers()){
      SetControlState(LYawChannel, leftyawcenter, 20, 3);
      SetControlState(RYawChannel, rightyawcenter, 20, 3);
      SetControlState(LPitchChannel, leftpitchcenter, 20, 3);
      SetControlState(RPitchChannel, rightpitchcenter, 20, 3);
      EventCode = 1102;
    }
  } else if(EventCode == 1102){
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[1], 20, 2);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[1], 20, 2);
      SetControlState(HeadRollChannel, headrollcenter, 20, .3);
      SetControlState(NeckYawChannel, neckyawcenter, 20, .3);
      EventCode = 1202;
    }
  } else if(EventCode == 1202){
    if(QueryControllers()){
      SetControlState(NeckPitchChannel , 50, 60, .3);
      SetControlState(HeadPitchChannel , 100, 60, .3);
      EventCode = 1302;
    }
  } else if(EventCode == 1302){
    if(QueryControllers()){
      SetControlState(NeckPitchChannel , 27, 60, .3);
      SetControlState(HeadPitchChannel , 80, 60, .3);
      EventCode = 1402;
    }
  } else if(EventCode == 1402){
    if (QueryControllers()){
      EventCode = 0;
    }
  }

  //wake up, end at active event
  if(EventCode == 3){
    if(QueryControllers()){
      SetControlState(UpperEyelidChannel, UpperEyeLidPoses[0], 20, 15);
      SetControlState(LowerEyelidChannel, LowerEyeLidPoses[0], 20, 15);
      EventCode = 1103;
    }
  } else if(EventCode == 1103){
    if(QueryControllers()){
      SetControlState(NeckPitchChannel , 50, 60, .2);
      SetControlState(HeadPitchChannel , 100, 60, .6);
      EventCode = 1203;
    }
  } else if (EventCode == 1203){
    if(QueryControllers()){
      SetControlState(NeckPitchChannel , neckpitchcenter, 60, .6);
      SetControlState(HeadPitchChannel , headpitchcenter, 60, .6);
      EventCode = 1303;
    }
  } else if(EventCode == 1303){
    if (QueryControllers()){
      EventCode = ACTIVEVENT;
    }
  }

  // calibration mode
  if(EventCode == 1){
    static bool state = false;
    static int channel = -1;
    if (Serial.available()> 0){
      int rlen = Serial.readBytesUntil('\n', buf, BUFFER_SIZE);
      buf[rlen] = '\n';
      //Serial.println(buf);
      if (buf[0] == 'e'){
          EventCode = atoi(buf+1);
      } else{

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
  }
  UpdateControllers();
}

bool QueryControllers(){
  for (int i = 0; i < NUMCONTROLLERS; i++){
    if (ControlActive[i]){
      return false;
    }
  }
  return true;
}

void UpdateYaw(float target){
  if (LastYawTime + 1000.0/YawUpdateFreq < millis() ){
    LastYawTime = millis();
    float diff = target - ControlCurrValue[NeckYawChannel];
    if (diff > 0){
      ControlCurrValue[NeckYawChannel] += YawRateLimit;
    } else{   
      ControlCurrValue[NeckYawChannel] -= YawRateLimit;
    }
    ledcWrite(NeckYawChannel, angleToDC(ControlCurrValue[NeckYawChannel]));
  }
}

void UpdateRoll(){
  if (LastHeadRollTime + 1000.0/RollUpdateFreq < millis() ){
    LastHeadRollTime = millis();
    float diff = ControlCurrValue[HeadRollChannel] - (headrollcenter + theta_r/2.0);
    if (diff > 0){
      ControlCurrValue[HeadRollChannel] -= HeadRateLimit;
    } else{   
      ControlCurrValue[HeadRollChannel] += HeadRateLimit;
    }
    ControlCurrValue[HeadRollChannel] = constrain(ControlCurrValue[HeadRollChannel], headrollrange[0], headrollrange[1]);
    ledcWrite(HeadRollChannel, angleToDC(ControlCurrValue[HeadRollChannel]));
  }
}

void UpdateControllers(){
  for(int i =0; i<NUMCONTROLLERS; i++){
    UpdateController(i);
  }
}

void SetControlState(int idx, float Target, float Freq, float StepSize){
  ControlActive[idx] = true;
  ControlTarget[idx] = Target;
  ControlFreq[idx] = Freq;
  ControlStepSize[idx] = StepSize;
}

void UpdateController(int idx){
  if (ControlActive[idx]){
    if (ControlLastTime[idx] + 1000.0/ControlFreq[idx] < millis() ){
      ControlLastTime[idx] = millis();
      float diff = ControlTarget[idx] - ControlCurrValue[idx];
      if (diff > 0){
            ControlCurrValue[idx] += ControlStepSize[idx];
            if (ControlCurrValue[idx] >= ControlTarget[idx]){
              ControlCurrValue[idx] = ControlTarget[idx];
              ControlActive[idx] = false;
            }
          } else{
            ControlCurrValue[idx]-= ControlStepSize[idx];
            if (ControlCurrValue[idx] <= ControlTarget[idx]){
              ControlCurrValue[idx] = ControlTarget[idx];
              ControlActive[idx] = false;
            }
          }
          /*
      Serial.print("commanding channel ");
      Serial.print(idx);
      Serial.print(" to value ");
      Serial.println(ControlCurrValue[idx]);
      */
      ledcWrite(idx, angleToDC(ControlCurrValue[idx]));
    }
  }
}