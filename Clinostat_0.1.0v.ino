#include "HardwareTimer.h"
#include "TM1637.h"
TM1637 tm1637(PB8,PB9);

#include "GyverButton.h"
GButton butt_ON(PA0);        //ON_OFF button

#include "GyverEncoder.h"
Encoder enc1(PA1, PA2, PA5); //Encoder

  int8_t ListDisp[4];             //Display digit buffer
  int Disp_speed = 0;             //Screen Display Buffer
  volatile int speed_set = 60;    //Set speed
  int speed_temp = 0;             //Time speed (target)
  volatile int speed_PWM = 0;     //Speed to PWM converter
  volatile int speed_tone = 0;    //Speed to stepper tone converter
  volatile int tone_temp = 0;     //Intermediate stepper speed (acceleration)
  volatile int speed_in = 0;      //Internal disk speed
  volatile int speed_out = 0;     //External disk speed
  volatile unsigned long time_temp = 0;
  volatile unsigned long time_prev = 0;
  volatile unsigned long time_enc_click = 0;//Time to enter the speed reference mode
  volatile unsigned long time_fan_in = 0;   //Previous turnover time of the internal speed sensor
  volatile unsigned long time_fan_out = 0;  //Previous turnover time of the external speed sensor
  volatile int deb_time_in = 50;            //Anti-bounce time of the Internal speed sensor
  volatile int deb_time_out = 50;           //Anti-bounce time of the external speed sensor
  volatile boolean run_rotation = false;    //Enable or disable rotation
  boolean enc_click = false;                //Speed setting mode button at the Encoder
  volatile boolean run_in = false;          //Allow the internal disk to run
  volatile boolean off_rotation = false;    //Rotation stop
  
HardwareTimer *Timer3 = new HardwareTimer(TIM3);
HardwareTimer *PWM_STP = new HardwareTimer(TIM1);

void setup() {
  Serial.begin(9600);
  pinMode(PB4, INPUT_PULLUP);     // Pull-up pin 4 to interrupt external hall sensor
  pinMode(PA0, INPUT_PULLUP);     // Pull-up pin 0 for interrupt ON_OFF button
  
  analogWriteFrequency(10000);    // Frequency at PWM (10 KHz)
  analogWriteResolution(13);      // PWM bit rate
  
  pinMode(PA15, OUTPUT);          //EN stepper motor (0 - on, 5V - off)
  digitalWrite(PA15, HIGH);
  pinMode(PA10, OUTPUT);          //Stepper motor SLP(sleep) (0 - sleep, 5V - operation)
  digitalWrite(PA10, HIGH);
  pinMode(PA8, OUTPUT);           //Stepper motor DIR (0 - clockwise, 5V - counterclockwise)
  digitalWrite(PA8, HIGH);

  
  enc1.setType(TYPE2);                    //Encoder
  enc1.setFastTimeout(80);
  attachInterrupt(PA1, isrCLK, CHANGE);   // Interrupt on pin 1 CLK at the Encoder
  attachInterrupt(PA2, isrDT, CHANGE);    // Interrupt on pin 2 DT at the Encoder
  attachInterrupt(PA5, isrSW, CHANGE);    // Interrupt on pin 5 Button at the Encoder
  attachInterrupt(PA0, ON_OFF, CHANGE);   // Interrupt on pin 0 Button at the ON_OFF
  attachInterrupt(PB4, fan1, FALLING);    // Interrupt on pin 4 hall sensor internal
  attachInterrupt(PB3, fan2, RISING);     // Interrupt on pin 3 hall sensor external
  butt_ON.setDebounce(80);                // Anti-bounce setting for ON_OFF button (default 80 ms)
  
  tm1637.init();
  tm1637.set(BRIGHTEST);                  // BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  
  Timer3->pause();                        // stop the timer before setting
  Timer3->setOverflow(10000, MICROSEC_FORMAT); // Timer start repeat in microseconds (10 ms)
  Timer3->attachInterrupt(func_tim_3);    // Enable interrupt
  Timer3->refresh();                      // Reset the timer 
  Timer3->resume();                       // Start timer
  PWM_STP->setPWM(2, PA9, 100, 0);        // Stepper motor STEP
}

void isrCLK(){    // Processing in the interrupt left turn of the Encoder
  enc1.tick();
}
void isrDT(){     // Processing in the right turn interrupt at the Encoder
  enc1.tick();
}
void isrSW(){     // Processing in the interrupt of pressing the Button on the Encoder
  enc1.tick();
}
void ON_OFF(){    // Processing in the interrupt of pressing the Button ON_OFF
  butt_ON.tick();
}
//////////////////////////////////////////////////////////////////////
void func_tim_3() { // Internal interrupt handler once every (10 ms)
  if ((millis()-time_fan_in) >= deb_time_in) {
    attachInterrupt(PB4, fan1, FALLING); //Enable interrupt on pin 4 after the calculated time (anti-bounce)
  }
  if ((millis()-time_fan_out) >= deb_time_out) {
    attachInterrupt(PB3, fan2, RISING);  //Enable interrupt on pin 3 after the calculated time (anti-bounce)
  }
  if (run_rotation){
    if ((speed_tone - 48) > tone_temp){
      tone_temp += 4;
      PWM_STP->setPWM(2, PA9, tone_temp, 50); 
      off_rotation = 0;
    }
    if ((speed_tone + 48) < tone_temp){
      tone_temp -= 4;
      PWM_STP->setPWM(2, PA9, tone_temp, 50); 
      off_rotation = 0;
    }
  }
  else{
    if (960 < tone_temp){
      tone_temp -= 32;
      PWM_STP->setPWM(2, PA9, tone_temp, 50);
      analogWrite(PB7,2000);
    }
    else{
      off_rotation = 1;
    }
  }
}
//////////////////////////////////////////////////////////////////////
void fan1() {       // Processing of internal hall sensor, external interrupt
  if (off_rotation){                //Turn to zero position and stop
    speed_PWM = 0;
    analogWrite(PB7,speed_PWM);
    speed_in = 0;
  }
  else{
    int qwe = millis()-time_fan_in;  //Calculate turnaround time
    if (qwe < deb_time_in) qwe = 60000/speed_in;  //If the entry time is shorter than the estimated time, use the previous
    speed_in = 60000/qwe;
    time_fan_in = millis();         //Time stamp
    detachInterrupt(PB4);           //Disable interrupt on pin 4
    
    Serial.print("deb_time_in- ");
    Serial.print(deb_time_in);
    Serial.print(" / ");
    Serial.print("speed_PWM- ");
    Serial.print(speed_PWM);
    Serial.print(" / ");
    Serial.print("time- ");
    Serial.println(qwe);
      
    if ((qwe - 3) > (60000/speed_set)){            //Add PWM at low speed
      speed_PWM += (qwe - 60000/speed_set)/2 + 1;
      if(speed_PWM > 8191) speed_PWM = 8191;
      analogWrite(PB7,speed_PWM);
      Serial.print("+= ");
      Serial.println((qwe - 60000/speed_set)/2 + 1);
    }
    if ((qwe + 3) < (60000/speed_set)){            //Turn down PWM at high speed
      speed_PWM -= (60000/speed_set - qwe)/2 + 1;
      if (speed_PWM < 0) speed_PWM = 0;
      analogWrite(PB7,speed_PWM);
      Serial.print("-= ");
      Serial.println((60000/speed_set - qwe)/2 + 1);
    }
  }
}
//////////////////////////////////////////////////////////////////////
void fan2() {       // Processing of external hall sensor, external interrupt
  if (off_rotation){                 //Turn to zero position and stop
    digitalWrite(PA15, HIGH);        //EN 5V stepper motor - off
    PWM_STP->setPWM(2, PA9, 100, 0); //Disable step frequency on stepper motor
    speed_tone = 0;
    speed_out = 0;
  }
  else{
    int qwe = millis()-time_fan_out;  //Calculate turnaround time
    if (qwe < deb_time_out) qwe = 60000/speed_out;  //If the entry time is shorter than the estimated time, use the previous
    speed_out = 60000/qwe;
    time_fan_out = millis();         //Time stamp
    detachInterrupt(PB3);            //Disable interrupt on pin 3
    
    Serial.print("deb_time_out- ");
    Serial.print(deb_time_out);
    Serial.print(" / ");
    Serial.print("speed_tone- ");
    Serial.print(tone_temp);
    Serial.print(" / ");
    Serial.print("time- ");
    Serial.println(qwe);
      
    if ((qwe - 1) > (60000/speed_set)){            //Add PWM at low speed
      tone_temp += 4;
      PWM_STP->setPWM(2, PA9, tone_temp, 50);
      Serial.print("+= ");
      Serial.println("4");
    }
    if ((qwe + 1) < (60000/speed_set)){            //Turn down PWM at high speed
      tone_temp -= 4;
      PWM_STP->setPWM(2, PA9, tone_temp, 50);
      Serial.print("-= ");
      Serial.println("4");
    }
    if ((qwe) == (60000/speed_set)&& !run_in){     //When speed is reached, start the internal disk
      run_in = 1;
      analogWrite(PB7,speed_PWM);
      time_fan_in = millis() - 60000/speed_set;
    }
  }
}
//////////////////////////////////////////////////////////////////////
void loop() {
  butt_ON.tick();
  enc1.tick();

  if (enc1.isRight()) speed_temp --;     //Encoder increment to the right
  if (enc1.isLeft())  speed_temp ++;     //Encoder increment to the left
  if (enc1.isFastR()) speed_temp -= 3;   //Encoder acceleration increment to the right
  if (enc1.isFastL()) speed_temp += 3;   //Encoder acceleration increment to the left
  if (speed_temp > 200) speed_temp = 200;//Speed limit of 200 rpm
  if (speed_temp < 0) speed_temp = 0;    //Speed limit of 0 rpm

  if (enc1.isSingle()){                  //Enable or disable speed change, Button at the Encoder
    if (enc_click){
      speed_set = speed_temp;
      speed_PWM = map(speed_set, 0, 80, 500, 8191); //!!!corrected 200 to 80 (this 12 volt motor only gives 70 rpm).
      speed_tone = speed_set*96;
      deb_time_in = 60000/speed_set - 60000/speed_set/10; //Converting the internal hall sensor anti-bounce time
      deb_time_out= 60000/speed_set - 60000/speed_set/10; //Converting the external hall sensor anti-bounce time
    }
    else
    {
      speed_temp = speed_set;
    }
    enc_click = !enc_click;
  }

  if (butt_ON.isClick()){
    run_rotation = !run_rotation; //Enable or disable rotation
    if (run_rotation){
      digitalWrite(PA15, LOW);    //Stepper motor EN 0 - on
      time_fan_out= millis() - 60000/speed_set;
      speed_PWM = map(speed_set, 0, 80, 500, 8191); //!!!corrected 200 to 80 (this 12 volt motor only gives 70 rpm).
      speed_tone = speed_set*96;
      tone_temp = 0;
      deb_time_in = 60000/speed_set - 60000/speed_set/10; //Converting the internal hall sensor anti-bounce time
      deb_time_out= 60000/speed_set - 60000/speed_set/10; //Converting the external hall sensor anti-bounce time
    }
    else{
      run_in = 0;
    }
  }
  
  if (enc_click){                 //Rotation speed selection menu
    Disp_speed = speed_temp;
    ListDisp[3] = Disp_speed%10;
    Disp_speed = Disp_speed / 10;
    ListDisp[2] = Disp_speed%10;
    Disp_speed = Disp_speed / 10;
    ListDisp[1] = Disp_speed%10;
    Disp_speed = Disp_speed / 10;
    ListDisp[0] = Disp_speed%10;
    tm1637.display(ListDisp);
  }
  else{
    if (!run_in){                 //Rotation speed display
      Disp_speed = speed_out;
    }
    else{
      Disp_speed = speed_in;
    }
    ListDisp[3] = Disp_speed%10;
    Disp_speed = Disp_speed / 10;
    ListDisp[2] = Disp_speed%10;
    Disp_speed = Disp_speed / 10;
    ListDisp[1] = Disp_speed%10;
    Disp_speed = Disp_speed / 10;
    ListDisp[0] = Disp_speed%10;
    tm1637.display(0, 0x7f);
    tm1637.display(1, ListDisp[1]);
    tm1637.display(2, ListDisp[2]);
    tm1637.display(3, ListDisp[3]);
  }
}
