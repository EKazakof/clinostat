#include "HardwareTimer.h"
#include "TM1637.h"
TM1637 tm1637(PB8,PB9);

#include "GyverButton.h"
GButton butt_ON(PA0);        //Кнопка ON_OFF

#include "GyverEncoder.h"
Encoder enc1(PA1, PA2, PA5); //Энкодер

  int8_t ListDisp[4];             //хранитель цифр дисплея
  int Disp_speed = 0;             //отображение скорости
  volatile int speed_set = 60;    //Заданная скорость
  int speed_temp = 0;             //Временная скорость (целевая)
  volatile int speed_PWM = 0;     //переход скорость в PWM
  volatile int speed_tone = 0;    //переход скорость в tone шаговика
  volatile int tone_temp = 0;     //Промежуточная скорость шаговика (ускорение)
  volatile int speed_in = 0;      //Скорость внутреннего диска
  volatile int speed_out = 0;     //Скорость внешнего диска
  volatile unsigned long time_temp = 0;
  volatile unsigned long time_prev = 0;
  volatile unsigned long time_enc_click = 0;//Время входа в режим задания скорости
  volatile unsigned long time_fan_in = 0;   //Предыдущая временная отсечка внутреннего холла
  volatile unsigned long time_fan_out = 0;  //Предыдущая временная отсечка внешнего холла
  volatile int deb_time_in = 50;            //Время антидребезга внутреннего холла
  volatile int deb_time_out = 50;           //Время антидребезга внешнего холла
  volatile boolean run_rotation = false;    //Разрешить или запретить вращение
  boolean enc_click = false;                //Режим задания скорости кнопка у Энкодера
  volatile boolean run_in = false;          //разрешить один запуск внутреннего диска
  volatile boolean off_rotation = false;    //остановка вращения
  
HardwareTimer *Timer3 = new HardwareTimer(TIM3);
HardwareTimer *PWM_STP = new HardwareTimer(TIM1);

void setup() {
  Serial.begin(9600);
  pinMode(PB4, INPUT_PULLUP);     // Подтяжка пина 4 для прерывания внешнего холла
  pinMode(PA0, INPUT_PULLUP);     // Подтяжка пина 0 для прерывания кнопка ON_OFF
  
  analogWriteFrequency(10000);    // Частота у PWM (10 KHz)
  analogWriteResolution(13);      // Битность PWM
  
  pinMode(PA15, OUTPUT);          //Шаговый двигатель EN (0 — включен, 5В — выключен)
  digitalWrite(PA15, HIGH);
  pinMode(PA10, OUTPUT);          //Шаговый двигатель SLP(сон) (0 — сон, 5В — работа)
  digitalWrite(PA10, HIGH);
  pinMode(PA8, OUTPUT);           //Шаговый двигатель DIR (0 — по часовой, 5В — против часовой)
  digitalWrite(PA8, HIGH);

  
  enc1.setType(TYPE2);                    //Энкодер
  enc1.setFastTimeout(80);
  attachInterrupt(PA1, isrCLK, CHANGE);   // прерывание на 1 пине! CLK у Энкодера
  attachInterrupt(PA2, isrDT, CHANGE);    // прерывание на 2 пине! DT у Энкодера
  attachInterrupt(PA5, isrSW, CHANGE);    // прерывание на 5 пине! Кнопка у Энкодера
  attachInterrupt(PA0, ON_OFF, CHANGE);   // прерывание на 0 пине! кнопка ON_OFF
  attachInterrupt(PB4, fan1, FALLING);    // прерывание на 4 пине! датчик холла внутренний
  attachInterrupt(PB3, fan2, RISING);     // прерывание на 3 пине! датчик холла внешний
  butt_ON.setDebounce(80);                // настройка антидребезга для кнопки ON_OFF (по умолчанию 80 мс)
  
  tm1637.init();
  tm1637.set(BRIGHTEST);                  // BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  
  Timer3->pause();                        // останавливаем таймер перед настройкой
  Timer3->setOverflow(10000, MICROSEC_FORMAT); // Повтор запуска таймера в микросекундах (10 мс)
  Timer3->attachInterrupt(func_tim_3);    // активируем прерывание
  Timer3->refresh();                      // обнулить таймер 
  Timer3->resume();                       // запускаем таймер
  PWM_STP->setPWM(2, PA9, 100, 0);        // Шаговый двигатель step
}

void isrCLK(){    // отработка в прерывании поворот на лево у Энкодера
  enc1.tick();
}
void isrDT(){     // отработка в прерывании поворот на право у Энкодера
  enc1.tick();
}
void isrSW(){     // отработка в прерывании Кнопка у Энкодера
  enc1.tick();
}
void ON_OFF(){    // отработка в прерывании Кнопка ON_OFF
  butt_ON.tick();
}
//////////////////////////////////////////////////////////////////////
void func_tim_3() { // обработчик внутреннего прерывания раз в (10 мс)
  if ((millis()-time_fan_in) >= deb_time_in) {
    attachInterrupt(PB4, fan1, FALLING); //разрешить прерывание на пине 4 после расчетного времени
  }
  if ((millis()-time_fan_out) >= deb_time_out) {
    attachInterrupt(PB3, fan2, RISING);  //разрешить прерывание на пине 3 после расчетного времени
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
void fan1() {       // отработка в прерывании внутреннего холла
  if (off_rotation){                //докрутить оборот и выключить
    speed_PWM = 0;
    analogWrite(PB7,speed_PWM);
    speed_in = 0;
  }
  else{
    int qwe = millis()-time_fan_in;  //вычислить время оборота
    if (qwe < deb_time_in) qwe = 60000/speed_in;  //Если время входа меньше расчетного использовать предыдущее
    speed_in = 60000/qwe;
    time_fan_in = millis();         //запомнить время
    detachInterrupt(PB4);           //запрещаю прерывание на 4 пине
    
    Serial.print("deb_time_in- ");
    Serial.print(deb_time_in);
    Serial.print(" / ");
    Serial.print("speed_PWM- ");
    Serial.print(speed_PWM);
    Serial.print(" / ");
    Serial.print("time- ");
    Serial.println(qwe);
      
    if ((qwe - 3) > (60000/speed_set)){            //Добавить PWM при низкой скорости
      speed_PWM += (qwe - 60000/speed_set)/2 + 1;
      if(speed_PWM > 8191) speed_PWM = 8191;
      analogWrite(PB7,speed_PWM);
      Serial.print("+= ");
      Serial.println((qwe - 60000/speed_set)/2 + 1);
    }
    if ((qwe + 3) < (60000/speed_set)){            //Убавить PWM при высокой скорости
      speed_PWM -= (60000/speed_set - qwe)/2 + 1;
      if (speed_PWM < 0) speed_PWM = 0;
      analogWrite(PB7,speed_PWM);
      Serial.print("-= ");
      Serial.println((60000/speed_set - qwe)/2 + 1);
    }
  }
}
//////////////////////////////////////////////////////////////////////
void fan2() {       // отработка в прерывании внешнего холла
  if (off_rotation){                 //докрутить оборот и выключить
    digitalWrite(PA15, HIGH);        //Шаговый двигатель EN 5В — выключен
    PWM_STP->setPWM(2, PA9, 100, 0); //Отключить частоту step на шаговом двигателе
    speed_tone = 0;
    speed_out = 0;
  }
  else{
    int qwe = millis()-time_fan_out;  //вычислить время оборота
    if (qwe < deb_time_out) qwe = 60000/speed_out;  //Если время входа меньше расчетного использовать предыдущее
    speed_out = 60000/qwe;
    time_fan_out = millis();         //запомнить время
    detachInterrupt(PB3);            //запрещаю прерывание на 3 пине
    
    Serial.print("deb_time_out- ");
    Serial.print(deb_time_out);
    Serial.print(" / ");
    Serial.print("speed_tone- ");
    Serial.print(tone_temp);
    Serial.print(" / ");
    Serial.print("time- ");
    Serial.println(qwe);
      
    if ((qwe - 1) > (60000/speed_set)){            //Добавить PWM при низкой скорости
      tone_temp += 4;
      PWM_STP->setPWM(2, PA9, tone_temp, 50);
      Serial.print("+= ");
      Serial.println("4");
    }
    if ((qwe + 1) < (60000/speed_set)){            //Убавить PWM при высокой скорости
      tone_temp -= 4;
      PWM_STP->setPWM(2, PA9, tone_temp, 50);
      Serial.print("-= ");
      Serial.println("4");
    }
    if ((qwe) == (60000/speed_set)&& !run_in){     //при достижении скорости, запустить внутренний диск
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

  if (enc1.isRight()) speed_temp --;     //Инкремент Энкодера на право
  if (enc1.isLeft())  speed_temp ++;     //Инкремент Энкодера на лево
  if (enc1.isFastR()) speed_temp -= 3;   //Инкремент ускорения Энкодера на право
  if (enc1.isFastL()) speed_temp += 3;   //Инкремент ускорения Энкодера на лево
  if (speed_temp > 200) speed_temp = 200;//Ограничение скорость в 200 об/мин
  if (speed_temp < 0) speed_temp = 0;    //Ограничение скорость в 0 об/мин

  if (enc1.isSingle()){                  //Разрешить или запретить изменение скорости, Кнопка у Энкодера
    if (enc_click){
      speed_set = speed_temp;
      speed_PWM = map(speed_set, 0, 80, 500, 8191); //!!!поправил 200 на 80 (данный мотор на 12 вольт дает только 70 об\мин)
      speed_tone = speed_set*96;
      deb_time_in = 60000/speed_set - 60000/speed_set/10; //Конвертировать  время антидребезга внутреннего холла
      deb_time_out= 60000/speed_set - 60000/speed_set/10; //Конвертировать  время антидребезга внешнего холла
    }
    else
    {
      speed_temp = speed_set;
    }
    enc_click = !enc_click;
  }

  if (butt_ON.isClick()){
    run_rotation = !run_rotation; //Разрешить или запретить вращение
    if (run_rotation){
      digitalWrite(PA15, LOW);    //Шаговый двигатель EN 0 — включен
      time_fan_out= millis() - 60000/speed_set;
      speed_PWM = map(speed_set, 0, 80, 500, 8191); //!!!поправил 200 на 80 (данный мотор на 12 вольт дает только 70 об\мин)
      speed_tone = speed_set*96;
      tone_temp = 0;
      deb_time_in = 60000/speed_set - 60000/speed_set/10; //Конвертировать  время антидребезга внутреннего холла
      deb_time_out= 60000/speed_set - 60000/speed_set/10; //Конвертировать  время антидребезга внешнего холла
    }
    else{
      run_in = 0;
    }
  }
  
  if (enc_click){                 //отобразить дисплей выбора скорости
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
    if (!run_in){                 //отобразить дисплей реальной скорости
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
