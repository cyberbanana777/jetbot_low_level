#include <Arduino.h>
#include <math.h>

//библиотеки регуляции и управления
#include <Wire.h>                     
#include <stdio.h>

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool input_open = false;

//Константы физических параметров
const float Pi = 3.14159;
const float l = 0.117;
const float r = 0.065/2;

#define LED_PIN 13 // Оставлен для debug


// Для таймеров
unsigned long tmr = 0;
unsigned long delta = 0;
int dt = 30;
const unsigned long timer_timeout = dt * 1000; //мкс

// Для остановки после 5 секунд простоя
const unsigned int im_timer_timeout = 5000;
unsigned int im_timer = 0;


// Переменные скорости правые
float R = 1/3.5;
float L = 1/3.5; // Подгоночный коэффициент для перевода из об/с в нормированные единицы

float _obrat = 3.5; 

// target
double target_speed_left = 0.0;
double target_speed_right = 0.0;
double real_speed_left = 0.0;
double real_speed_right = 0.0;

// feedback
double x_pos_ = 0.0;
double y_pos_ = 0.1; //= 0.0;
double heading_ = 0.2; //= 0.0;
double real_x_linear_velocity_ = 0.3;//= 0.0;
double real_z_angular_velocity_ = 0.4;//= 0.0;
double wheel_position_l_ = 0.5;//= 0.0;
double wheel_position_r_ = 0.6;//= 0.0;
double wheel_load_l_ = 0.7;//= 0.0;
double wheel_load_r_ = 0.8;//= 0.0;
double wheel_temperature_l_ = 0.9;//= 0.0;
double wheel_temperarure_r_ = 0.10;//= 0.0;
double wheel_voltage_l_ = 0.11;//= 0.0;
double wheel_voltage_r_ = 0.12;//= 0.0;


String input_m[2] = {"", ""};
String feedback_msg_str = "";


void feedbackPublish () {
  feedback_msg_str = 
    /*"*;" + */
    String(x_pos_, 5) + ';' +
    String(y_pos_, 5) + ';' +
    String(heading_, 5) + ';' +

    String(real_x_linear_velocity_, 5) + ';' +
    String(real_z_angular_velocity_, 5) + ';' +

    String(wheel_position_l_, 5) + ';' +
    String(wheel_position_r_, 5) + ';' +

    String(wheel_load_l_, 5) + ';' +
    String(wheel_load_r_, 5) + ';' +

    String(wheel_temperature_l_, 5) + ';' +
    String(wheel_temperarure_r_, 5) + ';' +

    String(wheel_voltage_l_, 5) + ';' +
    String(wheel_voltage_r_, 5) + ';' 
    /* + ";#"*/;
  Serial.println(feedback_msg_str);
}

//int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);
float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void speed_converter(double xl, double zw) {
  target_speed_left = xl - zw * l / 2;
  target_speed_right = xl + zw * l / 2;
  target_speed_left = mapFloat(target_speed_left, -1.2, 1.2, -0.8, 0.8 );
  target_speed_right = mapFloat(target_speed_right, -1.2, 1.2, -0.8, 0.8 );
}

//Ответ на входное сообщение
void serialEvent() {
  im_timer = millis();
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if(inChar == '$' or input_open == true){
      // add it to the inputString:
      inputString += inChar;
      input_open = true;
    }
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '#') {
      stringComplete = true;
      input_open = false;
    }
  }
}


void setup() {

  // Светодиод ошибки
  pinMode(LED_PIN, INPUT); // Оставлен для debug
  
  Serial.begin(250000);
  inputString.reserve(150);
  
  delay(100);
  
}


void loop() {

  if (millis() - im_timer > im_timer_timeout){
    target_speed_left = 0;
    target_speed_right = 0;
    /*
    ------------------------------
    Отправка сигнала остановки на моторы
    ------------------------------
    */

  }

  delta = micros() - tmr;
  
  if (delta >= timer_timeout){  
    tmr = micros();

    //Подсчёт скорости
    double vel_dt = timer_timeout/1000;
    double linear_vel_x = (real_speed_right + real_speed_left)*_obrat*2*Pi*r/2;
    double angular_vel_z = (real_speed_right - real_speed_left)*_obrat*2*Pi*r/l;
    double delta_heading = angular_vel_z * vel_dt/1000; //radians
    double cos_h = cos(heading_);
    double sin_h = sin(heading_);
    double delta_x = (linear_vel_x * cos_h) * vel_dt/1000; //m
    double delta_y = (linear_vel_x * sin_h) * vel_dt/1000; //m

    /*
    ------------------------------
    Получение оставших значений
    ------------------------------
    */

    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    /*
    ------------------------------------------
    Обновление глобальных переменных состояний
    ------------------------------------------
    */

    feedbackPublish();

  }

    //Обработка входного значения
  if (stringComplete) {
    unsigned int i = 0;
    for (i = 1; i < inputString.length()-1; i++){
      if (inputString[i] == ';') break;
      input_m[0] += inputString[i];
    }

    for (i = i+1; i < inputString.length()-1; i++){
      if (inputString[i] == ';') break;
      input_m[1] += inputString[i];
    }

    char str1[input_m[0].length()];
    char str2[input_m[1].length()]; 
    for(i=0; i < input_m[0].length(); i++) str1[i] = input_m[0][i];
    for(i=0; i < input_m[1].length(); i++) str2[i] = input_m[1][i];
    input_m[0] = "";
    input_m[1] = "";

    speed_converter(atof(str1), atof(str2));
    real_speed_left = target_speed_left;
    real_speed_right = target_speed_right;
    /*
    ------------------------------------------------
    Отправка сигнала с целевыми скоростями на моторы
    ------------------------------------------------
    */

    inputString = "";
    stringComplete = false;
  }
}