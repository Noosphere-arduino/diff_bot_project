// IMU
// отключаем магнитометр
#define BMX055_DISABLE_BMM
#include <iarduino_Position_BMX055.h>
iarduino_Position_BMX055 sensor(BMX);

// моторы
#define M1_ENA 6
#define M2_ENB 5
#define M1_IN1 7
#define M1_IN2 8
#define M2_IN3 3
#define M2_IN4 4

// ультразвук
#define TRIG_PIN 9
#define ECHO_PIN 10
float dist_3[] = {0.0, 0.0, 0.0}; // массив для хранения последних трех измерений
byte i;
float k;
float dist_prev;
int min_dist;                     // расстояние до препятствия, при котором машинка останавливается

// параметры регулятора
float kp;
float ki;
float kd;

int dt;

unsigned long prevTime = 4000; // задаем паузу перед началом движения, чтобы колебание измерений IMU после старта
int course;                    // курс в градусах, по которому будем двигаться
int target_speed;              // скорость движения
int minOut;                    // минимальная скорость
int maxOut;                    // максимальная скорость

void setup() {
  
  Serial.begin(115200);

  // IMU
  sensor.begin();
  sensor.setFastOffset();

  // ультразвук
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  k = ;
  dist_prev = ;

  // моторы
  setup_motors_pin();
  stop_motors();

  // задаем курс в градусах, по которому будем двигаться
  course = ;

}

void loop() {

  unsigned long sensTime = millis();

  // цикл управления
  if (sensTime - prevTime > dt) {

    //считываем курс, измеренный IMU
    sensor.read();
    float heading = sensor.axisZ;
    //Serial.print("IMU Heading = ");Serial.println(heading);    

    // считываем данные с датчиков препятствий
    float front_dist = read_sonar();
    //Serial.print("Sonic dist = ");Serial.println(front_dist);    

    // есть ли препятствие?
    if (front_dist < min_dist) {
      
      //Serial.print("Obstacle at ");Serial.println(front_dist);
      
      // TODO: Задание 1 - остановка машинки
      // остановка

      
      // TODO: Задание 3 - разворот на новый курс
      // разворот (новый курс)
      
    }
    else {
      // езда по прямой
      course_control(heading, course);
    }

    // обновляем переменную времени
    prevTime = sensTime;
  }

}

void setup_motors_pin()
{
  pinMode(M1_ENA, OUTPUT);
  pinMode(M2_ENB, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);
}

// функция остановки моторов (пример из урока NRF)
void stop_motors()
{
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, LOW);
  digitalWrite(M2_IN4, LOW);
}

// функция ПИД регулятора
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {

  // TODO: написать код П или ПИ или ПИД регулятора
  // https://alexgyver.ru/gyverpid/
   
  return output;
}


// функция управления моторами (пример из урока NRF)
void control_motors(int pwm_l, int pwm_r) {
  uint8_t dir_l = 0;
  uint8_t dir_r = 0;
  if (pwm_l < 0) dir_l = 1;
  if (pwm_r < 0) dir_r = 1;

  digitalWrite(M1_IN1, !dir_l);
  digitalWrite(M1_IN2, dir_l);

  digitalWrite(M2_IN3, !dir_r);
  digitalWrite(M2_IN4, dir_r);

  analogWrite(M1_ENA, abs(pwm_l));
  analogWrite(M2_ENB, abs(pwm_r));
}

// функция езды по прямой
void course_control(float heading, float course) {

  
  //int dir = 0;   // направление подруливания: 1 = вправо, -1 = влево, 0 = ??? 
  //int regulator; // результат выполнения функции computePID()

  // TODO: Задание 1 - написать код езды по линии для курса = 0
  
  // regulator = ;
  // dir = ;
  

  // TODO: Задание 2 - написать код езды по линии для разных значений курса
  
  // тест 1: course = 180.0, heading < -90 
  //if (course - heading > ) { 
    // для корректной работы регулятора heading должен быть больше курса на величину ...
    //regulator = ;
    //dir = ;
  //}
  
  // тест 2: course = -180.0, heading > 90
  // else if (heading - course > 270){
    // для корректной работы регулятора heading должен быть меньше курса на величину abs(course) - heading
    //regulator = ;
    //dir = ;
  //}
  
  //else {
    //regulator = ;
    //dir = ;
  //}
    
  //if (dir == -1) { 
    // подруливаем влево 
  //}
  
  //else if (dir == 1) { 
    // подруливаем вправо
  //}
  
  // отладка:
  //Serial.print(constrain(target_speed - abs(regulator), minOut, maxOut)); Serial.print(", ");
  //Serial.println(constrain(target_speed + abs(regulator), minOut, maxOut));// Serial.print(", ");
}

// функция разворота
void turnaround(float heading, float course) {
  
  // TODO: Задание 3 - написать функцию разворота

}

// медианный фильтр
float median_filter(float a, float b, float c) {
  float middle = 0.0;
  if ((a <= b) && (a <= c)) {
    //middle = (b <= c) ? b : c;
    if (b <= c) middle = b;
    else middle = c;
  }
  else {
    if (( b <= a) && (b <= c)) {
      middle = (a <= c ) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}

// чтение данных с сонара
float read_sonar() {

  int duration;
  float distance;
  float dist_median;
  float dist_filtered;

  // счетчик от 0 до 2
  if (i > 1) {
    i = 0;
  }
  else i++;

  // выставляем значение LOW на пине TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // измерим длительность высокого сигнала на пине ECHO_PIN
  duration = pulseIn(ECHO_PIN, HIGH);

  distance = duration / 58;

  if (distance > 400) distance = 400;
  
  dist_3[i] = distance;

  dist_median = median_filter(dist_3[0], dist_3[1], dist_3[2]);
  dist_filtered = dist_median * k + dist_prev * (1 - k);

  dist_prev = dist_filtered;

  return dist_filtered;
}
