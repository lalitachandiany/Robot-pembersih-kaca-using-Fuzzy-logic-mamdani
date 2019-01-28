#include <Arduino.h>
  
//sensor debu
const int pinGP2Y = A0;
int pinLED = 40;
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
int dustDensity;

//motor dc
const int IN1 = 7;
const int IN2 = 6;
const int ENA = 9;

//sensor ultrasonik
const int trigPin = 10;
const int echoPin = 11;
const int buzzer = 12;
const int ledPin = 13;

// defines variables of ultrasonic sensor
long duration;
int distance;
int safetyDistance;

//servo
#include <Servo.h>
Servo myservo; // membuat objek servo di library Servo.h
int pos = 0; // geser servopada posisi 0 derajat

//photodiode module
int photodiode = 42;
int ncahaya;

// fuzzy
#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyIO.h>
#include <FuzzyInput.h>
#include <FuzzyOutput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzySet.h>

#define FUZZY_IN_DEBU 1
#define FUZZY_IN_CAHAYA 2
#define FUZZY_OUT_PWM 3

float DebuX;
float CahayaX;

#define APP_PORT_DEBUG Serial
// fuzzy object
Fuzzy *fuzzy_main_obj = new Fuzzy();

// input debu
FuzzySet *debu_tipis = new FuzzySet(25, 25, 122.5, 220);
FuzzySet *debu_sedang = new FuzzySet(122.5, 220, 220, 330);  
FuzzySet *debu_tebal = new FuzzySet(220, 330, 330, 440.18);

// input cahaya
FuzzySet *cahaya_gelap = new FuzzySet(37, 37, 143.5, 250);
FuzzySet *cahaya_agak_terang = new FuzzySet(143.5, 250, 250, 281);
FuzzySet *cahaya_terang = new FuzzySet(250, 281, 281, 312);

// output pwm
FuzzySet *pwm_lambat = new FuzzySet(100, 100, 125, 150);
FuzzySet *pwm_sedang = new FuzzySet(125, 150, 150, 202.5);
FuzzySet *pwm_cepat = new FuzzySet(150, 202.5, 202.5, 255);

float PwmX = 0.0;

/**
 * create new fuzzy rule with and boolean
 * @method createNewFuzzyRule
 * @param  ruleId             id rule
 * @param  in1                fuzzy set 1
 * @param  in2                fuzzy set 2
 * @param  out1               fuzzt set output
 * @return                    new fuzzy rule
 */

 void APP_DEBUG_PRINT(String alog) {
  char dtx[16] = {0};
  snprintf_P(dtx, sizeof(dtx), (const char *)F("%-10u : "), millis());
  APP_PORT_DEBUG.println(String(dtx) + alog);
}

  FuzzyRule *createNewFuzzyRule(int ruleId, FuzzySet *in1, FuzzySet *in2, FuzzySet *out1) {
  FuzzyRuleConsequent *fzThen = new FuzzyRuleConsequent();
  fzThen->addOutput(out1);

  FuzzyRuleAntecedent *fzIf = new FuzzyRuleAntecedent();
  fzIf->joinWithAND(in1, in2);

  return new FuzzyRule(ruleId, fzIf, fzThen);
}

/**
 * init all fuzzy input, rule, and output
 * @method fuzzyInit
 */
void fuzzyInit() {
  // FuzzyInput DEBU
  FuzzyInput *fz_DEBU = new FuzzyInput(FUZZY_IN_DEBU);
  fz_DEBU->addFuzzySet(debu_tipis);
  fz_DEBU->addFuzzySet(debu_sedang);
  fz_DEBU->addFuzzySet(debu_tebal);
  fuzzy_main_obj->addFuzzyInput(fz_DEBU);

  // FuzzyInput CAHAYA
  FuzzyInput *fz_CAHAYA = new FuzzyInput(FUZZY_IN_CAHAYA);
  fz_CAHAYA->addFuzzySet(cahaya_gelap);
  fz_CAHAYA->addFuzzySet(cahaya_agak_terang);
  fz_CAHAYA->addFuzzySet(cahaya_terang);
  fuzzy_main_obj->addFuzzyInput(fz_CAHAYA);

  // FuzzyOutput PWM
  FuzzyOutput *fz_PWM = new FuzzyOutput(FUZZY_OUT_PWM);
  fz_PWM->addFuzzySet(pwm_lambat);
  fz_PWM->addFuzzySet(pwm_sedang);
  fz_PWM->addFuzzySet(pwm_cepat);
  fuzzy_main_obj->addFuzzyOutput(fz_PWM);

  // fuzzy rule
  fuzzy_main_obj->addFuzzyRule(
      createNewFuzzyRule(1, debu_tipis, cahaya_gelap, pwm_lambat));

  fuzzy_main_obj->addFuzzyRule(
      createNewFuzzyRule(2, debu_tipis, cahaya_agak_terang, pwm_lambat));

  fuzzy_main_obj->addFuzzyRule(
      createNewFuzzyRule(3, debu_tipis, cahaya_terang, pwm_cepat));

  fuzzy_main_obj->addFuzzyRule(
      createNewFuzzyRule(4, debu_sedang, cahaya_gelap, pwm_lambat));

  fuzzy_main_obj->addFuzzyRule(
      createNewFuzzyRule(5, debu_sedang, cahaya_agak_terang, pwm_sedang));

  fuzzy_main_obj->addFuzzyRule(
      createNewFuzzyRule(6, debu_sedang, cahaya_terang, pwm_cepat));

  fuzzy_main_obj->addFuzzyRule(
      createNewFuzzyRule(7, debu_tebal, cahaya_gelap, pwm_lambat));

  fuzzy_main_obj->addFuzzyRule(
      createNewFuzzyRule(8, debu_tebal, cahaya_agak_terang, pwm_lambat));

  fuzzy_main_obj->addFuzzyRule(
      createNewFuzzyRule(9, debu_tebal, cahaya_terang, pwm_cepat));
}


void fuzzyProcessInput(float DebuX, float CahayaX, float *pwmX) {
//  if (DebuX > 0) {
  fuzzy_main_obj->setInput(FUZZY_IN_DEBU, DebuX);
  fuzzy_main_obj->setInput(FUZZY_IN_CAHAYA, CahayaX);

  fuzzy_main_obj->fuzzify();

  *pwmX = fuzzy_main_obj->defuzzify(FUZZY_OUT_PWM);
  }
  
/**
 * debug fuzzy system
 * @method debugTest
 */
void debugTest() {
  float PwmX = 0.0;
  float DEBU = dustDensity;
  float CAHAYA = ncahaya;
  
  // set timeout for serial
  APP_PORT_DEBUG.setTimeout(30000);
  APP_DEBUG_PRINT(("PEMBACAAN SENSOR CAHAYA = ") + String(CAHAYA));
  APP_DEBUG_PRINT(("PEMBACAAN SENSOR DEBU = ") + String(DEBU));
  float DebuX = DEBU;
  float CahayaX = CAHAYA;
  
    //if ((DebuX >= 0.0)) {

        //if (DEBU>0){
          fuzzyProcessInput(DebuX, CahayaX, &PwmX);
       //}else{
          //PwmX =0;
        //}

    // hasil defuzikasi
    APP_DEBUG_PRINT(String("DEBU FUZZY = ") + String(DebuX) + String(" -- ") +
                    String(debu_tipis->getPertinence()) + String(" -- ") +
                    String(debu_sedang->getPertinence()) + String(" -- ") +
                    String(debu_tebal->getPertinence()));

    APP_DEBUG_PRINT(String("CAHAYA FUZZY = ") + String(CahayaX) + String(" -- ") +
                    String(cahaya_gelap->getPertinence()) + String(" -- ") +
                    String(cahaya_agak_terang->getPertinence()) + String(" -- ") +
                    String(cahaya_terang->getPertinence()));

    APP_DEBUG_PRINT(String("PWM = ") +
                    String((uint32_t)((float)PwmX)));
                                  
  //} else {
    //APP_DEBUG_PRINT(F("NO ACTION"));        
  //}
}

/**
 * all setup goes here
 * @method setup
 */
void setup() {
  APP_PORT_DEBUG.begin(9600);
  //fuzzy
  Serial.println("Log Fuzzy Logic Control"); //Print a message
  //sensors.begin();
  
  //sensor debu
  pinMode(pinLED,OUTPUT);

  //motor dc
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (ENA, OUTPUT);

  //sensor ultrasonik
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(buzzer, OUTPUT);
  pinMode(ledPin, OUTPUT);

  //servo
  myservo.attach(31); //definisikan pin yang digunakan untuk mengontrol motor servo adalah pin 31  

  //photodiode module
  //Serial.begin(9600);   
  pinMode(photodiode, OUTPUT); 

  // init fuzzy
  fuzzyInit();

  APP_DEBUG_PRINT(F("INIT DONE"));

}

/**
 * main loop
 * @method loop
 */
void loop() {
  //sensor debu
  digitalWrite(pinLED,LOW); // power on the LED
  delayMicroseconds(samplingTime);

  float voMeasured = analogRead(pinGP2Y);
  delayMicroseconds(deltaTime);
  digitalWrite(pinLED, HIGH);
  delayMicroseconds(sleepTime);
  float calcVoltage = voMeasured * (3.3 / 1024);

  float dustDensity = (0.17 * calcVoltage - 0.1) * 1000;
  if (dustDensity < 0) {
    dustDensity = 0;
  }
  //delay(10);

  //Serial.print(" - Dust Density: ");
  //Serial.print(dustDensity);
  //Serial.println(" Kg/m3");

   //photodiode modul
  int ncahaya = analogRead(photodiode);
  //Serial.println(ncahaya);
  //delay(8);

  //motor dc
  analogWrite(ENA, 255);
  digitalWrite(IN1, LOW);
  delay(500);
  digitalWrite(IN2, HIGH);
  delay(500);

  //sensor ultrasonik
  digitalWrite(trigPin, LOW);
  delayMicroseconds(1);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance= (duration/2) / 29.1;        // Calculating the distance

  //float DebuX = dustDensity;
  //float CahayaX = ncahaya;
  //fuzzyProcessInput(DebuX, CahayaX, &PwmX);
  
  safetyDistance = distance;
  if (safetyDistance < 10 && safetyDistance > 1){
    digitalWrite(buzzer, HIGH);
    digitalWrite(ledPin, HIGH);

    //Serial.print("Distance: ");
    //Serial.println(distance);

    analogWrite(ENA, 255);
  
    digitalWrite(IN1, HIGH);
    delay(500);
    digitalWrite(IN2, LOW);
    delay(500);
  }
  else if
    (safetyDistance < 20 && safetyDistance > 10){
    //digitalWrite(buzzer, HIGH);
    //digitalWrite(ledPin, HIGH);

    //Serial.print("Distance: ");
    //Serial.println(distance);

    analogWrite(ENA, 255);
  
    digitalWrite(IN1, HIGH);
    delay(500);
    digitalWrite(IN2, LOW);
    delay(500);
  }
  else
  {
    digitalWrite(buzzer, LOW);
    digitalWrite(ledPin, LOW);

    analogWrite(ENA, 255);
  
    digitalWrite(IN1, LOW);
    delay(500);
    digitalWrite(IN2, HIGH);
    delay(500);
  }

  // Prints the distance on the Serial Monitor
  //Serial.print("Distance: ");
  //Serial.println(distance);
  //delay(100);


  //servo
  for (pos = 0; pos <= 180; pos += 1) { // menjalankan fungsi for loop dari 0 - 180 dan nilai ini akan digunakan sebagai nilai posisiservo
  myservo.write(pos); // mengatur posisi servo berdasarkan nilai dari for loop
  delay(5); // beri jeda 5 miliseconds untuk setiap perubahan posisi
  }
  for (pos = 180; pos >= 0; pos -= 1) { // menjalankan fungsi for loop dari 180 - 0 dan nilai ini akan digunakan sebagai nilai posisiservo
  myservo.write(pos); // mengatur posisi servo berdasarkan nilai dari for loop
  delay(5); // beri jeda 5 miliseconds untuk setiap perubahan posisi
  }
  
  float PwmX = 0.0;
  
  float DEBU = dustDensity;
  float CAHAYA = ncahaya;
  
  debugTest();
  float DebuX = DEBU;
  float CahayaX = CAHAYA;

  if (DEBU > 0 ) {
  fuzzyProcessInput(DebuX, CahayaX, &PwmX);
  } else {
    PwmX = 0;
  }
  

  }
