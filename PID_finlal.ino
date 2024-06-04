#include <PID_v1.h>

// Motor A pins
const byte ENA_PIN_a = 9;
const byte IN1_PIN_a = 11;
const byte IN2_PIN_a = 10;

// Motor B pins
const byte ENA_PIN_b = 6;
const byte IN1_PIN_b = 8;
const byte IN2_PIN_b = 7;

// Motor B pins
const byte ENA_PIN_c = 3;
const byte IN1_PIN_c = 5;
const byte IN2_PIN_c = 4;

// Potentiometer pins
// POT1 = user, POT2 = robot;
const int POT1_PIN_a = A0;
const int POT2_PIN_a = A1;
const int POT1_PIN_b = A2;
const int POT2_PIN_b = A3;
const int POT1_PIN_c = A4; 
const int POT2_PIN_c = A5;

// PID variables
int val = 0;
int valb = 0;
int valc=0;

int encoder_valb=0;
int encoder_val =0;
int encoder_valc=0;
float kp = 0.2;
float ki = 0.00000 ;
float kd = 10.00;
float kpb = 0.2;
float kib = 0.0;
float kdb=12.00;
float kpc = 0.2;
float kic = 0.00000 ;
float kdc = 11.00;

float Theta, Theta_d, Thetab, Theta_db, Thetac, Theta_dc;
int dt, dtb, dtc;
unsigned long t, tb, tc;
unsigned long t_prev = 0, t_prevb=0, t_prevc=0;

int val_prev = 0,val_prevb=0,val_prevc=0;
float e, e_prev = 0, inte, inte_prev = 0;
float eb, e_prevb=0,inteb,inte_prevb=0;
float ec, e_prevc=0,intec,inte_prevc=0;

float Vmax = 12;
float Vmin = -12;
float V = 0.1;
float Vb=0.1;
float Vc=0;



//***Motor Driver Functions*****
void WriteDriverVoltage(float V, float Vmax,const byte PWMPin, const byte DirPin1, const byte DirPin2) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  }
  else if (V < 0) {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  }
  else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }
  analogWrite(PWMPin, PWMval);

}


void setup() {
  Serial.begin(9600);
  pinMode(IN1_PIN_a, OUTPUT);
  pinMode(IN1_PIN_b, OUTPUT);
  pinMode(IN1_PIN_c, OUTPUT);

  pinMode(IN2_PIN_a, OUTPUT);
  pinMode(IN2_PIN_b, OUTPUT);
  pinMode(IN2_PIN_c, OUTPUT);

}


void loop() {
  val = analogRead(POT1_PIN_a);                           // Read V_out from Reference Pot
  encoder_val =analogRead(POT2_PIN_a);               // Read V_out from Feedback Pot
  valb=analogRead(POT1_PIN_b);
  encoder_valb=analogRead(POT2_PIN_b);
  valc=analogRead(POT1_PIN_c);
  encoder_valc=analogRead(POT2_PIN_c);
  valc-=5;
  t = millis();
  dt = (t - t_prev);                                  // Time step
  Theta = val;                                        // Theta= Actual Angular Position of the Motor
  Theta_d = encoder_val;                              // Theta_d= Desired Angular Position of the Motor

  e = Theta_d - Theta;                                // Error
  inte = inte_prev + (dt * (e + e_prev) / 2);         // Integration of Error
  V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ; // Controlling Function

  if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    if (V < Vmin) {
      V = Vmin;
      inte = inte_prev;
      val_prev= val;
  }
  
  tb = millis();
  dtb = (tb - t_prevb);                                  // Time step
  Thetab = valb;                                        // Theta= Actual Angular Position of the Motor
  Theta_db = encoder_valb;                              // Theta_d= Desired Angular Position of the Motor

  eb = Theta_db - Thetab;                                // Error
  inteb = inte_prevb + (dtb * (eb + e_prevb) / 2);         // Integration of Error
  Vb = kpb * eb + kib * inteb + (kdb * (eb - e_prevb) / dtb) ; // Controlling Function

  if (Vb > Vmax) {
      Vb = Vmax;
      inteb = inte_prevb;
    }
    if (Vb < Vmin) {
      Vb = Vmin;
      inteb = inte_prevb;
      val_prevb= valb;
  }

  tc = millis();
  dtc = (tc - t_prevc);                                  // Time step

  if(valc<510){
    valc=510;
  }
  else if(620<valc){
    valc=620;
  }
  Thetac = valc;                                        // Theta= Actual Angular Position of the Motor
  Theta_dc = encoder_valc;                              // Theta_d= Desired Angular Position of the Motor


  ec = Theta_dc - Thetac;                                // Error
  intec = inte_prevc + (dtc * (ec + e_prevc) / 2);         // Integration of Error
  Vc = kpc * ec + kic * intec + (kdc * (ec - e_prevc) / dtc) ; // Controlling Function
Serial.println(encoder_valc); Serial.print(" \t");
  if (Vc > Vmax) {
      Vc = Vmax;
      intec = inte_prevc;
    }
    if (Vc < Vmin) {
      Vc = Vmin;
      intec = inte_prevc;
      val_prevc= valc;
  }

  WriteDriverVoltage(V, Vmax, ENA_PIN_a, IN1_PIN_a,IN2_PIN_a);
  WriteDriverVoltage(Vb, Vmax,ENA_PIN_b, IN1_PIN_b,IN2_PIN_b);
  WriteDriverVoltage(Vc, Vmax,ENA_PIN_c, IN1_PIN_c,IN2_PIN_c);

  //Serial.println(valc); Serial.print(" \t");
  //  Serial.print(Theta); Serial.print(" \t ");
    delay(10);
    t_prev = t;
    inte_prev = inte;
    e_prev = e; 

    t_prevb = tb;
    inte_prevb = inteb;
    e_prevb = eb; 

    t_prevc = tc;
    inte_prevc = intec;
    e_prevc = ec;
}