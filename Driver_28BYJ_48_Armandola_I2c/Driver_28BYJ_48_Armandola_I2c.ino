
//#include "TramasMicros2.h"
#include <Wire.h>
//#include "PID_v1.h"

////Define Variables we'll be connecting to
//double Setpoint, Input,Input0, Output;
//
////Define the aggressive and conservative Tuning Parameters
//double aggKp=4, aggKi=0.2, aggKd=1;
//double consKp=1, consKi=0.05, consKd=0.25;
//
////Specify the links and initial tuning parameters
//PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


//TramaTiempo blink_Led = TramaTiempo(50000, Led);
//definicion de pins
#define PIN_INPUT A7
#define PIN_OUTPUT 10
const int motor1Pin1 = 6;    // 28BYJ48 In1
const int motor1Pin2 = 7;    // 28BYJ48 In2
const int motor1Pin3 = 8;   // 28BYJ48 In3
const int motor1Pin4 = 9;   // 28BYJ48 In4

const int motor2Pin1 = 2;    // 28BYJ48 In1
const int motor2Pin2 = 3;    // 28BYJ48 In2
const int motor2Pin3 = 4;   // 28BYJ48 In3
const int motor2Pin4 = 5;   // 28BYJ48 In4

const int motor3Pin1 = A0;    // 28BYJ48 In1
const int motor3Pin2 = A1;    // 28BYJ48 In2
const int motor3Pin3 = A2;   // 28BYJ48 In3
const int motor3Pin4 = A3;   // 28BYJ48 In4

const int motor3PinPow = A3;   // 28BYJ48 In4

//const int motor3PinStep = A6;   // 28BYJ48 In3
//const int motor3PinDir = A7;   // 28BYJ48 In4
//
//const int motor2PinStep = 10;   // 28BYJ48 In3
//const int motor2PinDir = 11;   // 28BYJ48 In4
//
//const int motor1PinStep = A2;   // 28BYJ48 In3
//const int motor1PinDir = A3;   // 28BYJ48 In4

const int PinLed = 13;   // 28BYJ48 In3
//definicion variables
int motorSpeed = 900;// 100000; 1200;   //variable para fijar la velocidad
int stepCounter1 = 0;     // contador para los pasos
int stepCounter2 = 0;
int stepCounter3 = 0;
int stepsPerRev = 4076;  // pasos para una vuelta completa

//tablas con la secuencia de encendido (descomentar la que necesiteis)
//secuencia 1-fase
//const int numSteps = 4;
//const int stepsLookup[4] = { B1000, B0100, B0010, B0001 };

//secuencia 2-fases
//const int numSteps = 4;
//const int stepsLookup[4] = { B1100, B0110, B0011, B1001 };

//secuencia media fase
const int numSteps = 8;
const int stepsLookup[8] = { B1000, B1100, B0100, B0110, B0010, B0011, B0001, B1001 };


void setup()
{
  bitWrite(ADCSRA, ADPS2, 1);
  bitWrite(ADCSRA, ADPS1, 0);
  bitWrite(ADCSRA, ADPS0, 1);
  delay(100);
  //ADPS2 - ADPS1 - ADPS0 - Division Factor
  //0        - 0       - 0        ->2
  //0        - 0       - 1        ->2
  //0        - 1       - 0        ->4
  //0        - 1       - 1        ->8
  //1        - 0       - 0        ->16
  //1        - 0       - 1        ->32
  //1        - 1       - 0        ->64
  //1        - 1       - 1        ->128
  //declarar pines como salida
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1Pin3, OUTPUT);
  pinMode(motor1Pin4, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor2Pin3, OUTPUT);
  pinMode(motor2Pin4, OUTPUT);

  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor3Pin3, OUTPUT);
  pinMode(motor3Pin4, OUTPUT);

  //pinMode(motor3PinStep, INPUT);
  //pinMode(motor3PinDir, INPUT);



  pinMode(PinLed, OUTPUT);

  // Unimos este dispositivo al bus I2C con direcci�n 1
  Wire.begin(0x07);
  delay(23);
  // Registramos el evento al recibir datos
  Wire.onReceive(receiveEvent);

  digitalWrite(PinLed, LOW);
  //initialize the variables we're linked to
  //	Input = analogRead(PIN_INPUT);
  //	Setpoint = 100;

  //turn the PID on
  //myPID.SetMode(AUTOMATIC);


  // Iniciamos el monitor serie para monitorear la comunicaci�n
  // Serial.begin(9600);

}
void loop()
{
  ////	blink_Led.check();
  //int x = 0;
  // for (x = 0; x < 255000; x ++)
  //{
  // clockwise3();
  // delay(7);
  //}
  // for (x = 0; x < 255000; x ++)
  //{
  //
  //anticlockwise3();
  //delay(7);
  //    }


}


// Funci�n que se ejecuta siempre que se reciben datos del master
// siempre que en el master se ejecute la sentencia endTransmission
// recibir� toda la informaci�n que hayamos pasado a trav�s de la sentencia Wire.write
void receiveEvent(int howMany) {
  bool Correcto = true ;
  int Referencia = 0;
  int Valor = 0;
  if (Wire.available() > 0) {
    Referencia = Wire.read();
  }
  if (Wire.available() > 0) {
    Valor = Wire.read();
  }

  switch (Referencia) {
    case 1:
      if (Valor == 0) {
        setOutput1(stepCounter1);
      }
      else if (Valor == 1) {
        clockwise1();
      }
      else if (Valor == 2) {
        anticlockwise1();
      }
      else if (Valor == 3) {
        setOff1();
      }
      Correcto = true ;
      break;
    case 2:
      if (Valor == 0) {
        setOutput2(stepCounter2);
      }
      else if (Valor == 1) {
        clockwise2();
      }
      else if (Valor == 2) {
        anticlockwise2();
      }
      else if (Valor == 3) {
        setOff2();
      }
      //Serial.print("Estado ");
      //Serial.println(estado);
      Correcto = true ;
      break;
    case 3:
      if (Valor == 0) {
        setOutput3(stepCounter3);
      }
      else if (Valor == 1) {
        clockwise3();
      }
      else if (Valor == 2) {
        anticlockwise3();
      }
      else if (Valor == 3) {
        setOff3();
      }
      //Serial.print("Estado ");
      //Serial.println(estado);
      Correcto = true ;
      break;
    case 4:
      //		Setpoint=Valor;
      Correcto = true ;
      break;
    default:
      //Correcto = false ;
      break;
  }


  //
  //if(Wire.available() == 1) {
  //    if (Valor==0){setOutput1(stepCounter1);}
  //    else if (Valor==1) {clockwise1();}
  //    else if (Valor==2) {anticlockwise1();}
  //    else if (Valor==3) {setOff1();}
  //    Correcto = true ;
  //}
  //if(Wire.available() == 2){
  //    if (Valor==0){setOutput2(stepCounter2);}
  //    else if (Valor==1) {clockwise2();}
  //    else if (Valor==2) {anticlockwise2();}
  //    else if (Valor==3) {setOff2();}
  //    //Serial.print("Estado ");
  //    //Serial.println(estado);
  //    Correcto = true ;
  //}
  //if(Wire.available() == 3){
  //    if (Valor==0){setOutput3(stepCounter3);}
  //    else if (Valor==1) {clockwise3();}
  //    else if (Valor==2) {anticlockwise3();}
  //    else if (Valor==3) {setOff3();}
  //    //Serial.print("Estado ");
  //    //Serial.println(estado);
  //    Correcto = true ;
  //}
  //if(Wire.available() == 4){
  ////    Setpoint=Valor;
  //    Correcto = true ;
  //
  //}




  //// Si hay dos bytes disponibles
  //if (Referencia == 1)
  //{
  //// Leemos el primero que ser� el pin
  ////Valor = Wire.read();
  ////Serial.print("LED ");
  ////Serial.println(pinOut);
  //if (Valor==0){setOutput1(stepCounter1);}
  //else if (Valor==1) {clockwise1();}
  //else if (Valor==2) {anticlockwise1();}
  //else if (Valor==3) {setOff1();}
  //Correcto = true ;
  //}
  //// Si hay un byte disponible
  //if (Referencia == 2)
  //{
  ////Valor = Wire.read();
  //
  //if (Valor==0){setOutput2(stepCounter2);}
  //else if (Valor==1) {clockwise2();}
  //else if (Valor==2) {anticlockwise2();}
  //else if (Valor==3) {setOff2();}
  ////Serial.print("Estado ");
  ////Serial.println(estado);
  //Correcto = true ;
  //}
  //if (Referencia == 3)
  //{
  ////Valor = Wire.read();
  //if (Valor==0){setOutput3(stepCounter3);}
  //else if (Valor==1) {clockwise3();}
  //else if (Valor==2) {anticlockwise3();}
  //else if (Valor==3) {setOff3();}
  ////Serial.print("Estado ");
  ////Serial.println(estado);
  //Correcto = true ;
  //}
  //if (Referencia == 4)
  //{
  ////Valor = Wire.read();
  //Setpoint=Valor;
  //Correcto = true ;
  //
  //}
  //if (Correcto == true )
  //	{if (digitalRead(PinLed)){digitalWrite(PinLed, LOW);blink_Led.setInterval(50000) ;} else {digitalWrite(PinLed, HIGH);blink_Led.setInterval(50000) ;}}
  //{if (digitalRead(PinLed)){digitalWrite(PinLed, LOW);} else {digitalWrite(PinLed, HIGH);}}
  // Activamos/desactivamos salida
  //digitalWrite(pinOut,estado);
  if (digitalRead(PinLed)) {
    digitalWrite(PinLed, LOW);
  } else {
    digitalWrite(PinLed, HIGH);
  }
}


void  Led() {
  //lop0();

  ////Input = analogRead(PIN_INPUT);
  //Temometros2();
  //double gap = abs(Setpoint-((Input+Input0)/2)); //distance away from setpoint
  //if (gap < 10)
  //{  //we're close to setpoint, use conservative tuning parameters
  //myPID.SetTunings(consKp, consKi, consKd);
  //}
  //else
  //{
  ////we're far from setpoint, use aggressive tuning parameters
  //myPID.SetTunings(aggKp, aggKi, aggKd);
  //}
  //
  //myPID.Compute();
  //analogWrite(PIN_OUTPUT, Output);
  //Input0 = Input;

}

void Temometros2() {
  //	Serial.print(" -- calentador Celsius: ");

  float R2 = 9840;//9850;      //Valor medido de R2, no exactamente 10K, cambialo por el valor que uses tu.



  //24000 t5?C
  //9900  t25?C
  //4000  t45?C

  float A = 2.285544891E-3;//0.8589128459E-3; //???Datos calculados para un caso concreto...
  float B = 0.4815850804E-4;//2.607773241E-4;  //...debes cambiarlos por los que calcules tu...
  float C = 8.030689541E-7;//1.244870433E-7;  //...de tu propio termistor NTC!!!
  int TempLuctura1 =  analogRead(PIN_INPUT);
  long resistencia;
  //TempC=cTemp1;
  //	cTempS1a=0;

  resistencia = R2 * ((1024.0 / TempLuctura1) - 1); //Calculamos R1 mediante la lectura analogica
  TempLuctura1 = log(resistencia);
  TempLuctura1 = 1 / (A + (B * TempLuctura1) + (C * TempLuctura1 * TempLuctura1 * TempLuctura1));
  TempLuctura1 = TempLuctura1 - 273.15;  // Kelvin a grados centigrados
  //cTemp1=(cTemp1+TempC)/2;

  //Serial.print(cTempS1a);//Serial.print(TempC,1);

  //	Input=TempLuctura1;


  //TempCS1 = (cTempS1a+cTempS1b+cTempS1c)/3;
  //cTempS1c=cTempS1b;
  //cTempS1b= cTempS1a;
  Serial.println(" -- --");

}

int i_ContPadod = 0;
bool i_Dir = false;
void lop0()
{
  //for (int i = 0; i < stepsPerRev * 2; i++)
  //{
  //clockwise1();
  //delayMicroseconds(motorSpeed);
  //}
  //for (int i = 0; i < stepsPerRev * 2; i++)
  //{
  //anticlockwise1();
  //delayMicroseconds(motorSpeed);
  //}
  //setOff1();
  //delay(1000);
  if (i_Dir == false) {
    if (i_ContPadod < stepsPerRev * 2) {
      clockwise1(); clockwise2(); clockwise3();
      i_ContPadod++;
    }
    else {
      i_Dir = true;
    }

  }
  else {
    if (i_ContPadod > 0) {
      clockwise1(); clockwise2(); clockwise3();
      i_ContPadod--;
    }
    else {
      i_Dir = false;
    }
  }



}

void clockwise1()
{
  stepCounter1++;
  if (stepCounter1 >= numSteps) stepCounter1 = 0;
  setOutput1(stepCounter1);
}
void anticlockwise1()
{
  stepCounter1--;
  if (stepCounter1 < 0) stepCounter1 = numSteps - 1;
  setOutput1(stepCounter1);
}
void setOff1()
{
  digitalWrite(motor1Pin1, 0);
  digitalWrite(motor1Pin2, 0);
  digitalWrite(motor1Pin3, 0);
  digitalWrite(motor1Pin4, 0);
}
void setOutput1(int step)
{
  digitalWrite(motor1Pin1, bitRead(stepsLookup[step], 0));
  digitalWrite(motor1Pin2, bitRead(stepsLookup[step], 1));
  digitalWrite(motor1Pin3, bitRead(stepsLookup[step], 2));
  digitalWrite(motor1Pin4, bitRead(stepsLookup[step], 3));
}

void clockwise2()
{
  stepCounter2++;
  if (stepCounter2 >= numSteps) stepCounter2 = 0;
  setOutput2(stepCounter2);
}
void anticlockwise2()
{
  stepCounter2--;
  if (stepCounter2 < 0) stepCounter2 = numSteps - 1;
  setOutput2(stepCounter2);
}
void setOff2()
{
  digitalWrite(motor2Pin1, 0);
  digitalWrite(motor2Pin2, 0);
  digitalWrite(motor2Pin3, 0);
  digitalWrite(motor2Pin4, 0);
}
void setOutput2(int step)
{
  digitalWrite(motor2Pin1, bitRead(stepsLookup[step], 0));
  digitalWrite(motor2Pin2, bitRead(stepsLookup[step], 1));
  digitalWrite(motor2Pin3, bitRead(stepsLookup[step], 2));
  digitalWrite(motor2Pin4, bitRead(stepsLookup[step], 3));
}


void clockwise3()
{
  stepCounter3++;
  if (stepCounter3 >= numSteps) stepCounter3 = 0;
  setOutput3(stepCounter3);
}
void anticlockwise3()
{
  stepCounter3--;
  if (stepCounter3 < 0) stepCounter3 = numSteps - 1;
  setOutput3(stepCounter3);
}
void setOff3()
{
  digitalWrite(motor3Pin1, 0);
  digitalWrite(motor3Pin2, 0);
  digitalWrite(motor3Pin3, 0);
  digitalWrite(motor3Pin4, 0);
}
void setOutput3(int step)
{
  digitalWrite(motor3Pin1, bitRead(stepsLookup[step], 0));
  digitalWrite(motor3Pin2, bitRead(stepsLookup[step], 1));
  digitalWrite(motor3Pin3, bitRead(stepsLookup[step], 2));
  digitalWrite(motor3Pin4, bitRead(stepsLookup[step], 3));
}
