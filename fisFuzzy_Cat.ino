#include <Servo.h> // Include the Servo library
#include <Arduino.h>
#include "HX711.h"
#include "fis_header.h"

const int fis_gcI = 6; // Number of inputs to the fuzzy inference system
const int fis_gcO = 1; // Number of outputs to the fuzzy inference system
const int fis_gcR = 189; // Number of rules to the fuzzy inference system

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

Servo myservo; // Create a Servo object
const int trigPin = 7; // HC-SR04 trigger pin
const int echoPin = 6; // HC-SR04 echo pin
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
const int servoPin = 9;
const int ts_weight = 200;
const int ts_distance = 30;
HX711 scale;

// Setup routine runs once when you press reset:
void setup()
{
  Serial.begin(9600); // Initialize serial communication
  //espSerial.begin(9600); // Initialize ESP8266 serial communication
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); // Initialize the HX711 sensor
  scale.set_scale(455.88); // Replace with your calibration factor
  scale.tare(); // Reset the scale to 0

  myservo.attach(servoPin); // Attach servo to servoPin
  pinMode(trigPin, OUTPUT); // Set trigPin as OUTPUT
  pinMode(echoPin, INPUT); // Set echoPin as INPUT
}

// Loop routine runs over and over again forever:
void loop(){

  float distance = getDistance();
  float weight = scale.get_units();
  Serial.print(F("Distance: "));
  Serial.print(distance);
  Serial.println(" cm");
  Serial.print("Weight: ");
  Serial.print(weight, 1);;
  Serial.println(" g");

  delay (3000);
  int In_weight, In_distance;
  
    if (weight >= ((ts_weight/100)*65))
      {
        In_weight = 100;
      }
    
    else if (weight >= ((ts_weight/100)*35) && weight < ((ts_weight/100)*65) )
      {
        In_weight = 50;
      }
    
    else  if (weight < ((ts_weight/5)*35))
      {
        In_weight = 0;
      }


    if (distance < (ts_distance/5)*1.75)
      {
        In_distance = 0;
      }
    
    else if (distance >= ((ts_distance/5)*1.75) && distance < ((ts_distance/5)*3.25)) 
      {
        In_distance = 2.5;
      }
    
    else if (distance >= ((ts_distance/5)*3.25))
      {
        In_distance = 5;
      }

    // Read Input: Volume_Food
    g_fisInput[0] = In_distance;
    // Read Input: Weight_Food
    g_fisInput[1] = In_weight;
    // Read Input: Activeness
    g_fisInput[2] = 5;
    // Read Input: Age
    g_fisInput[3] = 5;
    // Read Input: Body_Condition
    g_fisInput[4] = 3;
    // Read Input: Breed_Size
    g_fisInput[5] = 5;

    g_fisOutput[0] = 0;

    fis_evaluate();

    // Set output vlaue: Servo_Time
    analogWrite(6 , g_fisOutput[0]);

    Serial.println(g_fisOutput[0]);

    if (g_fisOutput[0]<0.125)
      {
        myservo.write(90);
        delay(0);
      }
    else if (g_fisOutput[0]<0.325)
      {
        myservo.write(90);
        delay(500);
      }
    else if (g_fisOutput[0]<0.525)
      {
        myservo.write(90);
        delay(1000);
      }
    else if (g_fisOutput[0]<0.725)
      {
        myservo.write(90);
        delay(1500);
      }
    else if (g_fisOutput[0]<0.925)
      {
        myservo.write(90);
        delay(2000);
      }
    else if (g_fisOutput[0]<=1.00)
      {
        myservo.write(90);
        delay(2500);
      }
    //while(1);
}

// Function to get distance from HC-SR04 sensor
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.0343 / 2;

  return distance;
}


//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

// Trapezoidal Member Function
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
    return (FIS_TYPE) min(t1, t2);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trimf, fis_trapmf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 3, 3, 3, 3, 3, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 6 };

// Coefficients for the Input Member Functions
FIS_TYPE E[] = { -0.5, 0, 1.75 };
FIS_TYPE H[] = { 1, 2.5, 4 };
FIS_TYPE F[] = { 3.25, 5, 5 };
FIS_TYPE* VF[] = { E, H, F };
FIS_TYPE VL[] = { -50, 0, 35 };
FIS_TYPE L[] = { 20, 49.54, 80 };
FIS_TYPE Hvy[] = { 65, 100, 120 };
FIS_TYPE* WF[] = { VL, L, Hvy };
FIS_TYPE LA[] = { 0, 0, 2, 4 };
FIS_TYPE A[] = { 2.5, 4, 6, 7.5 };
FIS_TYPE MA[] = { 6, 8, 10, 10 };
FIS_TYPE* Active[] = { LA, A, MA };
FIS_TYPE K[] = { 0, 0, 1, 3 };
FIS_TYPE Ad[] = { 1, 3, 9, 10.5 };
FIS_TYPE Sen[] = { 9, 10.5, 15, 15 };
FIS_TYPE* Age[] = { K, Ad, Sen };
FIS_TYPE UW[] = { 0, 0, 2 };
FIS_TYPE N[] = { 1, 2.5, 4 };
FIS_TYPE OW[] = { 3, 5, 6 };
FIS_TYPE* BC[] = { UW, N, OW };
FIS_TYPE S[] = { 1, 2.5, 4 };
FIS_TYPE M[] = { 0, 0, 1, 2 };
FIS_TYPE Lg[] = { 3, 4, 5, 5 };
FIS_TYPE* BS[] = { S, M, Lg };
FIS_TYPE** MFI[] = { VF, WF, Active, Age, BC, BS };

// Coefficients for the Output Member Functions
FIS_TYPE CL[] = { 0, 0, 0.125 };
FIS_TYPE VS[] = { 0.075, 0.2, 0.325 };
FIS_TYPE Sh[] = { 0.275, 0.4, 0.525 };
FIS_TYPE Med[] = { 0.475, 0.6, 0.725 };
FIS_TYPE Long[] = { 0.675, 0.8, 0.925 };
FIS_TYPE VLong[] = { 0.875, 1, 1 };
FIS_TYPE* ST[] = { CL, VS, Sh, Med, Long, VLong };
FIS_TYPE** MFO[] = { ST };

// Input membership function set
int VolFood[] = { 0, 0, 0 };
int WeighFood[] = { 0, 0, 0 };
int Actv[] = { 1, 1, 1 };
int Ages[] = { 1, 1, 1 };
int BodyCond[] = { 0, 0, 0 };
int BreedSize[] = { 0, 1, 1 };
int* InputMF[] = { VolFood, WeighFood, Actv, Ages, BodyCond, BreedSize};

// Output membership function set
int ServoTime[] = { 0, 0, 0, 0, 0, 0 };
int* OutputMF[] = { ServoTime};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int RI0[] = { 1, 1, 2, 1, 1, 2 };
int RI1[] = { 1, 1, 3, 1, 1, 2 };
int RI2[] = { 1, 1, 3, 1, 2, 2 };
int RI3[] = { 1, 1, 2, 1, 2, 2 };
int RI4[] = { 1, 1, 1, 1, 2, 2 };
int RI5[] = { 1, 1, 2, 1, 3, 2 };
int RI6[] = { 1, 1, 1, 1, 3, 2 };
int RI7[] = { 2, 2, 2, 1, 1, 2 };
int RI8[] = { 2, 2, 3, 1, 1, 2 };
int RI9[] = { 2, 2, 3, 1, 2, 2 };
int RI10[] = { 2, 2, 2, 1, 2, 2 };
int RI11[] = { 2, 2, 1, 1, 2, 2 };
int RI12[] = { 2, 2, 2, 1, 3, 2 };
int RI13[] = { 2, 2, 1, 1, 3, 2 };
int RI14[] = { 3, 3, 2, 1, 1, 2 };
int RI15[] = { 3, 3, 3, 1, 1, 2 };
int RI16[] = { 3, 3, 1, 1, 2, 2 };
int RI17[] = { 3, 3, 2, 1, 2, 2 };
int RI18[] = { 3, 3, 3, 1, 2, 2 };
int RI19[] = { 3, 3, 2, 1, 3, 2 };
int RI20[] = { 3, 3, 1, 1, 3, 2 };
int RI21[] = { 1, 1, 2, 2, 1, 2 };
int RI22[] = { 1, 1, 3, 2, 1, 2 };
int RI23[] = { 1, 1, 1, 2, 2, 2 };
int RI24[] = { 1, 1, 2, 2, 2, 2 };
int RI25[] = { 1, 1, 3, 2, 2, 2 };
int RI26[] = { 1, 1, 1, 2, 3, 2 };
int RI27[] = { 1, 1, 2, 2, 3, 2 };
int RI28[] = { 2, 2, 2, 2, 1, 2 };
int RI29[] = { 2, 2, 3, 2, 1, 2 };
int RI30[] = { 2, 2, 1, 2, 2, 2 };
int RI31[] = { 2, 2, 2, 2, 2, 2 };
int RI32[] = { 2, 2, 3, 2, 2, 2 };
int RI33[] = { 2, 2, 2, 2, 3, 2 };
int RI34[] = { 2, 2, 1, 2, 3, 2 };
int RI35[] = { 3, 3, 2, 2, 1, 2 };
int RI36[] = { 3, 3, 3, 2, 1, 2 };
int RI37[] = { 3, 3, 1, 2, 2, 2 };
int RI38[] = { 3, 3, 2, 2, 2, 2 };
int RI39[] = { 3, 3, 3, 2, 2, 2 };
int RI40[] = { 3, 3, 1, 2, 3, 2 };
int RI41[] = { 3, 3, 2, 2, 3, 2 };
int RI42[] = { 1, 1, 2, 3, 1, 2 };
int RI43[] = { 1, 1, 3, 3, 1, 2 };
int RI44[] = { 1, 1, 1, 3, 2, 2 };
int RI45[] = { 1, 1, 2, 3, 2, 2 };
int RI46[] = { 1, 1, 3, 3, 2, 2 };
int RI47[] = { 1, 1, 1, 3, 3, 2 };
int RI48[] = { 1, 1, 2, 3, 3, 2 };
int RI49[] = { 2, 2, 2, 3, 1, 2 };
int RI50[] = { 2, 2, 3, 3, 1, 2 };
int RI51[] = { 2, 2, 1, 3, 2, 2 };
int RI52[] = { 2, 2, 2, 3, 2, 2 };
int RI53[] = { 2, 2, 3, 3, 2, 2 };
int RI54[] = { 2, 2, 1, 3, 3, 2 };
int RI55[] = { 2, 2, 2, 3, 3, 2 };
int RI56[] = { 3, 3, 2, 3, 1, 2 };
int RI57[] = { 3, 3, 3, 3, 1, 2 };
int RI58[] = { 3, 3, 1, 3, 2, 2 };
int RI59[] = { 3, 3, 2, 3, 2, 2 };
int RI60[] = { 3, 3, 3, 3, 2, 2 };
int RI61[] = { 3, 3, 1, 3, 3, 2 };
int RI62[] = { 3, 3, 2, 3, 3, 2 };
int RI63[] = { 1, 1, 2, 1, 1, 1 };
int RI64[] = { 1, 1, 3, 1, 1, 1 };
int RI65[] = { 1, 1, 1, 1, 2, 1 };
int RI66[] = { 1, 1, 2, 1, 2, 1 };
int RI67[] = { 1, 1, 3, 1, 2, 1 };
int RI68[] = { 1, 1, 1, 1, 3, 1 };
int RI69[] = { 1, 1, 2, 1, 3, 1 };
int RI70[] = { 2, 2, 2, 1, 1, 1 };
int RI71[] = { 2, 2, 3, 1, 1, 1 };
int RI72[] = { 2, 2, 1, 1, 2, 1 };
int RI73[] = { 2, 2, 2, 1, 2, 1 };
int RI74[] = { 2, 2, 3, 1, 2, 1 };
int RI75[] = { 2, 2, 1, 1, 3, 1 };
int RI76[] = { 2, 2, 2, 1, 3, 1 };
int RI77[] = { 3, 3, 2, 1, 1, 1 };
int RI78[] = { 3, 3, 3, 1, 1, 1 };
int RI79[] = { 3, 3, 1, 1, 2, 1 };
int RI80[] = { 3, 3, 2, 1, 2, 1 };
int RI81[] = { 3, 3, 3, 1, 2, 1 };
int RI82[] = { 3, 3, 1, 1, 3, 1 };
int RI83[] = { 3, 3, 2, 1, 3, 1 };
int RI84[] = { 1, 1, 2, 2, 1, 1 };
int RI85[] = { 1, 1, 3, 2, 1, 1 };
int RI86[] = { 1, 1, 3, 2, 2, 1 };
int RI87[] = { 1, 1, 2, 2, 2, 1 };
int RI88[] = { 1, 1, 1, 2, 2, 1 };
int RI89[] = { 1, 1, 1, 2, 3, 1 };
int RI90[] = { 1, 1, 2, 2, 3, 1 };
int RI91[] = { 2, 2, 2, 2, 1, 1 };
int RI92[] = { 2, 2, 3, 2, 1, 1 };
int RI93[] = { 2, 2, 3, 2, 2, 1 };
int RI94[] = { 2, 2, 2, 2, 2, 1 };
int RI95[] = { 2, 2, 1, 2, 2, 1 };
int RI96[] = { 2, 2, 1, 2, 3, 1 };
int RI97[] = { 2, 2, 2, 2, 3, 1 };
int RI98[] = { 3, 3, 2, 2, 1, 1 };
int RI99[] = { 3, 3, 3, 2, 1, 1 };
int RI100[] = { 3, 3, 1, 2, 2, 1 };
int RI101[] = { 3, 3, 2, 2, 2, 1 };
int RI102[] = { 3, 3, 3, 2, 2, 1 };
int RI103[] = { 3, 3, 1, 2, 3, 1 };
int RI104[] = { 3, 3, 2, 2, 3, 1 };
int RI105[] = { 1, 1, 2, 3, 1, 1 };
int RI106[] = { 1, 1, 3, 3, 1, 1 };
int RI107[] = { 1, 1, 1, 3, 2, 1 };
int RI108[] = { 1, 1, 2, 3, 2, 1 };
int RI109[] = { 1, 1, 3, 3, 2, 1 };
int RI110[] = { 1, 1, 1, 3, 3, 1 };
int RI111[] = { 1, 1, 2, 3, 3, 1 };
int RI112[] = { 2, 2, 2, 3, 1, 1 };
int RI113[] = { 2, 2, 3, 3, 1, 1 };
int RI114[] = { 2, 2, 1, 3, 2, 1 };
int RI115[] = { 2, 2, 2, 3, 2, 1 };
int RI116[] = { 2, 2, 3, 3, 2, 1 };
int RI117[] = { 2, 2, 1, 3, 3, 1 };
int RI118[] = { 2, 2, 2, 3, 3, 1 };
int RI119[] = { 3, 3, 2, 3, 1, 1 };
int RI120[] = { 3, 3, 3, 3, 1, 1 };
int RI121[] = { 3, 3, 1, 3, 2, 1 };
int RI122[] = { 3, 3, 2, 3, 2, 1 };
int RI123[] = { 3, 3, 3, 3, 2, 1 };
int RI124[] = { 3, 3, 1, 3, 3, 1 };
int RI125[] = { 3, 3, 2, 3, 3, 1 };
int RI126[] = { 1, 1, 2, 1, 1, 3 };
int RI127[] = { 1, 1, 3, 1, 1, 3 };
int RI128[] = { 1, 1, 1, 1, 2, 3 };
int RI129[] = { 1, 1, 2, 1, 2, 3 };
int RI130[] = { 1, 1, 3, 1, 2, 3 };
int RI131[] = { 1, 1, 1, 1, 3, 3 };
int RI132[] = { 1, 1, 2, 1, 3, 3 };
int RI133[] = { 2, 2, 2, 1, 1, 3 };
int RI134[] = { 2, 2, 3, 1, 1, 3 };
int RI135[] = { 2, 2, 1, 1, 2, 3 };
int RI136[] = { 2, 2, 2, 1, 2, 3 };
int RI137[] = { 2, 2, 3, 1, 2, 3 };
int RI138[] = { 2, 2, 1, 1, 3, 3 };
int RI139[] = { 2, 2, 2, 1, 3, 3 };
int RI140[] = { 3, 3, 2, 1, 1, 3 };
int RI141[] = { 3, 3, 3, 1, 1, 3 };
int RI142[] = { 3, 3, 1, 1, 2, 3 };
int RI143[] = { 3, 3, 2, 1, 2, 3 };
int RI144[] = { 3, 3, 3, 1, 2, 3 };
int RI145[] = { 3, 3, 1, 1, 3, 3 };
int RI146[] = { 3, 3, 2, 1, 3, 3 };
int RI147[] = { 1, 1, 2, 2, 1, 3 };
int RI148[] = { 1, 1, 3, 2, 1, 3 };
int RI149[] = { 1, 1, 1, 2, 2, 3 };
int RI150[] = { 1, 1, 2, 2, 2, 3 };
int RI151[] = { 1, 1, 3, 2, 2, 3 };
int RI152[] = { 1, 1, 1, 2, 3, 3 };
int RI153[] = { 1, 1, 2, 2, 3, 3 };
int RI154[] = { 2, 2, 2, 2, 1, 3 };
int RI155[] = { 2, 2, 3, 2, 1, 3 };
int RI156[] = { 2, 2, 1, 2, 2, 3 };
int RI157[] = { 2, 2, 2, 2, 2, 3 };
int RI158[] = { 2, 2, 3, 2, 2, 3 };
int RI159[] = { 2, 2, 1, 2, 3, 3 };
int RI160[] = { 2, 2, 2, 2, 3, 3 };
int RI161[] = { 3, 3, 2, 2, 1, 3 };
int RI162[] = { 3, 3, 3, 2, 1, 3 };
int RI163[] = { 3, 3, 1, 2, 2, 3 };
int RI164[] = { 3, 3, 2, 2, 2, 3 };
int RI165[] = { 3, 3, 3, 2, 2, 3 };
int RI166[] = { 3, 3, 1, 2, 3, 3 };
int RI167[] = { 3, 3, 2, 2, 3, 3 };
int RI168[] = { 1, 1, 2, 3, 1, 3 };
int RI169[] = { 1, 1, 3, 3, 1, 3 };
int RI170[] = { 1, 1, 1, 3, 2, 3 };
int RI171[] = { 1, 1, 2, 3, 2, 3 };
int RI172[] = { 1, 1, 3, 3, 2, 3 };
int RI173[] = { 1, 1, 1, 3, 3, 3 };
int RI174[] = { 1, 1, 2, 3, 3, 3 };
int RI175[] = { 2, 2, 2, 3, 1, 3 };
int RI176[] = { 2, 2, 3, 3, 1, 3 };
int RI177[] = { 2, 2, 1, 3, 2, 3 };
int RI178[] = { 2, 2, 2, 3, 2, 3 };
int RI179[] = { 2, 2, 3, 3, 2, 3 };
int RI180[] = { 2, 2, 1, 3, 3, 3 };
int RI181[] = { 2, 2, 2, 3, 3, 3 };
int RI182[] = { 3, 3, 2, 3, 1, 3 };
int RI183[] = { 3, 3, 3, 3, 1, 3 };
int RI184[] = { 3, 3, 1, 3, 2, 3 };
int RI185[] = { 3, 3, 2, 3, 2, 3 };
int RI186[] = { 3, 3, 3, 3, 2, 3 };
int RI187[] = { 3, 3, 1, 3, 3, 3 };
int RI188[] = { 3, 3, 2, 3, 3, 3 };
int* RI[] = { RI0, RI1, RI2, RI3, RI4, RI5, RI6, RI7, RI8, RI9, RI10, RI11, RI12, RI13, RI14, RI15, RI16, RI17, RI18, RI19, RI20, RI21, RI22, RI23, RI24, RI25, RI26, RI27, RI28, RI29, RI30, RI31, RI32, RI33, RI34, RI35, RI36, RI37, RI38, RI39, RI40, RI41, RI42, RI43, RI44, RI45, RI46, RI47, RI48, RI49, RI50, RI51, RI52, RI53, RI54, RI55, RI56, RI57, RI58, RI59, RI60, RI61, RI62, RI63, RI64, RI65, RI66, RI67, RI68, RI69, RI70, RI71, RI72, RI73, RI74, RI75, RI76, RI77, RI78, RI79, RI80, RI81, RI82, RI83, RI84, RI85, RI86, RI87, RI88, RI89, RI90, RI91, RI92, RI93, RI94, RI95, RI96, RI97, RI98, RI99, RI100, RI101, RI102, RI103, RI104, RI105, RI106, RI107, RI108, RI109, RI110, RI111, RI112, RI113, RI114, RI115, RI116, RI117, RI118, RI119, RI120, RI121, RI122, RI123, RI124, RI125, RI126, RI127, RI128, RI129, RI130, RI131, RI132, RI133, RI134, RI135, RI136, RI137, RI138, RI139, RI140, RI141, RI142, RI143, RI144, RI145, RI146, RI147, RI148, RI149, RI150, RI151, RI152, RI153, RI154, RI155, RI156, RI157, RI158, RI159, RI160, RI161, RI162, RI163, RI164, RI165, RI166, RI167, RI168, RI169, RI170, RI171, RI172, RI173, RI174, RI175, RI176, RI177, RI178, RI179, RI180, RI181, RI182, RI183, RI184, RI185, RI186, RI187, RI188 };

// Rule Outputs
int RO0[] = { 3 };
int RO1[] = { 3 };
int RO2[] = { 3 };
int RO3[] = { 3 };
int RO4[] = { 3 };
int RO5[] = { 3 };
int RO6[] = { 3 };
int RO7[] = { 2 };
int RO8[] = { 2 };
int RO9[] = { 2 };
int RO10[] = { 2 };
int RO11[] = { 2 };
int RO12[] = { 2 };
int RO13[] = { 2 };
int RO14[] = { 1 };
int RO15[] = { 1 };
int RO16[] = { 1 };
int RO17[] = { 1 };
int RO18[] = { 1 };
int RO19[] = { 1 };
int RO20[] = { 1 };
int RO21[] = { 4 };
int RO22[] = { 4 };
int RO23[] = { 4 };
int RO24[] = { 4 };
int RO25[] = { 4 };
int RO26[] = { 3 };
int RO27[] = { 3 };
int RO28[] = { 3 };
int RO29[] = { 3 };
int RO30[] = { 3 };
int RO31[] = { 3 };
int RO32[] = { 3 };
int RO33[] = { 2 };
int RO34[] = { 2 };
int RO35[] = { 1 };
int RO36[] = { 1 };
int RO37[] = { 1 };
int RO38[] = { 1 };
int RO39[] = { 1 };
int RO40[] = { 1 };
int RO41[] = { 1 };
int RO42[] = { 3 };
int RO43[] = { 3 };
int RO44[] = { 3 };
int RO45[] = { 3 };
int RO46[] = { 3 };
int RO47[] = { 3 };
int RO48[] = { 3 };
int RO49[] = { 2 };
int RO50[] = { 2 };
int RO51[] = { 2 };
int RO52[] = { 2 };
int RO53[] = { 2 };
int RO54[] = { 2 };
int RO55[] = { 2 };
int RO56[] = { 1 };
int RO57[] = { 1 };
int RO58[] = { 1 };
int RO59[] = { 1 };
int RO60[] = { 1 };
int RO61[] = { 1 };
int RO62[] = { 1 };
int RO63[] = { 4 };
int RO64[] = { 4 };
int RO65[] = { 4 };
int RO66[] = { 4 };
int RO67[] = { 4 };
int RO68[] = { 3 };
int RO69[] = { 3 };
int RO70[] = { 3 };
int RO71[] = { 3 };
int RO72[] = { 3 };
int RO73[] = { 3 };
int RO74[] = { 3 };
int RO75[] = { 2 };
int RO76[] = { 2 };
int RO77[] = { 1 };
int RO78[] = { 1 };
int RO79[] = { 1 };
int RO80[] = { 1 };
int RO81[] = { 1 };
int RO82[] = { 1 };
int RO83[] = { 1 };
int RO84[] = { 5 };
int RO85[] = { 5 };
int RO86[] = { 5 };
int RO87[] = { 4 };
int RO88[] = { 4 };
int RO89[] = { 3 };
int RO90[] = { 3 };
int RO91[] = { 3 };
int RO92[] = { 3 };
int RO93[] = { 3 };
int RO94[] = { 2 };
int RO95[] = { 2 };
int RO96[] = { 2 };
int RO97[] = { 2 };
int RO98[] = { 1 };
int RO99[] = { 1 };
int RO100[] = { 1 };
int RO101[] = { 1 };
int RO102[] = { 1 };
int RO103[] = { 1 };
int RO104[] = { 1 };
int RO105[] = { 3 };
int RO106[] = { 3 };
int RO107[] = { 3 };
int RO108[] = { 3 };
int RO109[] = { 3 };
int RO110[] = { 3 };
int RO111[] = { 3 };
int RO112[] = { 2 };
int RO113[] = { 2 };
int RO114[] = { 2 };
int RO115[] = { 2 };
int RO116[] = { 2 };
int RO117[] = { 2 };
int RO118[] = { 2 };
int RO119[] = { 1 };
int RO120[] = { 1 };
int RO121[] = { 1 };
int RO122[] = { 1 };
int RO123[] = { 1 };
int RO124[] = { 1 };
int RO125[] = { 1 };
int RO126[] = { 5 };
int RO127[] = { 5 };
int RO128[] = { 4 };
int RO129[] = { 4 };
int RO130[] = { 4 };
int RO131[] = { 4 };
int RO132[] = { 4 };
int RO133[] = { 3 };
int RO134[] = { 3 };
int RO135[] = { 2 };
int RO136[] = { 2 };
int RO137[] = { 2 };
int RO138[] = { 2 };
int RO139[] = { 2 };
int RO140[] = { 1 };
int RO141[] = { 1 };
int RO142[] = { 1 };
int RO143[] = { 1 };
int RO144[] = { 1 };
int RO145[] = { 1 };
int RO146[] = { 1 };
int RO147[] = { 6 };
int RO148[] = { 6 };
int RO149[] = { 5 };
int RO150[] = { 5 };
int RO151[] = { 5 };
int RO152[] = { 4 };
int RO153[] = { 4 };
int RO154[] = { 4 };
int RO155[] = { 4 };
int RO156[] = { 3 };
int RO157[] = { 3 };
int RO158[] = { 3 };
int RO159[] = { 2 };
int RO160[] = { 2 };
int RO161[] = { 1 };
int RO162[] = { 1 };
int RO163[] = { 1 };
int RO164[] = { 1 };
int RO165[] = { 1 };
int RO166[] = { 1 };
int RO167[] = { 1 };
int RO168[] = { 5 };
int RO169[] = { 5 };
int RO170[] = { 4 };
int RO171[] = { 4 };
int RO172[] = { 4 };
int RO173[] = { 3 };
int RO174[] = { 3 };
int RO175[] = { 3 };
int RO176[] = { 3 };
int RO177[] = { 2 };
int RO178[] = { 2 };
int RO179[] = { 2 };
int RO180[] = { 2 };
int RO181[] = { 2 };
int RO182[] = { 1 };
int RO183[] = { 1 };
int RO184[] = { 1 };
int RO185[] = { 1 };
int RO186[] = { 1 };
int RO187[] = { 1 };
int RO188[] = { 1 };
int* RO[] = { RO0, RO1, RO2, RO3, RO4, RO5, RO6, RO7, RO8, RO9, RO10, RO11, RO12, RO13, RO14, RO15, RO16, RO17, RO18, RO19, RO20, RO21, RO22, RO23, RO24, RO25, RO26, RO27, RO28, RO29, RO30, RO31, RO32, RO33, RO34, RO35, RO36, RO37, RO38, RO39, RO40, RO41, RO42, RO43, RO44, RO45, RO46, RO47, RO48, RO49, RO50, RO51, RO52, RO53, RO54, RO55, RO56, RO57, RO58, RO59, RO60, RO61, RO62, RO63, RO64, RO65, RO66, RO67, RO68, RO69, RO70, RO71, RO72, RO73, RO74, RO75, RO76, RO77, RO78, RO79, RO80, RO81, RO82, RO83, RO84, RO85, RO86, RO87, RO88, RO89, RO90, RO91, RO92, RO93, RO94, RO95, RO96, RO97, RO98, RO99, RO100, RO101, RO102, RO103, RO104, RO105, RO106, RO107, RO108, RO109, RO110, RO111, RO112, RO113, RO114, RO115, RO116, RO117, RO118, RO119, RO120, RO121, RO122, RO123, RO124, RO125, RO126, RO127, RO128, RO129, RO130, RO131, RO132, RO133, RO134, RO135, RO136, RO137, RO138, RO139, RO140, RO141, RO142, RO143, RO144, RO145, RO146, RO147, RO148, RO149, RO150, RO151, RO152, RO153, RO154, RO155, RO156, RO157, RO158, RO159, RO160, RO161, RO162, RO163, RO164, RO165, RO166, RO167, RO168, RO169, RO170, RO171, RO172, RO173, RO174, RO175, RO176, RO177, RO178, RO179, RO180, RO181, RO182, RO183, RO184, RO185, RO186, RO187, RO188 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0, 0, 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 5, 100, 10, 15, 5, 5 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 1 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = RO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[OutputMF[o][index]])(x, MFO[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[OutputMF[o][index]])(x, MFO[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput3[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput4[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput5[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, fuzzyInput3, fuzzyInput4, fuzzyInput5, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[InputMF[i][j]])(g_fisInput[i], MFI[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = RI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = RI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
