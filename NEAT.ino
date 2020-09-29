#include "Wire.h"
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif


#define LIGHT_PIN A0
#define LED_PIN    2
#define LED_COUNT 3

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//========================================================================

const int MAX_CONNECTIONS           = 15;
const int MAX_NODES                 = 10;
const int MAX_CONNECTIONS_PER_NODE  = 2;

const int INPUT_NODES               = 3;
const int OUTPUT_NODES              = LED_COUNT + 1;
const int MUTATIONS                 = 50;

int timer                           = 0;
int timerMax                        = 50;
int brightnessStart                 = 250;
int brightness                      = brightnessStart;
unsigned long millisPrev            = 0;
unsigned long millisPrevBrightness  = 0;

//=========================================================================


struct Connection {
  int from;
  float weight;
};

struct Node {
  float x;
  float output;
  Connection connections[MAX_CONNECTIONS];
  int connectionsSize;
};

class Neat {
public:
  Node nodes[MAX_NODES];
  int nodesSize;
  int connectionsSize;
  int inputSize;
  int outputSize;

  Neat(int input, int output) {
    this->nodesSize = 0;
    this->connectionsSize = 0;
    this->inputSize = input;
    this->outputSize = output;

    for (int i = 0; i < inputSize; i++)
      AddNode(0);
 
    for (int i = 0; i < outputSize; i++)
      AddNode(1);

    for (int i = 0; i < MUTATIONS; i++) {
      MutateAddConnection();
      if (random(2) == 0)
        MutateAddNode();
    }

    //Print();
  }

  float RandDec() {
    return random(1000) / 10000.0;
  }

  int AddNode(float x) {
    Node node;
    node.x = x;
    node.output = 0;
    node.connectionsSize = 0;

    int index = nodesSize;
    for (int i = 0; i < nodesSize; i++) {
      if (x < nodes[i].x) {
        index = i;
        break;
      }
    }

    for (int i = index; i <= nodesSize; i++) {
      Node tmp = nodes[i];
      nodes[i] = node;
      node = tmp;
    }
    
    nodesSize++;
    return index;
  }

  void AddConnection(int from, int to) {
    Connection con;
    con.from = from;
    con.weight = (RandDec() - 0.5) * 2;
    nodes[to].connections[nodes[to].connectionsSize] = con;
    nodes[to].connectionsSize++;
    connectionsSize++;
  }

  bool ConnectionExists(int from, int to) {
    for (int i = 0; i < nodes[to].connectionsSize; i++) {
      if (nodes[to].connections[i].from == from)
        return true;
    }
    return false;
  }

  void MutateAddConnection() {
    if (nodesSize >= MAX_NODES || connectionsSize >= MAX_CONNECTIONS)
      return;
      
    for (int i = 0; i < 100; i++) {
      int index1 = random(nodesSize);
      int index2 = random(nodesSize);

      if (nodes[index1].x == nodes[index2].x)
        continue;

      if (nodes[index1].x < nodes[index2].x) {
        if (nodes[index2].connectionsSize >= MAX_CONNECTIONS_PER_NODE)
          continue;

        if (ConnectionExists(index1, index2))
          continue;

        AddConnection(index1, index2);
      }
      else {
        if (nodes[index1].connectionsSize >= MAX_CONNECTIONS_PER_NODE)
          continue;

        if (ConnectionExists(index2, index1))
          continue;

        AddConnection(index2, index1);
      }

      break;
    }
  }

  void MutateAddNode() {
    if (nodesSize >= MAX_NODES || connectionsSize >= MAX_CONNECTIONS)
      return;
    
    for (int i = 0; i < 100; i++) {
      int index = random(nodesSize);
      Node *node = &nodes[index];
      if (node->connectionsSize == 0)
        continue;

      int conIndex = random(node->connectionsSize);
      int firstNodeIndex = node->connections[conIndex].from;
      int newNodeIndex = AddNode((node->x + nodes[node->connections[conIndex].from].x) / 2);
      node->connections[conIndex].from = newNodeIndex;
      
      AddConnection(firstNodeIndex, newNodeIndex);
      
      break;
    }
  }

  void Calculate(double input[]) {
    for (int i = 0; i < inputSize; i++)
      nodes[i].output = input[i];

    for (int i = inputSize; i < nodesSize; i++) {
      double sum = 0;
      for (int j = 0; j < nodes[i].connectionsSize; j++) {
        sum += nodes[nodes[i].connections[j].from].output * nodes[i].connections[j].weight;
      }
      nodes[i].output = tanh(sum);
    }
  }

  double *GetOutput() {
    double static output[OUTPUT_NODES];
    int count = 0;
    
    for (int i = nodesSize - outputSize; i < nodesSize; i++) {
      output[count++] = nodes[i].output;
    }

    return output;
  }

  void Print() {
    for (int i = 0; i < nodesSize; i++) {
      Serial.print("Node ");
      Serial.print(i);
      Serial.print(": x");
      Serial.print(nodes[i].x);
      Serial.print(" connecions:");
      Serial.print(nodes[i].connectionsSize);
      Serial.print(" output:");
      Serial.print(nodes[i].output);
      Serial.println();
    }
    Serial.println();
  }
};



//======================================================================================

Neat *neat;




void UpdateLights() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  double accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  double accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  double accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  double temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  double gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  double gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  double gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  accelerometer_x /= 40000.0;
  accelerometer_y /= 40000.0;
  accelerometer_z /= 40000.0;
  temperature /= 50.0;
  gyro_x /= 20000.0;
  gyro_y /= 20000.0;
  gyro_z /= 20000.0;
  
  double lightLevel = analogRead(LIGHT_PIN);

  //double input[] = {(lightLevel - 500) / 500.0, sin(millis() / 1000.0), sin(millis() / 5000.0), sin(millis() / 10000.0), accelerometer_x, accelerometer_y, accelerometer_z, temperature};
  double input[] = {sin(millis() / 3000.0), sin(millis() / 5000.0), sin(millis() / 10000.0)};

  neat->Calculate(input);

  double *output = neat->GetOutput();

/*
  Serial.println("INPUT:");
  for (int i = 0; i < neat->inputSize; i++) {
    Serial.println(input[i]);
  }

  Serial.println("OUTPUT:");
  for (int i = 0; i < neat->outputSize; i++) {
    Serial.println((output[i] + 1) / 2 * 255);
  }
  */

  neat->Calculate(input);

  output = neat->GetOutput();

  for(int i=0; i<strip.numPixels(); i++)
    //strip.setPixelColor(i, strip.Color(abs(output[i*3]) / 2.0 * 255, abs(output[i*3+1]) / 2.0 * 255, abs(output[i*3+2]) / 2.0 * 255));
    strip.setPixelColor(i, strip.gamma32(strip.ColorHSV((output[i] + 1) / 2.0 * 65536)));

  if (output[LED_COUNT] >= 0.75)
    timer = timerMax;
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(analogRead(0));
  pinMode(13, OUTPUT);

  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(brightness);

  for(int i=0; i<strip.numPixels(); i++)// For each pixel in strip...
    strip.setPixelColor(i, strip.Color(255,   0,   255));         //  Set pixel's color (in RAM)
  strip.show();                          //  Update strip to match

  
  neat = new Neat(INPUT_NODES, OUTPUT_NODES);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  double input1[] = {0.5};
  double input2[] = {0.1, -0.7, -0.8, 0.6, -0.1};

  neat->Calculate(input1);
  //neat->Print();

  double *output = neat->GetOutput();

  for (int i = 0; i < neat->outputSize; i++) {
    Serial.println(output[i]);
  }

  neat->Calculate(input2);

  output = neat->GetOutput();

  for (int i = 0; i < neat->outputSize; i++) {
    Serial.println(output[i]);
  }

  neat->Print();

  //if (output[0] > 0) digitalWrite(13, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  UpdateLights();

  bool updateBrightness = false;
  if (millis() - updateBrightness > 100) {
    updateBrightness = true;
    millisPrevBrightness = millis();
  }

  if (millis() - millisPrev > 1000) {
    timer ++;
    Serial.println(timer);
    millisPrev = millis();
  }

  if (timer >= timerMax) {
    if (updateBrightness) {
      brightness -= 2;
      strip.setBrightness(brightness);
      strip.show();
      Serial.println(brightness);

      if (brightness <= 0) {
        Serial.println("NEW NEAT");
        brightness = 0;
        strip.setBrightness(brightness);

        delete neat;
        neat = new Neat(INPUT_NODES, OUTPUT_NODES);

        timer = 0;
      }
    }
  }
  else {
    if (brightness < brightnessStart) {
      if (updateBrightness) {
        brightness += 1;
        strip.setBrightness(brightness);
        Serial.println(brightness);
      }
    }
  }

  //delay(1000000);
  //delay(100);
  //delay(5000);
  strip.show();
}
