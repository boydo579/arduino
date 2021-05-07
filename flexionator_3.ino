// **Enter [0] into serial input to see menu options**
//Tools > serial monitor [ctrl+shift+M]

const int flexPin = A0;       // Pin connected to voltage divider output
const byte VCC = 5;           // voltage at Ardunio 5V line
const float R_DIV = 46200.0;  // resistor used to create a voltage divider
//const byte set0_pin= 7;     //pin connected to buttons 
//const byte set90_pin= 8;
float sensorRES=15200;
float sensorADC=0;
float sensorVOLT=0;
long startTime=0;
int instruction;
byte bin;
byte i=0;
byte v=0;


//varible values
float EXTflat_resistance = 15000;   // resistance when flat, flatResistance
float FLEXbend_resistance = 40000;  // resistance at 90 deg, bendResistance
int setmax_EXT=0;
int setmax_FLEX=90;
float oldAngle;
float angle=0;
float sendAngle;
float dis;
float velocity;
int max_v=0;

void setup() {
  Serial.begin(9600);
  pinMode(flexPin, INPUT);
  //pinMode(set0_pin, INPUT_PULLUP);
  //pinMode(set90_pin, INPUT_PULLUP);

}

void loop() {

  calc_sensorRES();     //Read in ADC, calc sensor resistance value
  map_Sensor();          //output angle
  calc_velc();          //calculate velocity
  menu();               //enter 0 to trigger this function
  //checkButtons();       //check buttons for range of motion setting
}

void calc_sensorRES()  {
  sensorADC = analogRead(flexPin);
  sensorVOLT = sensorADC * VCC / 1023.0;     //calculate output voltage
  sensorRES = R_DIV * (VCC / sensorVOLT - 1.0);  //calculate resistance of the flex sensor
}


void map_Sensor() {
  //estimate sensor bend angle: map(target, begincompare, endcompare, low_angle(0), high_angle(90))  
  oldAngle=angle;
  startTime = millis();
  angle = map(sensorRES, EXTflat_resistance, FLEXbend_resistance, setmax_EXT, setmax_FLEX);
  
  //denoising
  if (((angle > (2+oldAngle)) || angle < (oldAngle-2))){
    //Serial.println("Angle: "+String(angle));
    Serial.print("Angle: ");  
    Serial.println(angle); 
  }
  else{
    angle=oldAngle;
    Serial.println("Angle: "+String(angle));
  }
  delay(100);
}


void calc_velc(){
  long currentTime = millis();
  long elapsedTime = currentTime - startTime;
  dis = angle - oldAngle;
  velocity = dis/elapsedTime * 100;
  //Serial.println("Velocity: " + String(velocity) + " degrees/second \n");  
  Serial.println(velocity);   
  Serial.println(" degrees/second \n");
    if (max_v > velocity){
      max_v = velocity;
      //max_v_ext
      //if max ext < than prevext then max=prev
    }  
  }


void menu() {
  while (Serial.available()) {
  //Serial.println("Max flex is " +String(setmax_FLEX)+"\nMax extension is "+ String(setmax_EXT)+"\n");
  instruction = Serial.parseInt();
  delay(2000);
  while (instruction == 0){
  Serial.println("Menu:\n1 [max_v] report max velocity.");        // \n is new line
  Serial.println("2 [ext]  calibrate sensor for max angles.");
  Serial.println("3 [flex] calibrate sensor for max angles.");
  Serial.println("4 [setmax] change max angles for mapping.");
  //Serial.println("5 [exit] to exit the menu.");
  instruction = Serial.parseInt();
    Serial.println("menu instr: " +String(instruction));
    
    switch (instruction) {
      case 1:         // max velocity
        Serial.println(":REPORT:\nMax velocity: "+String(max_v)+" degrees/sec");
        Serial.println("Angle at this velocity is "+String(angle)+" degrees\n");
        max_v=0;
        delay(3000);
        break;
      
      case 2:         // calibrate
        EXTflat_resistance = sensorRES;
        Serial.println("extenstion resistance: "+String(sensorRES));
        delay(100); //debounce
        break;

      case 3:
        FLEXbend_resistance = sensorRES;
        Serial.println("flexion resistance: "+String(sensorRES));
        delay(100);
        break;
        
      case 4:
        Serial.println("Enter flex angle to set flexion maximum.");
        setmax_FLEX = Serial.parseInt();
        Serial.println("Enter ext angle to set extension maximum.");
        setmax_EXT = Serial.parseInt();
        delay(100);
        break;
      
    }   //switch end 
  }   //if end
  
  
   
  }   //while end
}   //menu end

// old button code --> upgraded to serial input
/*void checkButtons() {
  buttonState_0 = digitalRead(set0_pin);
  if (buttonState_0 == HIGH) {  // button pressed = HIGH:
    EXTflat_resistance = sensorRES;         //set 0 degree marker and notify  
    Serial.println("\n 0.0 degree bend marker set \n");
    delay(1000); //debouncing
  }
  buttonState_90 = digitalRead(set90_pin); 
  if (buttonState_90 == HIGH) {
    FLEXbend_resistance = sensorRES;           //set 90 degree marker and notify
    Serial.println("\n 90 degree bend marker set \n");
    delay(1000);
  }
  //#todo
  //buttonState_5 = digitalRead(record_button); 
}
*/
