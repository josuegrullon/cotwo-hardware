#include <math.h> 

#define WindSensorPin (2) // The pin location of the anemometer sensor 
#define output (A7)
#define input (A0)
unsigned long Rotations; // cup rotation counter used in interrupt routine 
unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine 
float interval = 0;
float WindSpeed; // speed miles per hour 

void setup() { 
    Serial.begin(9600); 
    pinMode(WindSensorPin, INPUT);
    pinMode(output, OUTPUT);  
    pinMode(input, INPUT); 
    attachInterrupt(digitalPinToInterrupt(WindSensorPin), rotation, FALLING); 
} 

void loop() { 
    Rotations = 0; // Set Rotations count to 0 ready for calculations 

    sei(); // Enables interrupts 
    delay (1000);
    cli(); // Disable interrupts 
    

    interval = (float)(interval / 1000.00);
    float out = interval != 0 ? (2.4 / interval) : 0.00;
   
  
    Serial.print((int)out); Serial.print("\t\t");  Serial.print(out); Serial.print("\t\t"); Serial.print(analogRead(input)); Serial.print("\t\t"); 
    Serial.println(interval);
    analogWrite(out, WindSpeed);
    interval = 0;

} 

// This is the function that the interrupt calls to increment the rotation count 
void rotation () { 
    unsigned long current = millis();
    if ((current - ContactBounceTime) > 15 ) { // debounce the switch contact. 
        interval = (float)current - ContactBounceTime;
        ContactBounceTime = current;     
    }
}
