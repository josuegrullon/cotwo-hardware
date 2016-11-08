void setup()
{
pinMode(A0,OUTPUT);
Serial.begin(9600);
}
void loop(){
Serial.println(analogRead(A0));

delay(100);

}
