#include <Wire.h>


void setup()
{
byte error=1, address=4;
int nDevices;
Wire.begin();
Wire.setClock(100000);
Wire.setClockStretchLimit(1500);
Serial.begin(115200);
Serial.println("\nI2C Scanner");
Serial.println("Scanning...");

nDevices = 0;

while (error != 0){
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  if (error==0){
    Serial.print("I2C device found at address 0x");
    if (address<16) 
      Serial.print("0");
    Serial.print(address,HEX);
    Serial.println(" !");
  }
  if (error==4){
    Serial.print("Unknow error at address 0x");
    if (address<16) 
        Serial.print("0");
    Serial.println(address,HEX);
  }
} 
Wire.beginTransmission(address);
Wire.write(2);             // sends value byte  
Wire.endTransmission();     // stop transmitting

}

byte val=0;

void loop()
{
byte error, address=4;
int nDevices;

  Wire.beginTransmission(address);
  Wire.write(val);             // sends value byte  
  
error=Wire.endTransmission();     // stop transmitting
Serial.print("TX:"); 
Serial.print(error);
Serial.print("\n"); 

  val++;        // increment value
  if(val == 64) // if reached 64th position (max)
  {
    val = 0;    // start over from lowest value
  }
  delay(200);
Serial.print(val);
Serial.print("\n");
}

