void setup() {
  Serial.begin(9600);
  
  int temperature = 32;
  int humidity = 60;
  float pm10 = 10.75;
  float pm25 = 8.037;
  char * commandeSigfox = "";
  
  commandeSigfox = prepareForSigfox(temperature, humidity, pm10, pm25);
  Serial.println(commandeSigfox);

}

void loop() {
}

char * prepareForSigfox(int temperature, int humidity, int pm10, int pm25) {
  char dataString[50] = {0};
  sprintf(dataString, "AT$SF=%02X%02X%02X%02X\r",temperature, humidity, pm10, pm25);
  return dataString;
}

// Woohoo


