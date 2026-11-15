const int fsrPin = A0;
const int numReadings = 5*200; // Number of samples per block

long total = 0;              // Running total for the current block
int sampleCount = 0;         // Tracks how many readings have been collected

void setup() {
  Serial.begin(9600);
  analogReadResolution(14); // 14-bit resolution for Uno R4
}

void loop() {
  // Add the current ADC value to the block total
  total += analogRead(fsrPin);
  sampleCount++;
  
  // Once we have collected 200 readings, calculate and print the average
  if (sampleCount >= numReadings) {
    long average = total / numReadings;
    
    Serial.print("Block Average ADC Value: ");
    Serial.println(average);
    
    // Reset variables for the next batch of 200
    total = 0;
    sampleCount = 0;
  }
  
  delay(10); // Small delay between samples
}