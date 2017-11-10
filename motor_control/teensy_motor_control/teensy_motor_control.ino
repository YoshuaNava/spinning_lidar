
// Setup:
void setup() {
    Serial.begin(9600);
}

// Loop:
void loop() {
    int time_now = micros();
    Serial.println("Hola!!!");
///    delay((DELAY - (micros() - time_now))/1000);
}
