int DIR_A = 8;
int PWM_A = 9;
int DIR_B = 7;
int PWM_B = 6;

void setup() {
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    int commaIndex = data.indexOf(',');
    if (commaIndex < 0) {
      return;
    }
    int speedAC = data.substring(0, commaIndex).toInt();
    int speedBD = data.substring(commaIndex + 1).toInt();
   
    if (speedAC >= 0) {
      digitalWrite(DIR_A, HIGH);
      analogWrite(PWM_A, speedAC);    
    } else {
      // Reverse
      digitalWrite(DIR_A, LOW);
      analogWrite(PWM_A, abs(speedAC));
    }

    if (speedBD >= 0) {
      digitalWrite(DIR_B, LOW);
      analogWrite(PWM_B, speedBD);    
    } else {
      digitalWrite(DIR_B, HIGH);
      analogWrite(PWM_B, abs(speedBD));  
    }
  }
}