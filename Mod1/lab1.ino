#define LEDPIN 12
#define SPEAKERPIN 11
#define INPIN 2
#define ANALOGPIN 0
#define MIN_FREQ 100
#define MAX_FREQ 1000

int state = 0;
int pressed = 0;
int isOn = 0;
int potValue = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(LEDPIN, OUTPUT);
  pinMode(SPEAKERPIN, OUTPUT);
  pinMode(INPIN, INPUT);
}

void loop() {
   if (digitalRead(INPIN) == HIGH && pressed == 1) {
     if (state == 0) {
       state = 1;
     } else {
       state = 0;
     }
     pressed = 0;
   } else if (digitalRead(INPIN) == LOW) {
     pressed = 1;
   }
   
   if (state == 0) {
     noTone(SPEAKERPIN);
   } else {
     digitalWrite(LEDPIN, LOW);
   }
   
   potValue = analogRead(ANALOGPIN);
   
   if (state == 0) {
    if (isOn == 0) {
     digitalWrite(LEDPIN, HIGH);
     isOn = 1;
    } else {
     digitalWrite(LEDPIN, LOW);
     isOn = 0;
    }
    delay(potValue/1023.0 * 1000);
   } else {
    tone(SPEAKERPIN, (potValue/1023.0)*(MAX_FREQ - MIN_FREQ)+MIN_FREQ);
   }
}
