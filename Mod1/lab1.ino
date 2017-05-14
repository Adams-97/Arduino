//Description

/*
When you press the button the LED will start blinking and when pressed again the Piezo will turn on
*/




#define LED1 13
#define SPEAKERPIN 11
#define BUTTON 7
#define analogPin 3    
#define MIN_FREQ 100
#define MAX_FREQ 1000
// potentiometer wiper (middle terminal) connected to A3, NOT 3 in the digital I/O

int flag = 0;
int val = 0;           // variable to store the value read

void setup(){
  pinMode(LED1,OUTPUT);
  pinMode(BUTTON,INPUT);
  pinMode(SPEAKERPIN, OUTPUT);
  
  Serial.begin(9600);          //  setup serial

  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);

  while(digitalRead(BUTTON)){}
  delay(50);
  while(!digitalRead(BUTTON)){}
}

void loop(){
  if(flag==0){
  val = analogRead(analogPin);    // read the input pin
  Serial.println(val);             // debug value
    
    digitalWrite(LED1,HIGH);
    delay(val);
    digitalWrite(LED1,LOW);
    delay(val);
    
    if(!digitalRead(BUTTON)){
      while(!digitalRead(BUTTON)){}
      flag = 1;
      digitalWrite(LED1,LOW);
      delay(50);
      }
  }
    else if(flag==1){
    tone(SPEAKERPIN, (potValue/1023.0)*(MAX_FREQ - MIN_FREQ)+MIN_FREQ);
    if(!digitalRead(BUTTON)){
      while(!digitalRead(BUTTON)){}
      flag = 0;
         noTone(SPEAKERPIN);
      delay(50);
      }
  }
  
  
  }

