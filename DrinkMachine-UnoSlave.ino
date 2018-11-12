#define LS_LOW 2
#define LS_MED 3
#define LS_HIGH 4
#define TEST_BTN 5
#define LS_OUT A0

void setup() {
  pinMode(LS_LOW, INPUT);
  pinMode(LS_MED, INPUT);
  pinMode(LS_HIGH, INPUT);
  pinMode(TEST_BTN, INPUT_PULLUP);
  pinMode(LS_OUT,OUTPUT);
}

//check input with debouncing, I assume all the switch is active low (DRY CONTACT)
bool digitalReadBounce(int pin) {
  bool result = LOW;
  if(digitalRead(pin)==LOW){
    delay(50);
    if(digitalRead(pin)==LOW)
      result = HIGH;
  }
  return result;  
}

void loop() {
  if(digitalRead(LS_LOW)==HIGH){
    if(digitalRead(LS_MED)==HIGH){
      if(digitalRead(LS_HIGH)==HIGH)
        analogWrite(LS_OUT,255);
      else
        analogWrite(LS_OUT,175);
    }
    else{
      analogWrite(LS_OUT,100);
    }
  }
  else if(digitalReadBounce(TEST_BTN)==HIGH){
    analogWrite(LS_OUT,50);
  }
  else{  
    analogWrite(LS_OUT,0);
  }
}
