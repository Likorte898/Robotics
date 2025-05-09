void setup()
{
  pinMode(13, OUTPUT); // Motor A +
  pinMode(12, OUTPUT); // Motor A-
  pinMode(11, OUTPUT); // Enable 1&2
  pinMode(10, OUTPUT); // Enable 3&4 
  pinMode (9, OUTPUT); // Motor B -
  pinMode (8, OUTPUT); // Motor B + 
  pinMode(6, INPUT);   // L1
  pinMode(5, INPUT);   // L2
  pinMode(4, INPUT);   // M
  pinMode(3, INPUT);   // R2
  pinMode(2, INPUT);   // R1
}

void loop()
{
  if((digitalRead(6 )== 1) && (digitalRead(5) == 1) && (digitalRead(4) == 0) && (digitalRead(3)== 1) && (digitalRead(3) == 1));
  {forward();} // calling functions
  if((digitalRead(6 )== 1) && (digitalRead(5) == 1) && (digitalRead(4) == 1) && (digitalRead(3)== 1) && (digitalRead(3) == 1));
  {backward();}
  if((digitalRead(6 )== 1) && (digitalRead(5) == 0) && (digitalRead(4) == 0) && (digitalRead(3)== 1) && (digitalRead(3) == 1));
  {left_s();}
  if((digitalRead(6 )== 0) && (digitalRead(5) == 0) && (digitalRead(4) == 0) && (digitalRead(3)== 1) && (digitalRead(3) == 1));
  {left_f();}
  if((digitalRead(6 )== 1) && (digitalRead(5) == 0) && (digitalRead(4) == 1) && (digitalRead(3)== 1) && (digitalRead(3) == 1));
  {left();}
  if((digitalRead(6 )== 1) && (digitalRead(5) == 1) && (digitalRead(4) == 0) && (digitalRead(3)== 0) && (digitalRead(3) == 1));
  {right_s();}
  if((digitalRead(6 )== 1) && (digitalRead(5) == 1) && (digitalRead(4) == 0) && (digitalRead(3)== 0) && (digitalRead(3) == 0));
  {right_f();}
  if((digitalRead(6 )== 1) && (digitalRead(5) == 1) && (digitalRead(4) == 1) && (digitalRead(3)== 0) && (digitalRead(3) == 1));
  {right();}
  if((digitalRead(6 )== 0) && (digitalRead(5) == 0) && (digitalRead(4) == 0) && (digitalRead(3)== 0) && (digitalRead(3) == 0));
  {stop();}
}

// Declaring Functions
void forward()
{
  digitalWrite(13, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  analogWrite(11, 80);
  analogWrite(10, 80);
  
}
void backward()
{
  digitalWrite(13, LOW);
  digitalWrite(12, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  analogWrite(11, 60);
  analogWrite(10, 60);
}
void left_s()
{
  digitalWrite(13, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  analogWrite(11, 70);
  analogWrite(10, 60);
}
void left_f()
{
  digitalWrite(13, LOW);
  digitalWrite(12, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  analogWrite(11, 60);
  analogWrite(10, 60);
}
void left()
{
  digitalWrite(13, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  analogWrite(11, 70);
  analogWrite(10, 50);
}
void right_s()
{
  digitalWrite(13, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  analogWrite(11, 60);
  analogWrite(10, 70);
}
void right_f()
{
  digitalWrite(13, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(9, HIGH);
  digitalWrite(8, LOW);
  analogWrite(11, 60);
  analogWrite(10, 60);
}
void right()
{
  digitalWrite(13, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  analogWrite(11, 50);
  analogWrite(10, 70);
}
void stop()
{
  digitalWrite(13, LOW);
  digitalWrite(12, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);
  analogWrite(11, 0);
  analogWrite(10, 0);
}
