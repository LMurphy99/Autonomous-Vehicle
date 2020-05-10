  //Stop
 void stopCar(){
 digitalWrite(IN1, LOW);     //set left wheel to off
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, LOW);     //set right wheel to off
 digitalWrite(IN4, LOW);
 }
  
  //Forward
 void forward(int PWM_Value){
 analogWrite(ENA, PWM_Value); //speed for left wheel
 analogWrite(ENB, PWM_Value); //speed for right wheel
 digitalWrite(IN1, HIGH);     //set left wheel direction to forwards
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, HIGH);     //set right wheel direction to forwards
 digitalWrite(IN4, LOW);
 }
  
  //Backward
 void backward(int PWM_Value){
 analogWrite(ENA, PWM_Value); //speed for left wheel
 analogWrite(ENB, PWM_Value); //speed for right wheel
 digitalWrite(IN1, LOW);     //set left wheel direction to backwards
 digitalWrite(IN2, HIGH);
 digitalWrite(IN3, LOW);     //set right wheel direction to backwards
 digitalWrite(IN4, HIGH);
 }
  
  //Turn Left 
 void left(int PWM_Value){
 analogWrite(ENA, PWM_Value); //speed for left wheel
 analogWrite(ENB, PWM_Value); //speed for right wheel
 digitalWrite(IN1, LOW);     //set left wheel direction to backwards
 digitalWrite(IN2, HIGH);
 digitalWrite(IN3, HIGH);     //set right wheel direction to forwards
 digitalWrite(IN4, LOW);
 }
  
  //Turn Right
 void right(int PWM_Value){
 analogWrite(ENA, PWM_Value); //speed for left wheel
 analogWrite(ENB, PWM_Value); //speed for right wheel
 digitalWrite(IN1, HIGH);     //set left wheel direction to forwards
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, LOW);     //set right wheel direction to backwards
 digitalWrite(IN4, HIGH);
 }
