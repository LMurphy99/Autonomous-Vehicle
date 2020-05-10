//move car

void locomtotion(float angle, float distance){
  
  //move by incoming angle degrees 
  if (angle <= 180){                      //angle 0-180 deg
    left(125);
    delay(5.56*angle);
  }
  else {                                  //angle above 180 deg
    right(125);
    delay(5.56*angle);
  }

  //move by incoming distance mm
  if (distance <= 160){
    if (distance < 80){
      forward(70);                          //distance below 80mm
      delay(distance*12.5);
      stopCar();
      return;
    }
    forward(((-80+distance)*0.6875)+70);    //distance between 80 & 160mm
    delay(1000);
    return;
  }
  if (distance > 2000){
    forward(255);                           //distance above 2000mm
    delay(distance*0.5);
    stopCar();
    return;
  }
  forward(((-160+distance)*0.071)+125);     //distance above 160mm & below 2000mm
  delay(1000);
  stopCar();
  return;
}

