void PID(){
  reading();
  while(sum == 0) reading();
  while(1){
    reading();
    if(sum == 0 && flag !='s'){
      digitalWrite(led, 1);
      (flag == 'r') ? error=8 : error = -8;
      p = error;
      d = previous_error;
      i = i + error;
      // kp=0.18*speed;

      previous_error = error;
      adj = (p*kp) + (i*ki) + (d*kd);
      motor(speed + adj, speed - adj);
      while (sum == 0) {
        reading();
        Serial.print("Turning ");
        (flag == 'r') ? Serial.println("Right "):Serial.println("Left ");
      }
      digitalWrite(led, 0);
    }
    else {
      // if(abs(error)<2) speed =255;
      // else speed =180;
      p = error;
      d = previous_error;
      i = i + error;

      // speed = (2-sqrt(1-pow(2, 1-abs(error/8))))*starting_speed/2;
      kp=0.125*speed;
      Serial.println(speed);

      previous_error = error;
      adj = (p*kp) + (i*ki) + (d*kd);
      motor(speed + adj, speed - adj);
    }
    // Serial.println(adj);
  }
}