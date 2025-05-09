void write(int channel) {
    digitalWrite(S0, channel & 0x01);
    digitalWrite(S1, (channel >> 1) & 0x01);
    digitalWrite(S2, (channel >> 2) & 0x01);

    delayMicroseconds(300);
}

void work(){
  write(0);
  s[1] = analogRead(sg2);
  s[10] = analogRead(sg1);
  write(1);
  s[7] = analogRead(sg2);
  s[12] = analogRead(sg1);
  write(2);
  s[3] = analogRead(sg2);
  s[8] = analogRead(sg1);
  write(3);
  s[6] = analogRead(sg2);
  s[13] = analogRead(sg1);
  write(4);
  s[2] = analogRead(sg2);
  s[9] = analogRead(sg1);
  write(5);
  s[4] = analogRead(sg2);
  s[15] = analogRead(sg1);
  write(6);
  s[0] = analogRead(sg2);
  s[11] = analogRead(sg1);
  write(7);
  s[5] = analogRead(sg2);
  s[14] = analogRead(sg1);

  (s[5] < mid[5]) ? k[5]=0: k[5]=1;
  (s[10] < mid[10]) ? k[10]=0: k[10]=1;
  temp_sum = k[5] + k[10];
}

void reading(){
  work();
  sum = 0;
  if(state == 1 && temp_sum == 0) {
    int m=millis();
    while(temp_sum==0) {
      work();
      Serial.println(millis()-m);
      if(millis()-m>transition_time) {
        state = 0;
        break;
      }
    }  
  }
  else if(state == 0 && temp_sum==2) {
    int m=millis();
    while(temp_sum==2) {
      work();
      Serial.println(millis()-m);
      if(millis()-m>transition_time) {
        state = 1;
        break;
      }
    } 
  }
  digitalWrite(led, state);
  if(state == 0) {
    for(int i=0;i<16;i++){
      (s[i]<mid[i]) ? k[i]=0 : k[i]=1;
      sum+=k[i];
    }
  } else {
    for(int i=0;i<16;i++){
      (s[i]>mid[i]) ? k[i]=0 : k[i]=1;
      sum+=k[i];
    }
  } 
  if(k[1]==1 && k[14]==0) flag = 'l';
  else if(k[1]==0 && k[14]==1) flag = 'r'; 

  error_calculation();

}

void cal(){
  digitalWrite(led, 1);
  for(int i=0;i<16;i++){
    maximum[i] = 0;
    minimum[i] = 4095;
  }
  motor(-70, 70);

  for(int i=0;i<5000;i++){
    work();
    for(int j=0;j<16;j++){
      maximum[j] = max(maximum[j], s[j]);
      minimum[j] = min(minimum[j], s[j]);
    }
  }

  motor(0, 0);
  

  for(int i=0;i<16;i++){
    mid[i] = (minimum[i] + maximum[i])/2;
  }
  digitalWrite(led, 0);
}
