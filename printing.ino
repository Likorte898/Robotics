void error_printing(){
  while(1){
    reading();
    Serial.print(state);
    Serial.print("\t");
    Serial.print(active_lines);
    Serial.print("\t");
    for(int j = 0; j < 8; j++){
      Serial.print(E[j]);
      Serial.print("\t");
    }
    Serial.print(sum);
    Serial.print("\t");
    Serial.print(error);
    Serial.println("");
  }
}

void sensor_printing(){
  while(1){
    reading();
    for(int j = 0; j < 16; j++){
      Serial.print(s[j]);
      Serial.print("\t");
      Serial.print(k[j]);
      Serial.print(" ");
    }
    Serial.print(sum);
    Serial.println("");
  }
}

void analog_printing(){
  while(1){
    reading();
    Serial.print(state);
    Serial.print("\t");
    for(int j = 0; j < 16; j++){
      Serial.print(s[j]);
      Serial.print("\t");
    }
    Serial.print(sum);
    Serial.println("");
  }
}
