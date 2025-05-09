void error_calculation(){
  for(int i = 0;i<8;i++){
    E[i] = 0;
  }

  active_lines = 0;
  int i = 0;
  int indi = 0;
  for(int j = 0;j<8;j++){
    while(i<16 && k[i]==0) i++;
    if(i>15) break;
    while(k[i] == 1 && i<16) {
      E[active_lines]+=sensor_weight[i];
      indi++;
      i++;
    }
    E[active_lines] /= indi;
    if(E[active_lines]!=0) active_lines++;
    indi = 0;
  }

  if(active_lines == 0) error = 0;
  else if(active_lines == 1) error = E[0];
  else if(active_lines == 2) (side == 'r') ? error = E[1] : error = E[0];
  else if(active_lines%2==1) error = E[active_lines/2];
  else error = (E[active_lines/2]+E[active_lines/2-1])/2;
}