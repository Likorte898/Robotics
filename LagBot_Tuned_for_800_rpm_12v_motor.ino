#define S0 0  // Control pin S0
#define S1 1  // Control pin S1
#define S2 2  // Control pin S2

#define left_motor_forward 7
#define left_motor_backward 6
#define right_motor_forward 9
#define right_motor_backward 8
#define left_motor_speed 5
#define right_motor_speed 10

#define sw 21
#define light 20

#define sg1 4
#define sg2 3

#define led 20

// delay timers
int transition_time = 50;

int s[16], mid[16] = {1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1500}, k[16], active_lines;
int maximum[16], minimum[16];
float sensor_weight[16] = {-8, -8, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 8, 8};
int sum;
char side = 'r';
char flag = 's';

float speed=255;
float starting_speed = 230;

//PID
float kp=0.125*speed, p, kd, d, ki, i, adj;

float E[8], error, previous_error;

int state = 0, temp_sum;

void setup() {
  Serial.begin(115200);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
    
  pinMode(led, OUTPUT);
  pinMode(sw, INPUT);

  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);

  pinMode(left_motor_speed, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(right_motor_speed, OUTPUT);
  delay(2000);
  motor(255, 255);
  delay(2000);
  motor(0, 0);
    
}

void loop() {
  int r = button_read();
  if(r==1) PID();
  else if(r==2) sensor_printing();
  else if(r==3) error_printing();
  else if(r==4) analog_printing();
  else if(r==5) cal();

}

