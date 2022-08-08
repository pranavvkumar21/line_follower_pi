//#include "Arduino.h"
#include "encoders.h"

#define LS_left 18
#define LS_center 20
#define LS_right 21
#define emit 11
#define NB_LS_PINS 3
#define time_elapsed 5000
#define L_PWM 10
#define R_PWM 9
#define L_DIR 16
#define R_DIR 15
#define sped 20
#define turn_speed 200
#define INITIAL_STATE 0
#define LINE_FOUND 1

int state, white_flag;
unsigned long w_delay, is_end, t, dt;
static float phi_l, phi_r, radius = 1.6, X_i, Y_i, theta_i, X_rt, Y_rt, theta_rt, length = 9.6, rot_vel_l, rot_vel_r, rot_vel_demand;
float p_pwm_l, p_pwm_r, i_pwm_l, i_pwm_r, d_pwm_l, d_pwm_r, err_lprev = 0.0, err_rprev = 0.0, err_l = 0.0, err_r = 0.0;
long count_prev0, count_prev1;
float kp = 10, ki = .01, kd = 10;
void setup() {
  setupEncoder0();
  setupEncoder1();
  state = 0;
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(emit, OUTPUT);
  pinMode(LS_left, INPUT);
  pinMode(LS_center, INPUT);
  pinMode(LS_right, INPUT);
  digitalWrite(emit, HIGH);
  radius = 1.6;
  length = 9.6;
  X_i = 0.0;
  Y_i = 0.0;
  i_pwm_l = 0.0;
  i_pwm_r = 0.0;
  theta_i = 0.0;
  rot_vel_demand = 2.0;
  err_l = 0.0;
  err_r = 0.0;
  count_prev1 = count_e1;
  count_prev0 = count_e0;
  X_i = Y_i = theta_i = 0;
  Serial.begin(9600);
  delay(1000);
  Serial.println("Set up completed");
}

unsigned long* read_ls() {

  int ls_pin[NB_LS_PINS] = {LS_left, LS_center, LS_right};
  int which;
  static unsigned long start_time, end_time[NB_LS_PINS], elapsed_time_LS[NB_LS_PINS];
  boolean done = false;


  pinMode(LS_left, OUTPUT);
  digitalWrite(LS_left, HIGH);
  pinMode(LS_center, OUTPUT);
  digitalWrite(LS_center, HIGH);
  pinMode(LS_right, OUTPUT);
  digitalWrite(LS_right, HIGH);

  delayMicroseconds(10);

  pinMode(LS_left, INPUT);
  pinMode(LS_center, INPUT);
  pinMode(LS_right, INPUT);

  bool flag[3] = {1, 1, 1};
  start_time = micros();

  while (done == false)
  {
    for (which = 0; which < NB_LS_PINS; which++)
    {
      if (digitalRead(ls_pin[which]) == LOW && flag[which] != 0)
      { flag[which] = 0;
        end_time[which] = micros();
        elapsed_time_LS[which] = end_time[which] - start_time;
      }
    }
    if ((flag[0] + flag[1] + flag[2]) == 0) {

      done = true;
      return elapsed_time_LS;
    }
  }
}

float get_error(unsigned long *elapsed_time_LS) {
  float sum;
  sum = elapsed_time_LS[0] + elapsed_time_LS[1] + elapsed_time_LS[2];
  float norm[3] = {elapsed_time_LS[0] / sum, elapsed_time_LS[1] / sum, elapsed_time_LS[2] / sum};
  float w_l, w_r, e_L;
  //Serial.println(norm[1]);
  w_l = norm[0] + norm[1] * 0.5;
  w_r = norm[2] + norm[1] * 0.5;
  e_L = w_l - w_r;
  //Serial.println(e_L);
  return e_L;
  
}

void run(float err) {
  float turn = turn_speed * (err);
  int lvalue = sped - turn, rvalue = sped + turn;

  if (lvalue < 0) lvalue = 0;
  if (rvalue < 0) rvalue = 0;
  analogWrite( L_PWM, lvalue);
  analogWrite( R_PWM, rvalue);
}
void stop1() {
  analogWrite( L_PWM, 0);
  analogWrite( R_PWM, 0);
}
void velocity() {
  dt = millis() - t;
  t += dt;
  rot_vel_l = (count_e1 - count_prev1) / dt;
  rot_vel_r = (count_e0 - count_prev0) / dt;
  count_prev1 = count_e1;
  count_prev0 = count_e0;

}

void forward_kinematics() {
  phi_l = ((count_e1 - count_prev1) * (2 * 3.14 / 358.3));
  phi_r = ((count_e0 - count_prev0) * (2 * 3.14 / 358.3));
  Serial.print("X_i: ");
  Serial.println(theta_rt);
  Serial.print("theta_i: ");
  Serial.println(theta_i);
  count_prev1 = count_e1;
  count_prev0 = count_e0;

  X_rt = (radius / 2) * (phi_l + phi_r);
  theta_rt = (radius / (2 * length)) * (phi_l - phi_r);
  theta_rt = theta_rt * 180 / 3.14;
  X_i = X_i + X_rt * cos(theta_i);
  Y_i += X_rt * sin(theta_i);
  theta_i += theta_rt;
}
void pid() {
  err_l = rot_vel_demand - rot_vel_l;
  err_r = rot_vel_demand - rot_vel_r;
  p_pwm_l = kp * err_l;
  p_pwm_r = kp * err_r;
  Serial.println(err_l * dt);
  i_pwm_l += dt * err_l;
  //i_pwm_r += ki*err_r*dt;
  d_pwm_l = kd * (err_l - err_lprev) / dt;
  d_pwm_r = kd * (err_r - err_rprev) / dt;
  err_lprev = err_l;
  err_rprev = err_r;
  float pwm_l = p_pwm_l + ki * i_pwm_l + d_pwm_l;
  float pwm_r = p_pwm_r + ki * i_pwm_r + d_pwm_r;
  Serial.println(i_pwm_l);
  digitalWrite(L_DIR, LOW);
  digitalWrite(R_DIR, LOW);
  analogWrite( L_PWM, pwm_l);
  analogWrite( R_PWM, pwm_r);
}

void loop() {
  //Serial.println(state);
  unsigned long *elapsed_time_LS;
  elapsed_time_LS = read_ls();
  float err = get_error(elapsed_time_LS);
  if (state == 0) {
    delay(2000);
    count_prev0 = count_e0;
    count_prev1 = count_e1;
    t = millis();
    is_end = millis();
    run(0);
    while (elapsed_time_LS[1] < 1000) {
      elapsed_time_LS = read_ls();
      //forward_kinematics();
    }
    run(.3);
    //forward_kinematics();
    delay(250);
    state = 1;

  }

  if (state == 1) {
    elapsed_time_LS = read_ls();
    err = get_error(elapsed_time_LS);
    run(err);
    //forward_kinematics();
    if (elapsed_time_LS[1] < 1000 && white_flag != 1) {
      white_flag = 1;
      w_delay = millis();
    elapsed_time_LS = read_ls();
    err = get_error(elapsed_time_LS);
    run(err);
    }
    if (elapsed_time_LS[1] > 1000 && white_flag == 1) {
      white_flag = 0;
    }
    if (elapsed_time_LS[1] < 1000 && white_flag == 1) {
      if ((millis() - w_delay) > 1000) {
        if ((millis() - is_end) > 10000) {
          state = 3;
        }
        else {
          state=2;
        }
      }
    }
  }
  if (state == 2) {
    elapsed_time_LS = read_ls();
    bool flag2 = 0;
    while (elapsed_time_LS[1] < 1000) {
      if (flag2 == 0) {
        digitalWrite(R_DIR, HIGH);
        analogWrite(L_PWM, 50);
        analogWrite(R_PWM, 50);
        delay(500);
        digitalWrite(R_DIR, LOW);
        analogWrite(L_PWM, 20);
        analogWrite(R_PWM, 20);
        flag2 = 1;
      }
      elapsed_time_LS = read_ls();
      state = 1;

    }
  }
  if (state == 3) {
    stop1();
  }
}
