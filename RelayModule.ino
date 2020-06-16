#include "debug.h"
#include "Arduino.h"
#include "./libraries/MsTimer2/MsTimer2.cpp"
//*****************
#include "relayModule.h"
float vout;
float pressure;
float Ipressure;
float Epressure;
float PIP;
float PLAT;


void setup() {
  VENT_DEBUG_FUNC_START();
  Serial.begin(115200);     // The Serial port of Arduino baud rate.
  Serial.println(F("Signum Techniks"));           // say hello to check serial line

  Serial3.begin(115200);

  pinMode(INDICATOR_LED, OUTPUT);

  pinMode(INHALE_SYNC_PIN, OUTPUT);
  digitalWrite(INHALE_SYNC_PIN, HIGH);
  pinMode(EXHALE_SYNC_PIN, OUTPUT);
  digitalWrite(EXHALE_SYNC_PIN, LOW);

  //Stepper Motor step and direction
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  digitalWrite(MOTOR_STEP_PIN, HIGH);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);

  //motor controls & Home pins
  //pinMode(MOTOR_RUN_PIN, INPUT_PULLUP);
  pinMode(HOME_SENSOR_PIN, INPUT_PULLUP);
  //digitalWrite(MOTOR_RUN_PIN, HIGH);
  digitalWrite(HOME_SENSOR_PIN, HIGH);

  //Valves
  pinMode(EXHALE_VLV_PIN, OUTPUT);
  pinMode(INHALE_VLV_PIN, OUTPUT);
  pinMode(O2Cyl_VLV_PIN, OUTPUT);
  pinMode(O2Hln_VLV_PIN, OUTPUT);
  pinMode(INHALE_RELEASE_VLV_PIN, OUTPUT);
  //Valves Pin Initialize
  digitalWrite(EXHALE_VLV_PIN, LOW);
  digitalWrite(INHALE_VLV_PIN, LOW);
  digitalWrite(O2Cyl_VLV_PIN, LOW);
  digitalWrite(O2Hln_VLV_PIN, LOW);
  digitalWrite(INHALE_RELEASE_VLV_PIN, LOW);


  INHALE_EXHALE_SYNC_PIN_OFF();  //DIGITAL PIN SYNC
  inti_all_Valves();
  //stop_timer();


  //home cycle on power up
  home_cycle = true;
  motion_profile_count_temp = 0;
  VENT_DEBUG_ERROR("Power On Home Cycle : ", 0);
  run_pulse_count_temp = 0.0;
  run_pulse_count = 200000.0;
  digitalWrite(MOTOR_DIR_PIN, EXP_DIR);
  //This is mandatory to initate the Timer block properly
  initialize_timer1_for_set_RPM(home_speed_value * 10.0);
  run_motor = true;

  delay(5000);
  flag_Serial_requested = true;
  VENT_DEBUG_ERROR("Requesting paramemters : $VSP10001&", 0);
  Serial3.print("$VSP10001&");
  VENT_DEBUG_FUNC_END();
}



void loop() {
  VENT_DEBUG_FUNC_START();
//#if GP_connected
//  //delay(10);
//  vout = analogRead(INHALE_GAUGE_PRESSURE) * 0.0048828125;
//  pressure = ((vout - ((0.05 * 0.0)) - (5 * 0.04)) / (5 * 0.09));
//  Ipressure = ((pressure - 0.07) / 0.09075);
//  //Serial.print("Inhale :"); Serial.print(Ipressure); 
//  
//  vout = analogRead(EXHALE_GUAGE_PRESSURE) * 0.0048828125;
//  pressure = ((vout - ((0.05 * 0.0)) - (5 * 0.04)) / (5 * 0.09));
//  Epressure = ((pressure - 0.07) / 0.09075);
//  //Serial.print("   Exhale:"); Serial.println(Epressure); 
//#endif

  //Expansion completed & Compression start
  if ((cycle_start == true) && (exp_start == true) && (exp_end == true) && (exp_timer_end == true)) {
    EXHALE_VLV_CLOSE();
    //Serial.print("PEEP:"); Serial.println(Epressure); 
    INHALE_VLV_OPEN();
    VENT_DEBUG_ERROR("IER: 1:", IER); 
    VENT_DEBUG_ERROR("  BPM: ", BPM);  
    VENT_DEBUG_ERROR("  TV: ", tidal_volume); 
    VENT_DEBUG_ERROR("  Stroke: ", Stroke_length);
    VENT_DEBUG_ERROR("comp : ", (c_end_millis - c_start_millis) / 1000.0);  
    VENT_DEBUG_ERROR("/", inhale_time);
    VENT_DEBUG_ERROR("  ExpTime : ", (e_timer_end_millis - e_start_millis) / 1000.0); 
    VENT_DEBUG_ERROR("/", exhale_time); 
    VENT_DEBUG_ERROR("  Cycle : ", (e_timer_end_millis - c_start_millis) / 1000.0);  
    VENT_DEBUG_ERROR("/", cycle_time);
    VENT_DEBUG_ERROR("Inhale-hold : ", inhale_hold_time / 1000.0);  
    VENT_DEBUG_ERROR("  MotorRet. : ", (e_end_millis - e_start_millis) / 1000.0);
    if ((BPM_new != BPM) || (tidal_volume_new != tidal_volume) || (IER_new != IER)) {
      convert_all_set_params_2_machine_values();
    }
    Start_inhale_cycle();
  }

  //compression started & is in progress
  if ((cycle_start == true) && (comp_start == true) && (comp_end == false)) {
//       if(Ipressure > 40.0) {
//          INHALE_VLV_CLOSE();
//          //Stop motor
//          Emergency_motor_stop = true;
//       }
  }

  //Compression completed & start Expansion
  if ((cycle_start == true) && (comp_start == true) && (comp_end == true)) {
    Start_exhale_cycle();
    inhale_hold_time = (inhale_time * (inhale_hold_percentage / 100)) * 1000;
    delay(inhale_hold_time); //expansion delay
    //Serial.print("PLAT:"); Serial.println(Epressure); 
    EXHALE_VLV_OPEN();
    //Start_exhale_cycle();
  }

  //Expansion started & is in progress
  if ((cycle_start == true) && (exp_start == true) && (exp_end == false)) {  }
  VENT_DEBUG_FUNC_END();
}






ISR(TIMER1_COMPA_vect) { //timer1 interrupt 1Hz toggles pin 13 (LED)
  VENT_DEBUG_FUNC_START();
  //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  if (run_motor == true) {
    if ((motion_profile_count_temp == 0) && (run_pulse_count_temp == 0.0)) {
      //compression cycle start only once
      if ((comp_start == true) & (comp_end == false)) {
        //Serial.print("comp: "); Serial.println(motion_profile_count_temp);
        c_start_millis = millis();
        run_pulse_count = compression_step_array[motion_profile_count_temp];
        digitalWrite(MOTOR_DIR_PIN, COMP_DIR);
        OCR1A = OCR1A_comp_array[motion_profile_count_temp] ;
        load_TCCR1B_var(TCCR1B_comp_array[motion_profile_count_temp]);
        Emergency_motor_stop = false;
        INHALE_RELEASE_VLV_CLOSE();
        EXHALE_VLV_CLOSE();
        INHALE_VLV_OPEN();
      }

      //after inhale-hold time --> Expansion cycle start only once
      if ((exp_start == true) & (exp_end == false)) {
        //Serial.print("exp: "); Serial.println(motion_profile_count_temp);
        e_start_millis = millis();
        run_pulse_count = expansion_step_array[motion_profile_count_temp];
        digitalWrite(MOTOR_DIR_PIN, EXP_DIR);
        OCR1A = OCR1A_exp_array[motion_profile_count_temp] ;
        load_TCCR1B_var(TCCR1B_exp_array[motion_profile_count_temp]) ;
        Emergency_motor_stop = false;
        INHALE_RELEASE_VLV_CLOSE();
        INHALE_VLV_CLOSE();
        //Commented to check if i can open the Valve after hold time.. so hold time and motor retraction will have overlap.
        //EXHALE_VLV_OPEN();
      }
    }

    //Actual motor pulse generation block
    if (run_pulse_count_temp < run_pulse_count) {
      if (Emergency_motor_stop == false) digitalWrite(MOTOR_STEP_PIN, digitalRead(MOTOR_STEP_PIN) ^ 1);
      run_pulse_count_temp = run_pulse_count_temp + 0.5;
      if (home_cycle == true) {
        if (digitalRead(HOME_SENSOR_PIN) == HOME_SENSE_VALUE) {
          run_motor = false;
          run_pulse_count_temp = 0.0;
          home_cycle = false;
          motion_profile_count_temp = 0;
          VENT_DEBUG_ERROR("Home Cycle Complete...", 0);
          if (cycle_start == true) inti_Start();
        }
      }
      if ((cycle_start == true) && (digitalRead(MOTOR_DIR_PIN) == EXP_DIR)) {
        if (digitalRead(HOME_SENSOR_PIN) == HOME_SENSE_VALUE) {
          run_pulse_count_temp = run_pulse_count;
          motion_profile_count_temp = CURVE_EXP_STEPS;
          Emergency_motor_stop = true;
          //motion_profile_count_temp = 0;
          //run_pulse_count_temp = 0.0;
        }
      }
    } else {
      noInterrupts();
      run_motor = false;
      run_pulse_count_temp = 0.0;
      motion_profile_count_temp = motion_profile_count_temp + 1;
      //Compression end check
      if ((comp_start == true) & (comp_end == false)) {
        if (motion_profile_count_temp < CURVE_COMP_STEPS) {
          //Serial.print("comp: "); Serial.println(motion_profile_count_temp);
          run_pulse_count = compression_step_array[motion_profile_count_temp];
          digitalWrite(MOTOR_DIR_PIN, COMP_DIR);
          OCR1A = OCR1A_comp_array[motion_profile_count_temp] ;
          load_TCCR1B_var(TCCR1B_comp_array[motion_profile_count_temp]) ;
          run_motor = true;
        } else {
          c_end_millis = millis();
          motion_profile_count_temp = 0;
          run_pulse_count_temp = 0.0;
          Emergency_motor_stop = false;
          //Serial.print("PIP:"); Serial.println(Ipressure); 
          INHALE_RELEASE_VLV_CLOSE();
          INHALE_VLV_CLOSE();
          //commented as this will be Opened after Inhale-Hold Delay.
          //EXHALE_VLV_OPEN();
          comp_end = true;
        }
      }
      //expansion end check
      if ((exp_start == true) & (exp_end == false)) {
        if (motion_profile_count_temp < CURVE_EXP_STEPS) {
          //Serial.print("exp: "); Serial.println(motion_profile_count_temp);
          run_pulse_count = expansion_step_array[motion_profile_count_temp];
          digitalWrite(MOTOR_DIR_PIN, EXP_DIR);
          OCR1A = OCR1A_exp_array[motion_profile_count_temp] ;
          load_TCCR1B_var(TCCR1B_exp_array[motion_profile_count_temp]) ;
          run_motor = true;
        } else {
          e_end_millis = millis();
          motion_profile_count_temp = 0;
          run_pulse_count_temp = 0.0;
          Emergency_motor_stop = false;
          INHALE_RELEASE_VLV_CLOSE();
          //these are skipped to enable motor fast retract and wait for timer to close the exhale valve
          //          EXHALE_VLV_CLOSE();
          //          INHALE_VLV_OPEN();
          exp_end = true;
        }
      }
      interrupts();
    }
  }
  VENT_DEBUG_FUNC_END();
}

boolean Start_exhale_cycle() {
  VENT_DEBUG_FUNC_START();
  Serial3.print("$VSSY0003&");   //expansion flag
  //Serial.print("CYCLE Exhale Time: " );Serial.println(exhale_time);
  MsTimer2::set(exhale_time * 1000, Exhale_timer_timout); //period
  MsTimer2::start();
  EXHALE_SYNC_PIN_ON();  //DIGITAL PIN SYNC
  digitalWrite(INHALE_SYNC_PIN, LOW); 
  digitalWrite(EXHALE_SYNC_PIN, HIGH);

  cycle_start = true;
  comp_start = false;
  comp_end = false;
  exp_start = true;
  exp_end = false;
  exp_timer_end = false;
  run_motor = true;
  VENT_DEBUG_FUNC_END();
  return true;
}

boolean Start_inhale_cycle() {
  VENT_DEBUG_FUNC_START();
  VENT_DEBUG_ERROR("$VSSY0001&", 0);//comp start flag
  Serial3.print("$VSSY0001&");   
  INHALE_SYNC_PIN_ON();  //DIGITAL PIN SYNC
  digitalWrite(INHALE_SYNC_PIN, HIGH); digitalWrite(EXHALE_SYNC_PIN, LOW);
  cycle_start = true;
  comp_start = true;
  comp_end = false;
  exp_start = false;
  exp_end = false;
  exp_timer_end = false;
  run_motor = true;
  VENT_DEBUG_FUNC_END();
  return true;
}

boolean Exhale_timer_timout() {
  VENT_DEBUG_FUNC_START();
  MsTimer2::stop();
  digitalWrite(INDICATOR_LED, digitalRead(INDICATOR_LED) ^ 1);
  e_timer_end_millis = millis();
  EXHALE_VLV_CLOSE();
  INHALE_VLV_OPEN();
  exp_timer_end = true;
  VENT_DEBUG_FUNC_END();
  return true;
}

void load_TCCR1B_var(int TCCR1B_var_temp) {
  if (TCCR1B_var_temp == 1)    {
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  }
  if (TCCR1B_var_temp == 8)    {
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  }
  if (TCCR1B_var_temp == 64)   {
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  }
  if (TCCR1B_var_temp == 256)  {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  }
  if (TCCR1B_var_temp == 1024) {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
  }

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
}

boolean convert_all_set_params_2_machine_values() {
  VENT_DEBUG_FUNC_START();
  VENT_DEBUG_ERROR("Speed curve calculations : ", 0);

  BPM = BPM_new;
  tidal_volume = tidal_volume_new;
  Stroke_length = Stroke_length_new;
  inhale_ratio = 1.0;
  IER = IER_new; exhale_ratio = IER_new;

  //global variable
  cycle_time = 60.0 / BPM;
  inhale_time = (cycle_time * inhale_ratio) / (inhale_ratio + exhale_ratio);
  exhale_time = (cycle_time * exhale_ratio) / (inhale_ratio + exhale_ratio);
  //  exhale_time = exhale_time - (inhale_hold_time/1000);

  MsTimer2::set(exhale_time * 1000, Exhale_timer_timout); //period
  VENT_DEBUG_ERROR("Calculated Exhale Time: ", exhale_time );
 
  float inhale_vpeak = ((Stroke_length * 0.8) / (inhale_time * 0.8)) ;
  float exhale_vpeak = ((Stroke_length * 0.8) / (0.90 * 0.8));  //exhale_time

  compression_speed = ( inhale_vpeak / LEAD_SCREW_PITCH ) * 60;
  expansion_speed =   ( exhale_vpeak / LEAD_SCREW_PITCH ) * 60;

  //this 1.8 degree step motor so 200 steps for 360 degree
  run_pulse_count_1_full_movement = ((micro_stepping * (Stroke_length / LEAD_SCREW_PITCH * 1.0)) / 2.0);
  run_pulse_count_1_piece_compression = (run_pulse_count_1_full_movement / 100.0);   //CURVE_COMP_STEPS   '''taking 100 pieces to ease the % calculation
  run_pulse_count_1_piece_expansion = (run_pulse_count_1_full_movement / 100.0);     //CURVE_EXP_STEPS  '''taking 100 pieces to ease the % calculation

  //avoid accel/deccel below 300 rpm
  if (compression_speed > MIN_RPM_NO_ACCEL) {
    compression_speed = compression_speed + (compression_speed * 0.08);
    compression_min_speed = MIN_RPM_NO_ACCEL;
  } else compression_min_speed = compression_speed;
  if (expansion_speed > MIN_RPM_NO_ACCEL) {
    expansion_speed = expansion_speed + (expansion_speed * 0.08);
    expansion_min_speed = MIN_RPM_NO_ACCEL;
  } else expansion_min_speed = expansion_speed;

  compression_step_array[10] = run_pulse_count_1_piece_compression * 80;
  compression_speed_array[10] = compression_speed;
  pre_calculate_timer_values_4_different_RPM(compression_speed_array[10] * 10.0);
  TCCR1B_comp_array[10] = TCCR1B_var;
  OCR1A_comp_array[10] = OCR1A_var;

  expansion_step_array[10] = run_pulse_count_1_piece_expansion * 80;
  expansion_speed_array[10] = expansion_speed;
  pre_calculate_timer_values_4_different_RPM(expansion_speed_array[10] * 10.0);
  TCCR1B_exp_array[10] = TCCR1B_var;
  OCR1A_exp_array[10] = OCR1A_var;

  float comp_slope = (compression_speed - compression_min_speed) / 8.0;
  float exp_slope = (expansion_speed - expansion_min_speed) / 8.0;
  int i;
  for (i = 0; i < 10; i++) {
    compression_step_array[i] = run_pulse_count_1_piece_compression;
    compression_step_array[20 - i] = run_pulse_count_1_piece_compression;
    compression_speed_array[i] = (comp_slope * ((Stroke_length * 0.01) * (i ) )) + compression_min_speed;
    compression_speed_array[20 - i] = (comp_slope * ((Stroke_length * 0.01) * (i ) )) + compression_min_speed;

    //compression_speed_array[i] = compression_min_speed + (i * ((compression_speed - compression_min_speed) / 10.0));
    //compression_speed_array[20 - i] = compression_min_speed + (i * ((compression_speed - compression_min_speed) / 10.0));

    pre_calculate_timer_values_4_different_RPM(compression_speed_array[i] * 10.0);
    TCCR1B_comp_array[i] = TCCR1B_var;
    OCR1A_comp_array[i] = OCR1A_var;
    TCCR1B_comp_array[20 - i] = TCCR1B_var;
    OCR1A_comp_array[20 - i] = OCR1A_var;

    expansion_step_array[i] = run_pulse_count_1_piece_expansion;
    expansion_step_array[20 - i] = run_pulse_count_1_piece_expansion;
    expansion_speed_array[i] = (exp_slope * ((Stroke_length * 0.01) * (i ))) + expansion_min_speed;
    expansion_speed_array[20 - i] = (exp_slope * ((Stroke_length * 0.01) * (i ))) + expansion_min_speed;

    //expansion_speed_array[i] = expansion_min_speed + (i * ((expansion_speed - expansion_min_speed) / 10.0));
    //expansion_speed_array[20 - i] = expansion_min_speed + (i * ((expansion_speed - expansion_min_speed) / 10.0));

    pre_calculate_timer_values_4_different_RPM(expansion_speed_array[i] * 10.0);
    TCCR1B_exp_array[i] = TCCR1B_var;
    OCR1A_exp_array[i] = OCR1A_var;
    TCCR1B_exp_array[20 - i] = TCCR1B_var;
    OCR1A_exp_array[20 - i] = OCR1A_var;
  }

  //  for (i = 0; i < 21; i++) {
  //    Serial.print("Compression: "); Serial.print(i); Serial.print(" | step: "); Serial.print(compression_step_array[i]); Serial.print(" | rpm: "); Serial.println(compression_speed_array[i]);
  //    Serial.print("expansion  : "); Serial.print(i); Serial.print(" | step: "); Serial.print(expansion_step_array[i]);  Serial.print(" | rpm: "); Serial.println(expansion_speed_array[i]);
  //  }
  VENT_DEBUG_FUNC_END();
  return true;
}

boolean stop_timer() {
  //cli();  // One way to disable the timer, and all interrupts
  VENT_DEBUG_FUNC_START();
  TCCR1B &= ~(1 << CS12); // turn off the clock altogether
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);

  TIMSK1 &= ~(1 << OCIE1A); // turn off the timer interrupt
  VENT_DEBUG_FUNC_END();
  return true;
}

void pre_calculate_timer_values_4_different_RPM(float rpm) {
  VENT_DEBUG_FUNC_START();
  double freq;
  //  long OCR1A_var;
  //  long TCCR1B_var;

  rpm = abs(rpm);
  freq = round( long((rpm * micro_stepping) / 600.0));
  //Serial.print(("rpm  : ")); Serial.print(rpm / 10); Serial.print(("  Freq : ")); Serial.print(freq); Serial.print(("  Micro stepping: ")); Serial.println(micro_stepping);

  // initialize timer1
  //noInterrupts(); // disable all interrupts
  //set timer1 interrupt at 1Hz
  //TCCR1A = 0;// set entire TCCR1A register to 0
  //TCCR1B = 0;// same for TCCR1B
  //TCNT1  = 0;//initialize counter value to 0

  // set compare match register for 1hz increments
  TCCR1B_var = 1;
  OCR1A_var = (16000000.0 / (freq * TCCR1B_var)) - 1; // (must be <65536)
  //  Serial.println("1 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0 ) {
    TCCR1B_var = 8;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("2 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 64;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1; // (must be <65536)
  }
  //  Serial.println("3 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 256;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("4 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 1024;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("5 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);
  VENT_DEBUG_FUNC_END();
}

boolean initialize_timer1_for_set_RPM(float rpm) {
  VENT_DEBUG_FUNC_START();
  double freq;
  //  long OCR1A_var;
  //  long TCCR1B_var;

  rpm = abs(rpm);
  //  Serial.print(("rpm  : ")); Serial.println(rpm / 10);
  //  Serial.print(("Micro: ")); Serial.println(micro_stepping);
  freq = round( long((rpm * micro_stepping) / 600.0));
  //  Serial.print(("Freq : ")); Serial.println(freq);


  // initialize timer1
  noInterrupts(); // disable all interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0

  // set compare match register for 1hz increments
  TCCR1B_var = 1;
  OCR1A_var = (16000000.0 / (freq * TCCR1B_var)) - 1; // (must be <65536)
  //  Serial.println("1 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);
  if (OCR1A_var > 65536 || OCR1A_var <= 0 ) {
    TCCR1B_var = 8;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("2 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);
  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 64;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1; // (must be <65536)
  }
  //  Serial.println("3 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);
  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 256;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("4 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);
  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 1024;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("5 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  OCR1A = OCR1A_var;

  // Set CS12, CS11 and CS10 bits for X prescaler
  //001 = 1
  //010 = 8
  //011 = 64
  //100 = 256
  //101 = 1024
  if (TCCR1B_var == 1)    {
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  }
  if (TCCR1B_var == 8)    {
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  }
  if (TCCR1B_var == 64)   {
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  }
  if (TCCR1B_var == 256)  {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  }
  if (TCCR1B_var == 1024) {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
  }

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  interrupts(); // enable all interrupts
  VENT_DEBUG_FUNC_END();
}
