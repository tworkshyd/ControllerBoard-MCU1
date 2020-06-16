//#include "Variables.h"
#include "relayModule.h"
#include "debug.h"

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '$') {
      comcnt = 1;
      rxdata = "";
    }
    if  (comcnt >= 1) {
      rxdata += inChar;
      comcnt = comcnt + 1;
      if (inChar == '&') {
        if (comcnt >= 10) {
          Serial.print(rxdata);
          Prcs_RxData();
        }
      }
    }
  }
}

void serialEvent3() {
  while (Serial3.available()) {
    char inChar = (char)Serial3.read();
    if (inChar == '$') {
      comcnt = 1;
      rxdata = "";
    }
    if  (comcnt >= 1) {
      rxdata += inChar;
      comcnt = comcnt + 1;
      if (inChar == '&') {
        if (comcnt >= 10) {
          Serial.println(rxdata);
          Prcs_RxData();
        }
      }
    }
  }
}



boolean Prcs_RxData() {
  String p1;
  String p2;
  String p3;
  String p4;
  String payload;
  VENT_DEBUG_FUNC_START();
  p1 = rxdata.substring(1, 3);
  p2 = rxdata.substring(3, 5);
  p3 = rxdata.substring(5, 7);
  p4 = rxdata.substring(7, 9);
  payload = p3 + p4;


  if (p1 == "VM") {
    if (p2 == "ST") {
      //stepper motor
      if (payload == "0000") {
        inti_Stop_n_Home();
      }
    } else if (p2 == "IN") {
      if (payload == "0000") {
        //if (cycle_start == false)
        inti_Home_n_Start();
      }
      if (payload == "0001") {
        if (cycle_start == true) inti_Stop_n_Home();
      }
      if (payload == "0003") {
        if ((cycle_start == true) && (digitalRead(HOME_SENSOR_PIN) == HOME_SENSE_VALUE)) breathe_detected_skip_exhale_n_start_inhale();
      }
      if (payload == "0002") {
        inti_all_Valves();
      }
    } else if (p2 == "PP") {
      if (payload == "0000") {
        Par_editstat = 1;
      }
      if (payload == "1111") {
        Par_editstat = 0;
      }
    }
    else if (p2 == "P1") {
      tidal_volume_new = payload.toInt();
      VENT_DEBUG_ERROR("TV : ", tidal_volume_new);
      
      //      if (tidal_volume_new == 50) Stroke_length_new = 16;
      //      if (tidal_volume_new == 100) Stroke_length_new = 24;
      //      if (tidal_volume_new == 150) Stroke_length_new = 31;
      if (tidal_volume_new == 200) Stroke_length_new = 50;
      if (tidal_volume_new == 250) Stroke_length_new = 56;
      if (tidal_volume_new == 300) Stroke_length_new = 61.5;
      if (tidal_volume_new == 350) Stroke_length_new = 66.70;
      if (tidal_volume_new == 400) Stroke_length_new = 71.0;
      if (tidal_volume_new == 450) Stroke_length_new = 75.5;
      if (tidal_volume_new == 500) Stroke_length_new = 79.5;
      if (tidal_volume_new == 550) Stroke_length_new = 83.3;
      if (tidal_volume_new == 600) Stroke_length_new = 87.5;
      if (tidal_volume_new == 650) Stroke_length_new = 92.8;
      if (tidal_volume_new == 700) Stroke_length_new = 98;
      //      if (tidal_volume_new == 750) Stroke_length_new = 86;
      //      if (tidal_volume_new == 800) Stroke_length_new = 90;
      //      if (tidal_volume_new == 850) Stroke_length_new = 95;
      //      if (tidal_volume_new == 900) Stroke_length_new = 100;
      //      if (tidal_volume_new == 950) Stroke_length_new = 105;

      //Stroke_length_new=tidal_volume_new/10;
      VENT_DEBUG_ERROR("SL : ", Stroke_length_new);
      if (flag_Serial_requested == true) {
        Serial3.print("$VSP20002&");
      }

    }
    else if (p2 == "P2") {
      BPM_new = payload.toInt();
      VENT_DEBUG_ERROR("BPM : ", BPM_new);
      cycle_time = 60.0 / BPM_new;
      VENT_DEBUG_ERROR("cycle time : ", cycle_time);
      inhale_hold_time = (cycle_time * (inhale_hold_percentage / 100)) * 1000;
      VENT_DEBUG_ERROR("Compression hold in mS: ", inhale_hold_time);
      if (flag_Serial_requested == true) {
        Serial3.print("$VSP50004&");
      }
    }
    else if (p2 == "P3") {
      peak_prsur = payload.toInt();
      //Serial.print("Peak_prsur : "); Serial.println(peak_prsur);
    }
    else if (p2 == "P4") {
      FiO2 = payload.toInt();
      //Serial.print("FiO2 : "); Serial.println(FiO2);
    }
    else if (p2 == "P5") {
      IER_new = payload.toInt();
      //Serial.print("IER : "); Serial.println(IER_new);
      //      IER = 1020;
      //      inhale_ratio = 1.0;
      //      exhale_ratio = 2.0;
      if (flag_Serial_requested == true) {
        flag_Serial_requested = false;
        convert_all_set_params_2_machine_values();
        Serial3.print("$VSO20000&");
      }
    }
    else if (p2 == "P6") {
      PEEP_new = payload.toInt();
      //Serial.print("PEEP_new : "); Serial.println(PEEP_new);
    }
    else if (p2 == "SV") {
      if (p3 == "01") {
        if (p4 == "00") {
          //digitalWrite(INHALE_VLV_PIN, LOW);
          INHALE_VLV_CLOSE();
          //Stop motor
          if ((cycle_start == true) && (comp_start == true) && (comp_end == false)) Emergency_motor_stop = true;
          //relief valve ON
          //INHALE_RELEASE_VLV_OPEN();
        } else if (p4 == "01") {
          //digitalWrite(INHALE_VLV_PIN, HIGH);
          INHALE_VLV_OPEN();
        }
      } else if (p3 == "02") {
        if (p4 == "00") {
          //digitalWrite(EXHALE_VLV_PIN, LOW);
          EXHALE_VLV_CLOSE();
        } else if (p4 == "01") {
          //digitalWrite(EXHALE_VLV_PIN, HIGH);
          EXHALE_VLV_OPEN();
        }
      } else if (p3 == "03") {
        if (p4 == "00") {
          INHALE_RELEASE_VLV_CLOSE();
        } else if (p4 == "01") {
          INHALE_RELEASE_VLV_OPEN();
        }
      }
    } else if (p2 == "O2") {   //solanoide valve for Oxygen line
      if (p3 == "01") {
        if (p4 == "00") {
          //digitalWrite(O2Cyl_VLV_PIN, LOW);
          VENT_DEBUG_ERROR("2Hln_VLV SELECTED ", 0);
          O2_line_option = 1;
          if (cycle_start == true) {
            O2Cyl_VLV_CLOSE();
            O2Hln_VLV_OPEN();
          }
        } else if (p4 == "01") {
          //digitalWrite(O2Cyl_VLV_PIN, HIGH);
          VENT_DEBUG_ERROR("O2Cyl_VLV SELECTED ", 0);
          O2_line_option = 0;
          if (cycle_start == true) {
            O2Cyl_VLV_OPEN();
            O2Hln_VLV_CLOSE();
          }
        }
      } else if (p3 == "02") {
        if (p4 == "00") {
          //digitalWrite(O2Hln_VLV_PIN, LOW);
          VENT_DEBUG_ERROR("O2Cyl_VLV SELECTED ", 0);
          O2_line_option = 0;
          if (cycle_start == true) {
            O2Hln_VLV_CLOSE();
            O2Cyl_VLV_OPEN();
          }
        } else if (p4 == "01") {
          //digitalWrite(O2Hln_VLV_PIN, HIGH);
          VENT_DEBUG_ERROR("2Hln_VLV SELECTED ", 0);
          O2_line_option = 1;
          if (cycle_start == true) {
            O2Hln_VLV_OPEN();
            O2Cyl_VLV_CLOSE();
          }
        }
      }
    }
  }
  VENT_DEBUG_FUNC_END();
  return true;
}

boolean open_selected_O2_value(void) {
  VENT_DEBUG_FUNC_START();
  if (O2_line_option == 0) {
    VENT_DEBUG_ERROR("O2Cyl_VLV Opened... ", 0);
    O2Hln_VLV_CLOSE();
    O2Cyl_VLV_OPEN();
  } else
  {
    VENT_DEBUG_ERROR("2Hln_VLV Opened... ", 0);
    O2Hln_VLV_OPEN();
    O2Cyl_VLV_CLOSE();
  }
  VENT_DEBUG_FUNC_END();
}

boolean inti_all_Valves(void) {
  VENT_DEBUG_FUNC_START();
  //Normally Opened
  EXHALE_VLV_OPEN();
  INHALE_VLV_OPEN();

  //Normally closed
  INHALE_RELEASE_VLV_CLOSE();
  O2Cyl_VLV_CLOSE();
  O2Hln_VLV_CLOSE();
  VENT_DEBUG_FUNC_END();
  return true;
}


boolean breathe_detected_skip_exhale_n_start_inhale() {
  VENT_DEBUG_FUNC_START();
  Emergency_motor_stop = false;
  VENT_DEBUG_ERROR("Skipping Home Cycle : ", 0);
  home_cycle = false;
  cycle_start = true;
  comp_start = false;
  comp_end = false;
  exp_start = true;
  exp_end = true;
  //exp_timer_end = true;
  Exhale_timer_timout();
  //run_motor = true
  VENT_DEBUG_FUNC_END();
  return true;
}

boolean inti_Stop_n_Home() {
  VENT_DEBUG_FUNC_START();
  cycle_start = false;
  Emergency_motor_stop = false;
  run_motor = true;
  Exhale_timer_timout();
  VENT_DEBUG_ERROR("Cycle Stop & goto Home : ", 0);
  run_pulse_count = 200000;
  digitalWrite(MOTOR_DIR_PIN, EXP_DIR);
  initialize_timer1_for_set_RPM(home_speed_value * 10.0);
  comp_start = false;
  comp_end = false;
  exp_start = false;
  exp_end = false;
  home_cycle = true;
  cycle_start = false;
  run_motor = true;
  INHALE_EXHALE_SYNC_PIN_OFF();  //DIGITAL PIN SYNC
  inti_all_Valves();
  VENT_DEBUG_FUNC_END();
  return true;
}

boolean inti_Home_n_Start() {
  VENT_DEBUG_FUNC_START();
  Emergency_motor_stop = false;
  motion_profile_count_temp = 0;
  run_pulse_count_temp = 0.0;
  if (digitalRead(HOME_SENSOR_PIN) == !(HOME_SENSE_VALUE))
  {
    VENT_DEBUG_ERROR("Home Cycle : ", 0);
    run_pulse_count = 200000;
    digitalWrite(MOTOR_DIR_PIN, EXP_DIR);
    initialize_timer1_for_set_RPM(home_speed_value * 10.0);
    comp_start = false;
    comp_end = false;
    exp_start = false;
    exp_end = false;
    home_cycle = true;
    run_motor = true;
    delay(200);
    cycle_start = true;
    Serial3.print("$VSSY0000&");
  } else {
    inti_Start();
  }
  VENT_DEBUG_FUNC_END();
  return true;
}

boolean inti_Start() {
  VENT_DEBUG_FUNC_START();
  convert_all_set_params_2_machine_values();
  open_selected_O2_value();
  Emergency_motor_stop = false;
  VENT_DEBUG_ERROR("Skipping Home Cycle : ", 0);
  home_cycle = false;
  cycle_start = true;
  comp_start = false;
  comp_end = false;
  exp_start = true;
  exp_end = true;
  //exp_timer_end = true;
  Exhale_timer_timout();
  //run_motor = true
  VENT_DEBUG_FUNC_END();
  return true;
}
