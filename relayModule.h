/**************************************************************************/
/*!
    @file     Variables.h

    @brief    Ventilator Driver Module

    @author   Tworks

    @defgroup VentilatorModule  VentilatorModule

    Module to initialize and deinitialize the ventilator
  @{
*/
/**************************************************************************/
#ifndef _relayModule_H
#define _relayModule_H


#define COMP_DIR HIGH             /*!< Defines the signal at Direction pin of motor driver for CCW rotation */
#define EXP_DIR LOW               /*!< Defines the signal at Direction pin of motor driver for CW rotation */
#define LEAD_SCREW_PITCH 8.0      /*!< Lead screw pitch is 8.0 mm, i.e when we rotate it by 360 degree then the linear travel will be 8.0 mm in a direction */

#define CURVE_COMP_STEPS 21       /*!< During inhale motor forward motion is broken in 21 pieces to achieve trapozoidal motion profile */
#define CURVE_EXP_STEPS 21        /*!< During exhale motor return motion is broken in 21 pieces to achieve trapozoidal motion profile */
#define MIN_RPM_NO_ACCEL 250.0    /*!< This setting removes accel/decel in motion profile below mentioned RPM */
#define HOME_SENSE_VALUE  LOW  //0 for PNP(alway high)   //1 for NPN (alway low)

//A0-A15  D54 to D69
#define INHALE_SYNC_PIN 66        /*!< A12 Ouput for digital sync for exhale */
#define EXHALE_SYNC_PIN 67        /*!< A13 Ouput for digital sync for exhale */
#define MOTOR_STEP_PIN  68        /*!< A14 Ouput for motor steps to control position & velocity */
#define MOTOR_DIR_PIN   69        /*!< A15 Ouput for motor direction control CW/CCW */
#define HOME_SENSOR_PIN  57       /*!< A3 Input for detecting home position for Moving assembly */
#define INHALE_RELEASE_VLV_PIN 10 /*!< Ouput for inhale high pressure release line solenoid On/Off */
#define INHALE_VLV_PIN 64         /*!<A10 Ouput for Inhale line solenoid On/Off */
#define EXHALE_VLV_PIN 65         /*!< A11 Ouput for Exhale line solenoid On/Off */
#define O2Cyl_VLV_PIN 11          /*!< Ouput for Cylinder Oxygen line solenoid On/Off */
#define O2Hln_VLV_PIN 12          /*!< Ouput for Hospital Oxygen line solenoid On/Off */
#define INDICATOR_LED 13          /*!< Ouput for Indicating which cycle is in progress. ON=Compression  & OFF=Expansion */
#define INHALE_GAUGE_PRESSURE 61 
#define EXHALE_GUAGE_PRESSURE 60

/*!< Digital sync pins for other board */
#define INHALE_SYNC_PIN_ON() {digitalWrite(INHALE_SYNC_PIN, HIGH);digitalWrite(EXHALE_SYNC_PIN, LOW);}       /*!< Digital sync PINs inhale=1 & exhale=0 */
#define EXHALE_SYNC_PIN_ON() {digitalWrite(INHALE_SYNC_PIN, LOW); digitalWrite(EXHALE_SYNC_PIN, HIGH);}      /*!< Digital sync PINs inhale=0 & exhale=1 */
#define INHALE_EXHALE_SYNC_PIN_OFF() {digitalWrite(INHALE_SYNC_PIN, LOW);digitalWrite(EXHALE_SYNC_PIN, LOW);}/*!< Digital sync PINs inhale=0 & exhale=0 */

/*!< Normally Opened Valves */
#define EXHALE_VLV_OPEN()  digitalWrite(EXHALE_VLV_PIN, LOW)    /*!< Exhale line solenoid valve Open */
#define EXHALE_VLV_CLOSE() digitalWrite(EXHALE_VLV_PIN, HIGH)   /*!< Exhale line solenoid valve Close */
#define INHALE_VLV_OPEN()  digitalWrite(INHALE_VLV_PIN, LOW)    /*!< Inhale line solenoid valve Open */
#define INHALE_VLV_CLOSE() digitalWrite(INHALE_VLV_PIN, HIGH)   /*!< Inhale line solenoid valve Close */

/*!< Normally Closed Valves */
#define INHALE_RELEASE_VLV_OPEN()  digitalWrite(INHALE_RELEASE_VLV_PIN, HIGH)       /*!< Inhale line high pressure release solenoid valve Open */
#define INHALE_RELEASE_VLV_CLOSE() digitalWrite(INHALE_RELEASE_VLV_PIN, LOW)        /*!< Inhale line high pressure release solenoid valve Close */
#define O2Cyl_VLV_OPEN()  digitalWrite(O2Cyl_VLV_PIN, HIGH)                         /*!< Cylinder Oxygen line solenoid valve Open */
#define O2Cyl_VLV_CLOSE() digitalWrite(O2Cyl_VLV_PIN, LOW)                          /*!< Cylinder Oxygen line solenoid valve Close */
#define O2Hln_VLV_OPEN()  digitalWrite(O2Hln_VLV_PIN, HIGH)                         /*!< Hospital Oxygen line solenoid valve Open */
#define O2Hln_VLV_CLOSE() digitalWrite(O2Hln_VLV_PIN, LOW)                          /*!< Hospital Oxygen line solenoid valve Close */

volatile byte O2_line_option = 0;  //0 = O2 cylinder, 1 = O2 Hosp line & 3 = both off
volatile bool flag_Serial_requested = false;
volatile float compression_min_speed = MIN_RPM_NO_ACCEL;  /*!< This setting removes accel/decel in motion profile below mentioned RPM */
volatile float expansion_min_speed = MIN_RPM_NO_ACCEL;    /*!< This setting removes accel/decel in motion profile below mentioned RPM */

volatile float micro_stepping =800.0;                                      /*!< Stepper motor mcro stepping setting, i.e. number of pulses need to rotate shaft by 360 degree */
volatile float run_pulse_count_1_full_movement = micro_stepping;            /*!< Based on micro stepping for 360 degree, we need 800 pulses */
volatile float run_pulse_count_1_piece_compression = micro_stepping / 100;  /*!< Pulses needed for 1 piece of motion profile */
volatile float run_pulse_count_1_piece_expansion = micro_stepping / 100;    /*!< Pulses needed for 1 piece of motion profile */

volatile float home_speed_value = 200.0;    /*!< Homing RPM default setting */
volatile float compression_speed = 300.0;   /*!< Compression RPM default setting */
volatile float expansion_speed = 200.0;     /*!< Expansion RPM default setting */

//this machanisum has 1:2 ratio adjusted formulas to reflect below data after belt ratio
volatile int Par_editstat;                  /*!< Machine State used to run machine in Auto mode when UI goes in Parameter edit mode */
volatile float cycle_time;                  /*!< Calculated cycle time based on BPM setting */
volatile float inhale_time;                 /*!< Calculated inhale time based on BPM & IER setting */
volatile float exhale_time;                 /*!< Calculated exhale time based on BPM & IER setting */
volatile float inhale_hold_time = 0.5;      /*!< Calculated inhale hold time based on inhale_hold_percentage & ihale_time setting */
volatile float inhale_hold_percentage = 10; /*!< Set Percentage Value for inhale hold time calculation */


volatile float tidal_volume = 350.0;        /*!< Machine runtime Tidal Volume setting to calcualte equivalent Stroke length to compress */
volatile float BPM = 10.0;                  /*!< Machine runtime BPM setting to calculate cycle time */
volatile float peak_prsur = 12.0;           /*!< Machine runtime PIP/PeakPressure/PAW */
volatile float FiO2 = 21.0;                 /*!< Machine runtime Air+Oxygen Percentage on inhale Line */
volatile float IER = 2.0;                   /*!< Machine runtime inhale:exhale ratio */
volatile float inhale_ratio = 1.0;          /*!< Machine runtime inale ratio part is always 1.0 for us */
volatile float exhale_ratio = 2.0;          /*!< Machine runtime exhale ration parts varies from 1 to 3 */
volatile float Stroke_length = 75.0;        /*!< Machine runtime Auto calculated Stroke length based on Tidal Volume lookup table created based on our calibration.*/
volatile float PEEP = 20.0;                 /*!< Machine runtime minimum PEEP value */

volatile float tidal_volume_new = 350.0;    /*!< User setable Tidal Volume setting to calcualte equivalent Stroke length to compress */
volatile float BPM_new = 10.0;              /*!< User setable BPM setting to calculate cycle time */
volatile float IER_new = 2.0;               /*!< User setable inhale:exhale ratio */
volatile float inhale_ratio_new = 1.0;      /*!< User setable inale ratio part is always 1.0 for us */
volatile float exhale_ratio_new = 2.0;      /*!< User setable exhale ration parts varies from 1 to 3 */
volatile float Stroke_length_new = 75.0;    /*!< Auto calculated Stroke length based on Tidal Volume lookup table created based on our calibration.*/
volatile float PEEP_new = 20.0;             /*!< User setable minimum PEEP value */


volatile long OCR1A_var;                        /*!< Timer OCRxA register value for a required frequency */
volatile long TCCR1B_var;                       /*!< Timer TCCRxB register value for a required frequency */
volatile float run_pulse_count = 0.0;           /*!< Run time calcualted pulses value */
volatile float run_pulse_count_temp = 0.0;      /*!< Run time pulses counter */
volatile long motion_profile_count_temp = 0;    /*!< Tapoziodal motion profile Piece counter */
volatile float compression_step_array[21];      /*!< inhale pulse count for one piece of motion profile */
volatile float expansion_step_array[21];        /*!< exhale pulse count for one piece of motion profile */
volatile float compression_speed_array[21];     /*!< inhale velocity for one piece of motion profile */
volatile float expansion_speed_array[21];       /*!< exhale velocity for one piece of motion profile */
volatile long TCCR1B_comp_array[21];            /*!< inhale Timer TCCRxB register value for one piece of motion profile */
volatile long OCR1A_comp_array[21];             /*!< inhale Timer OCRxA register value for one piece of motion profile */
volatile long TCCR1B_exp_array[21];             /*!< exhale Timer TCCRxB register value for one piece of motion profile */
volatile long OCR1A_exp_array[21];              /*!< exhale Timer OCRxA register value for one piece of motion profile */

long c_start_millis = 0;      /*!< Timer millis value for inhale start */
long c_end_millis = 0;        /*!< Timer millis value for inhale stop */
long e_start_millis = 0;      /*!< Timer millis value for exhale start */
long e_end_millis = 0;        /*!< Timer millis value for exhale stop */
long e_timer_end_millis = 0;  /*!< Timer millis value for exhale timer timeout end/stop */

volatile boolean Emergency_motor_stop = false;    /*!< Flag to stop sending further pulses to motor if value is true */
volatile boolean run_motor = false;               /*!< Flag to start the exhale or inhale motion profile */
volatile boolean cycle_start = false;             /*!< Flag to start/stop the machine */
volatile boolean home_cycle = false;              /*!< Flag to start homing cycle */

volatile boolean comp_start = false;              /*!< Flag to start inhale cycle */
volatile boolean comp_end = false;                /*!< Flag to indicates end of inhale cycle */
volatile boolean exp_start = false;               /*!< Flag to start inhale cycle */
volatile boolean exp_end = false;                 /*!< Flag to indicates end of exhale cycle */
volatile boolean exp_timer_end = false;           /*!< Flag to indicate exhale timer timeout */


int comcnt = 0;                                   /*!< counter to count serial recieved bytes */
String rxdata = "";                               /*!< string to store serial recieved data  */


/**************************************************************************/
/*!

    @brief  Function to convert all set parameter values to machine runtime values to execute the compression and expansion cycles.

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean convert_all_set_params_2_machine_values(void);

/**************************************************************************/
/*!

    @brief  Function to initialize Exhale cycle on the ventilator

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean Start_exhale_cycle(void);

/**************************************************************************/
/*!

    @brief  Function to initialize Inhale cycle on the ventilator

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean Start_inhale_cycle(void);

/**************************************************************************/
/*!

    @brief  Function to initialize Inhale cycle skipping the current exhale cycle as there is deman of breadth detected.

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean breathe_detected_skip_exhale_n_start_inhale(void);

/**************************************************************************/
/*!

    @brief  Function to stop the exhal time and set the exhale_timer_end flag, so that we can start inhale cycle.

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean Exhale_timer_timout(void);

/**************************************************************************/
/*!

    @brief  Function to start homing and initialize/Start the ventilator by setting the Cycle_Start = true..

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean inti_Home_n_Start(void);

/**************************************************************************/
/*!

    @brief  Function to Deinitialize/Stop the ventilator by setting the Cycle_Start = false..

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean inti_Stop_n_Home(void);


/**************************************************************************/
/*!

    @brief  Function to initilaze the timer to generate the square pulses for stepper motor to achieve perticulare RPM.

    @param rpm paramter to configure the rotation per minute for pushing assembly

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean initialize_timer1_for_set_RPM(float rpm);

/**************************************************************************/
/*!

    @brief  Function to Stop the Timer1 and stop all the pulses generate.

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean stop_timer(void);


/**************************************************************************/
/*!

    @brief  Function to process the serial command after complete packet is recieved and perform the action.

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean Prcs_RxData(void);

/**************************************************************************/
/*!

    @brief  Function to open selected O2 valve when machine starts.

    @return indicates 0 for SUCCESS and 1 for FAILURE
*/
/**************************************************************************/
boolean open_selected_O2_value(void);



#endif

/**@}*/
