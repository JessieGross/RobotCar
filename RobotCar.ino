/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/

float theta_left = 0.1413;
float theta_right = 0.2075;
float beta_left = -70.28;
float beta_right = -43.21;
float v_star = 92.5;

// PWM inputs to jolt the car straight
int left_jolt = 180;
int right_jolt= 240;

// Control gains
float k_left = 0.5;
float k_right = 0.5;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/

float driveStraight_left(float delta) {
  return (v_star + beta_left + k_left * (-delta)) / theta_left;
}

float driveStraight_right(float delta) {
  return (v_star + beta_right + k_right * delta) / theta_right;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 0;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 60.0 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {1000, 2000, 2000, 2000};

float delta_reference(int k) {
  // IMPLEMENTED
  if (drive_mode == DRIVE_RIGHT) {
    return CAR_WIDTH * (v_star / 5) * (k / TURN_RADIUS);
  }
  else if (drive_mode == DRIVE_LEFT) {
    return -(CAR_WIDTH * (v_star / 5) * (k / TURN_RADIUS));
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int k) {
  // IMPLEMENTED
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35
/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                90
#define PRELENGTH                   10
#define THRESHOLD                   0.3

#define KMEANS_THRESHOLD            0.040
#define LOUDNESS_THRESHOLD          300

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/

float pca_vec1[SNIPPET_SIZE] = {-0.0177901088365, -0.0182232980503, -0.0136685526915, -0.0167168576849, -0.0183278790086, -0.0159091188477, -0.0215053562008, -0.0233185207158, -0.0263971143036, -0.016241162929, 0.140263946981, 0.212866132895, 0.195638093183, 0.169279914015, 0.185239400523, 0.180227322904, 0.168476421236, 0.153134607897, 0.147066317612, 0.137937621382, 0.119490027213, 0.109998415926, 0.118000549079, 0.12189997297, 0.1097303217, 0.103109669779, 0.135275888178, 0.133269240085, 0.125832995245, 0.140373428291, 0.149384242066, 0.143069689076, 0.159991582111, 0.165442106539, 0.146236638364, 0.151483559661, 0.149755350229, 0.127278514866, 0.0936774897137, 0.0711341735759, 0.0275175539477, -0.0166554563421, -0.0568715290523, -0.0692272387515, -0.0892774164707, -0.0975283879661, -0.0934562946828, -0.0916669565504, -0.0963995922012, -0.0865074936981, -0.0704880796264, -0.088070235785, -0.0827758562831, -0.0762871278181, -0.0846299407325, -0.0784348000297, -0.083972468972, -0.0846010547376, -0.0931758899379, -0.0912783593287, -0.081755415976, -0.0785850007794, -0.0825013899901, -0.0884644085136, -0.0908321357351, -0.10205299054, -0.0997025704866, -0.0938112064805, -0.100932029418, -0.119597157564, -0.111367953686, -0.103866901332, -0.100089916763, -0.103129122235, -0.117963235248, -0.106829370924, -0.101289748968, -0.0990772008188, -0.0841103723656, -0.078157730527, -0.0870907827145, -0.0780989450677, -0.0675948680776, -0.0623441972708, -0.0590689759052, -0.0484450723, -0.0590624337299, -0.0609757348689, -0.0615791211925, -0.0443030495301};
float pca_vec2[SNIPPET_SIZE] = {0.0104653108319, 0.0126866849673, 0.0102290613407, 0.010671863334, 0.00424174127144, 0.0109845953129, 0.0113944268476, 0.0271590589242, 0.0154252805935, 0.0369776719477, 0.0344366794979, -0.0240083741683, 0.0559238181921, 0.131372781403, 0.117407046927, 0.135638381161, 0.0960218642175, 0.114950718122, 0.11259425017, 0.113090883315, 0.114573825408, 0.0857192962296, 0.0727246727785, 0.055679788095, 0.0295573070745, -0.0317822986455, -0.075110712854, -0.1214323611, -0.116593873338, -0.140268043112, -0.156847162204, -0.140941380119, -0.162641594187, -0.197033894803, -0.183321378746, -0.174789922229, -0.0911753988563, -0.0461673449516, 0.00218402374745, 0.0881514327206, 0.125767319346, 0.137027590645, 0.209315974757, 0.169989223211, 0.217963310418, 0.249451038123, 0.201458345131, 0.154668518239, 0.171063104631, 0.153223718519, 0.0946881578492, 0.0417314282232, -0.0173550209909, -0.0248160746682, -0.045881392567, -0.0825504175043, -0.0851632337861, -0.0932408423358, -0.128038891657, -0.118190868868, -0.115802923041, -0.0986902570363, -0.113504865997, -0.125542475026, -0.126539061139, -0.118707172255, -0.117705361973, -0.09916829836, -0.0807986436648, -0.0904776448387, -0.0518505881386, -0.0729272320243, -0.076756034003, -0.0781733323552, -0.0838214758067, -0.0791101915572, -0.0808702642326, -0.0628354862718, -0.0365897946407, -0.0229744607695, -0.0148649455148, 0.000778862520418, 0.000723398992787, 0.0478908930704, 0.0752901429307, 0.0690515250695, 0.0838464369611, 0.110671536965, 0.126082527059, 0.054115473243};
float mean_vec[SNIPPET_SIZE] = {0.00194978593012, 0.00203372939712, 0.00169260172189, 0.00182076852863, 0.00193386771095, 0.00193912138031, 0.00212331588921, 0.00245758372791, 0.00279536624114, 0.00538183838278, 0.0185393185814, 0.0208183406295, 0.0206033667585, 0.0211446575004, 0.0217158079475, 0.0208502175354, 0.0205201490705, 0.0208434383778, 0.0207357901902, 0.0205474785778, 0.0205587214107, 0.0205548650608, 0.0204549492575, 0.0204678141178, 0.0200840632448, 0.0197664807794, 0.0193832373225, 0.0182354093075, 0.018222736833, 0.0185566946488, 0.0184813729812, 0.0186417705919, 0.0185673728724, 0.0185663619919, 0.0175798409327, 0.0182893217345, 0.0197773140448, 0.0201197258937, 0.0202254531981, 0.020724208672, 0.0191441311422, 0.0153093408924, 0.0158079263865, 0.0137731712824, 0.0127881792696, 0.0126696014007, 0.0112719271658, 0.00987186378332, 0.00954766694169, 0.00888972199327, 0.00867547391217, 0.00857113820397, 0.00677058606629, 0.00633187958601, 0.00661787128997, 0.00637805382617, 0.00698264473779, 0.00668005222047, 0.00695629447825, 0.00695176477877, 0.00638628022752, 0.00587043252213, 0.00660856728359, 0.00658536418991, 0.0063718004832, 0.0069233076681, 0.00663050485644, 0.00647911361491, 0.00698482056222, 0.00808051537571, 0.00779735628754, 0.00692663760114, 0.00658851428345, 0.00694417203606, 0.00721045790854, 0.00660870849885, 0.00626617040115, 0.00632072899846, 0.00543436866415, 0.00521895065707, 0.00573228537231, 0.00541972278857, 0.00479722915889, 0.00492933832746, 0.00516470736031, 0.00455567481606, 0.00501241585316, 0.00541422638758, 0.0053209599421, 0.00372311954067};
float centroid1[2] = {0.07625257, -0.00760706};
float centroid2[2] = {-0.01339055,  0.05445723};
float centroid3[2] = {0.0029391,  -0.01799813};
float centroid4[2] = {-0.05822919, -0.02296684};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i <= 4; i++) {
    sample_lens[i] = run_times[i]/SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);
    Serial.println("Looping");

    // if enveloped data is above some preset value
    if (envelope(re, result)) {
      Serial.println("Loudness good");
      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Subtract the mean
      for(int i = 0; i < SNIPPET_SIZE; i++) {
        result[i] -= mean_vec[i];
      }

      // Perform principal component projection
      // IMPLEMENTED

      for (int i = 0; i < SNIPPET_SIZE ; i++){
        proj1 += result[i] * pca_vec1[i];
        proj2 += result[i] * pca_vec2[i];
      }
      


      // Classification - Hint: use the function l2_norm defined above
      // IMPLEMENTED
      float norm1 =  l2_norm(proj1, proj2, centroids[0]);
      float norm2 =  l2_norm(proj1, proj2, centroids[1]);
      float norm3 =  l2_norm(proj1, proj2, centroids[2]);
      float norm4 =  l2_norm(proj1, proj2, centroids[3]);

      float min_norm = norm1;
      String s = "Go";
      int driveMode = 0;
      if(min_norm > norm2) {min_norm = norm2; s = "Go Back"; driveMode = 2;}
      if(min_norm > norm3) {min_norm = norm3; s = "Hungry"; driveMode = 1;}
      if(min_norm > norm4) {min_norm = norm4; s = "Banana"; driveMode = 3;}

      // Check against KMEANS_THRESHOLD and print result over serial
      // IMPLEMENTED
      
      if (min_norm < KMEANS_THRESHOLD) {
        Serial.println(s);
        drive_mode = driveMode; // from 0-3, inclusive
        start_drive_mode();
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.8*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TA2CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TA2CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TA2CCTL0 = CCIE; // enable interrupts for Timer A
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
