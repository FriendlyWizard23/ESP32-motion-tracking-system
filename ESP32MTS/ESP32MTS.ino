#include "WIFICredentials.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <TaskScheduler.h>
#include <Stepper.h>
#include <TinyStepper_28BYJ_48.h>
#include "camera_pins.h"

/// SKETCH DEFINITIONS
// CONFIG DEFINITIONS
#define SERIAL                          1
#define ENABLE_WEB_SERVER               0
#define CAMERA_MODEL_AI_THINKER
// RESOLUTION DEFINITIONS
#define FRAME_SIZE                      FRAMESIZE_QVGA
#define RES_WIDTH                       320
#define RES_HEIGHT                      240
// MOTION DEFINITIONS                               
#define MOTION_DETECTION_THRESHOLD      0
#define FRAME_BLOCK_DIFF_THRESHOLD      0.8
#define FRAME_BLOCK_SIZE                4 // maggiore è, minore è la precisione
// BLOCKS DEFINITIONS
#define FRAME_H                         (RES_HEIGHT/FRAME_BLOCK_SIZE)
#define FRAME_W                         (RES_WIDTH/FRAME_BLOCK_SIZE)
#define VIEWPORT_PIXELS                 (RES_WIDTH/FRAME_BLOCK_SIZE)
// REGIONS DEFINITIONS
#define WIM_REGIONS                     10
#define PIXELS_IN_REGION                (VIEWPORT_PIXELS/WIM_REGIONS)
#define MID_REGION                      floor((WIM_REGIONS+1)/2)
// STEPPER DEFINITIONS
#define STEPS_PER_DEGREE                6 
#define STEPS_PER_REVOLUTION            2048
#define STEPS_PER_SECOND                4096
#define ACCELLERATION_STEPS_PER_SECOND  256
#define STEPPER_RPM                     10
// BUZZER DEFINITIONS
#define USE_BUZZER                      1
// Definizione delle frequenze delle note musicali
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784

// Note
int melody[] = {
  NOTE_G4, NOTE_G4, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_E4, NOTE_C5,
  NOTE_D5, NOTE_E5, NOTE_G4, NOTE_G4, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_G4,
  NOTE_E4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_E4,
  NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5
};

// Durata delle note corrispondente (1 = una croma, 2 = una semi-breve, ecc.)
int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 4, 4,
  4, 4, 4, 4, 4, 4, 4, 4,
  4, 4, 4, 4, 4, 4, 4, 4,
  4, 4, 4, 4, 4, 4, 4, 4
};

// WebServer variables
WebServer server(80);
WiFiClient client;

// Buzzer Variables
const uint8_t BUZZER_IN = 2;

// Frame variables
camera_fb_t *fb;
uint16_t prev_frame[FRAME_H][FRAME_W]={0};
uint16_t current_frame[FRAME_H][FRAME_W]={0};
uint16_t empty_frame[FRAME_H][FRAME_W]={0};

// motion variables
long where_is_motion[VIEWPORT_PIXELS]={0};
bool flag = false;
int moveTP = 0;
int region_movement=0;

// stepper motor variables
int currentMP=0; // Current Motor Position
const uint8_t STEP_IN1 = 12;
const uint8_t STEP_IN2= 13;
const uint8_t STEP_IN3= 14;
const uint8_t STEP_IN4 = 15;
Stepper generic_stepper(STEPS_PER_REVOLUTION,STEP_IN1,STEP_IN3,STEP_IN2,STEP_IN4);

// Variabiles to track previous movement
int previous_moveTP = 0;    // Previous destination target position
int correction_factor = 5;  // Weighted correction factor

// laser stuff
bool youcantjustshootawholeintothesurfaceofmars=false;
const uint8_t LASER_IN=2;

// Scheduler
Scheduler tasks;
void moveStepperMotor();
Task stepperTask (1, TASK_FOREVER, &moveStepperMotor);
void get_frame();
Task getframeTask(30,TASK_FOREVER, &get_frame);
void send_jpg_frame();
Task sendjpgframeTask(10,TASK_FOREVER, &send_jpg_frame);
void emit_nuclear_laser_beam();
Task emitnuclearlaserbeamTask(10,TASK_FOREVER, &emit_nuclear_laser_beam);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN,OUTPUT);
  stepper_init();
  tasks_init();
  #if USE_BUZZER
  buzzer_init();
  playSound();
  #endif
  laser_init();
  bool camera=camera_init(FRAME_SIZE,PIXFORMAT_GRAYSCALE);
  #if SERIAL
  showDiagnostics();
  Serial.println(camera ? "CAMERA READY AND OK" : "ERROR INITIATING CAMERA");
  Serial.println("[SCHEDULER]> Tasks setup succesfully.");
  Serial.println("[STEPPER]> Stepper setup succesfully.");
  Serial.println("[BFG DIVISION]> Laser Beam Ready.");
  #endif 

  #if ENABLE_WEB_SERVER
  // if ENABLE_WEB_SERVER is set, activate wifi and start server
  connect_to_WIFI();
  startCameraServer();
  #endif
  stepperTest();
  blinkFlash();
  tasks.startNow();
}

void loop() {
  tasks.execute();
  #if ENABLE_WEB_SERVER
  server.handleClient();
  #endif
  bool motion=detect_motion();
  if (flag && motion){
    region_movement=calculate_region(where_is_motion);
    moveTP=calculate_moveTP(region_movement);
    #if SERIAL
    Serial.println("=====================================================================================================");
    Serial.println("[MOTION]> Motion detected");
    Serial.println("[MOTION]> Region: "+String(region_movement));
    Serial.println("[MOTION]> Moving to: "+String(moveTP));
    Serial.println("[MOTION]> Motion Array is: ");
    #endif
    clear_motion_buffer();
  }
  flag=true;

  // update previous frame. previous is now the latest captured
  update_prev_frame();
  // clean buffer
  esp_camera_fb_return(fb);
  // do not overload the esp32
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// SERVER MANAGEMENT ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void startCameraServer() {
  server.on("/stream", HTTP_GET, handle_jpg_stream_request);  // Percorso per visualizzare il feed MJPEG
  server.begin();  // Avvio del server
  #if SERIAL
  Serial.println("[SERVER]> Successfully started server");
  #endif
}

void handle_jpg_stream_request() {
  client = server.client(); 
  if (client) {
    String header = "HTTP/1.1 200 OK\r\n";
    header += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    client.print(header);
    #if SERIAL
    Serial.println("[SERVER]> Client connected, starting stream...");
    #endif
  }
}

void send_jpg_frame() {
  if (client && client.connected()) {  
    if (!fb) {
      Serial.println("[CAMERA]> Error while capturing frame");
      client.stop(); 
      return;
    }
    uint8_t *jpeg_buf = nullptr;
    size_t jpeg_len = 0;
    bool jpeg_converted = frame2jpg(fb, 40, &jpeg_buf, &jpeg_len);  // converting to jpg with 80% quality
    if (!jpeg_converted) {
      Serial.println("[CAMERA]> Error converting grayscale image to JPEG");
      esp_camera_fb_return(fb);  
      client.stop();  
      return;
    }
    // sending jpeg-ed frame to webserver
    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", jpeg_len);
    client.write(jpeg_buf, jpeg_len);
    client.print("\r\n");
    free(jpeg_buf);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// MOTION MANAGEMENT ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

int calculate_moveTP(int region) {
    int relative_region = region - MID_REGION;
    int moveTP = relative_region * WIM_REGIONS * STEPS_PER_DEGREE;
    int delta = moveTP - previous_moveTP;
    if (abs(delta) > correction_factor) {
        moveTP += delta / 2; 
    }
    previous_moveTP = moveTP;
    return moveTP;
}

/**
 * Detects motion by comparing the current frame to the previous frame.
 *
 * The function divides the frames into blocks and calculates the relative 
 * difference between corresponding blocks. If the difference exceeds a set 
 * threshold, the block is marked as changed. Motion is detected if the number 
 * of changed blocks exceeds a defined threshold.
 *
 * @return true if the number of changed blocks exceeds the motion detection threshold, 
 *         false otherwise.
 */
bool detect_motion() {
  uint16_t changed_blocks = 0;
  int lastBlock = 0;
  // total number of blocks inside a captured frame. NB: Adjust it on definitions for accuracy.
  const uint16_t total_blocks = (RES_WIDTH * RES_HEIGHT) / (FRAME_BLOCK_SIZE * FRAME_BLOCK_SIZE);
  for (uint16_t y = 0; y < FRAME_H; y++) {
    for (uint16_t x = 0; x < FRAME_W; x++) {
      if (prev_frame[y][x]==0){
        prev_frame[y][x]=1;
      }
      float current=current_frame[y][x];
      float prev=prev_frame[y][x];
      float fblock_diff = abs(current - prev)/prev;
      // Is the difference above the Threshold?
      //Serial.println("Current: "+String(current)+" Previous: "+String(prev)+" Diff is: "+String(fblock_diff));
      if (fblock_diff >= FRAME_BLOCK_DIFF_THRESHOLD) {
        where_is_motion[x] = 1;
        changed_blocks++;
      }
    }
  }
  // if no blocks are showing diffs it returns false.
  if (changed_blocks == 0) {
    return false;  
  }
  //clearing the mask array for next computation
  //clear_motion_buffer();
  // if the number of blocks that show differences are above threshold, yes, there is motion.
  return (1.0*changed_blocks/total_blocks) > MOTION_DETECTION_THRESHOLD;
}

bool naive_motion_detect(){

}

void get_frame() {
  fb = esp_camera_fb_get();
  if (!fb) {
    #if SERIAL
    Serial.println("[CAMERA]> ERROR WHILE GETTING FRAME");
    #endif
    return;
  }
  // down-sample image in blocks
  for (uint32_t i = 0; i < RES_WIDTH * RES_HEIGHT; i++) {
    const uint16_t x = i % RES_WIDTH;
    const uint16_t y = floor(i / RES_WIDTH);
    const uint8_t block_x = floor(x / FRAME_BLOCK_SIZE);
    const uint8_t block_y = floor(y / FRAME_BLOCK_SIZE);
    const uint8_t pixel = fb->buf[i];
    const uint16_t current = current_frame[block_y][block_x];
    // average pixels in block (accumulate)
    current_frame[block_y][block_x] += pixel;
  }
  // average pixels in block (rescale)
  for (int y = 0; y < FRAME_H; y++){
    for (int x = 0; x < FRAME_W; x++){
      current_frame[y][x] /= FRAME_BLOCK_SIZE * FRAME_BLOCK_SIZE;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// STEPPER MANAGEMENT /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


int calculate_region(long motion_view[]){
  int cur_region[PIXELS_IN_REGION];
  int region_movement_array[WIM_REGIONS];
  int region_counter=0;
  for(int motion_element=0; motion_element<VIEWPORT_PIXELS; motion_element++){
    cur_region[motion_element%(PIXELS_IN_REGION-1)]=motion_view[motion_element];
    if((motion_element%(PIXELS_IN_REGION-1)==0) && motion_element!=0){
      region_movement_array[region_counter]=count_ones_in_region(cur_region);
      region_counter++;
    }
  }
  return find_max_region(region_movement_array);

}

void moveStepperMotor() {
    // difference between target and current position
    int distance_to_target = moveTP - currentMP;
    // if target is yet to be reached
    if (distance_to_target != 0) {
        // take half of the distance as steps to be taken
        int step_to_take = distance_to_target / 2;
        // if movement is too small at least do one step
        if (abs(step_to_take) < 1) {
            step_to_take = distance_to_target; // final step
        }
        // moving stepper to number of steps needed
        generic_stepper.step(step_to_take);
        // updatieng current steps
        currentMP += step_to_take;
    } 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// SUPPORT FUNCTIONS ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void blinkFlash(){
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

void clear_motion_buffer(){
  #if SERIAL
  Serial.print("[");
  #endif
  for (uint16_t i = 0; i < VIEWPORT_PIXELS; i++) {
    #if SERIAL
    if((i%PIXELS_IN_REGION)==0 && i!=0){
      Serial.print(", ");
    }
    Serial.print(where_is_motion[i]);
    #endif
    where_is_motion[i] = 0;
  }
  #if SERIAL
  Serial.println("]");
  #endif
}

void print_frame(uint16_t frame[FRAME_H][FRAME_W]) {
    for (int y = 0; y < FRAME_H; y++) {
        for (int x = 0; x < FRAME_W; x++) {
            Serial.print(frame[y][x]);
            Serial.print('\t');
        }
        Serial.println();
    }
}

void showDiagnostics(){
  Serial.println("[ESP32]> CPU Frequency :: "+String(getCpuFrequencyMhz())+" Mhz");
  Serial.println("[ESP32]> XTAL Frequency :: "+String(getXtalFrequencyMhz())+" Mhz");
  Serial.println("[ESP32]> APB Freq = "+String(getApbFrequency())+" Hz");
}

bool connect_to_WIFI(){
  // Connessione Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    #if SERIAL
    Serial.print("[WIFI]> Connecting to wifi: " + String(WIFI_SSID) + " with pwd: " + String(WIFI_PASSWORD) + "...\n");
    #endif
  }
  #if SERIAL
  Serial.println("");
  Serial.println("[WIFI]> Wifi Connected");
  Serial.println("[WIFI]> IP: ");
  Serial.println(WiFi.localIP());
  #endif
  return true;
}

void update_prev_frame() {
  memcpy( prev_frame, current_frame, sizeof(prev_frame)); 
}

int find_max_region(int arr[]) {
    int index=0;
    int maxVal = arr[0];
    for (int i = 1; i < WIM_REGIONS; i++) {
        if(arr[i]>=maxVal){
          maxVal=arr[i];
          index=i;
        }
    }
    return index;
}

void emit_nuclear_laser_beam(){
  if (currentMP == moveTP) {
    digitalWrite(LASER_IN,HIGH);
  }else{
    digitalWrite(LASER_IN,LOW);
  }
}

int count_ones_in_region(int region[]){
  int ones=0;
  for(int i=0; i<PIXELS_IN_REGION;i++){
    if (region[i]==1){
      ones++;
    }
  }
  return ones;
}

void stepperTest(){
  generic_stepper.step(STEPS_PER_REVOLUTION/2);
  delay(1000);
  generic_stepper.step(-STEPS_PER_REVOLUTION/2);
}

void playSound() {
  tone(BUZZER_IN,1000,2000);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// GENERAL INITS /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void stepper_init(){
  generic_stepper.setSpeed(STEPPER_RPM);
}

void tasks_init(){
  tasks.addTask(stepperTask);
  tasks.addTask(getframeTask);
  #if ENABLE_WEB_SERVER
  tasks.addTask(sendjpgframeTask);
  sendjpgframeTask.enable();
  #endif
  tasks.addTask(emitnuclearlaserbeamTask);
  emitnuclearlaserbeamTask.enable();
  getframeTask.enable();
  stepperTask.enable();
}


bool camera_init(framesize_t frameSize,pixformat_t PIXEL_FORMAT) {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXEL_FORMAT;
    config.frame_size = frameSize;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    esp_camera_deinit();
    bool ok = esp_camera_init(&config) == ESP_OK;
    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_framesize(sensor, frameSize);
    return ok;
}

void laser_init(){
    pinMode(LASER_IN,OUTPUT);
    digitalWrite(LASER_IN,HIGH);
    delay(2000);
    digitalWrite(LASER_IN,LOW);
    delay(2000);
}

void buzzer_init(){
  pinMode(BUZZER_IN,OUTPUT);
}