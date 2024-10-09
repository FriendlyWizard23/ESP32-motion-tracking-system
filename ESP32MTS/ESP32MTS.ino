#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <TaskScheduler.h>
#include <Stepper.h>
#include <TinyStepper_28BYJ_48.h>
// CAMERA SETTINGS
#include "camera_pins.h"

/// SKETCH DEFINITIONS

#define ENABLE_WEB_SERVER             0
#define CAMERA_MODEL_AI_THINKER                               
#define MOTION_DETECTION_THRESHOLD    0
#define FRAME_BLOCK_DIFF_THRESHOLD    0.8
#define FRAME_BLOCK_SIZE              4 // maggiore è, minore è la precisione
#define RES_WIDTH                     320
#define RES_HEIGHT                    240
#define SERIAL                        0
#define FRAME_H                       (RES_HEIGHT/FRAME_BLOCK_SIZE)
#define FRAME_W                       (RES_WIDTH/FRAME_BLOCK_SIZE)
#define FRAME_SIZE                    FRAMESIZE_QVGA
#define VIEWPORT_PIXELS               (RES_WIDTH/FRAME_BLOCK_SIZE)
#define WIM_REGIONS                   10
#define PIXELS_IN_REGION              (VIEWPORT_PIXELS/WIM_REGIONS)
#define STEPS_PER_DEGREE              6
#define STEPS_PER_REVOLUTION          2048
#define STEPS_PER_SECOND              4096


// WIFI and WebServer Configuration
const char *ssid = "";
const char *password = "";
WebServer server(80);

// Frame variables
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


// scheduler stuff
TinyStepper_28BYJ_48 motor;

Stepper myStepper(STEPS_PER_REVOLUTION, STEP_IN1, STEP_IN3, STEP_IN2, STEP_IN4);

// Scheduler
Scheduler tasks;
void moveStepperMotor();
/*  250 milliseconds delay
    run forever
    moveStepperMotor function
    tasks scheduler name
    enabled forever
*/
Task stepperTask (50, TASK_FOREVER, &moveStepperMotor);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN,OUTPUT);
  stepper_init();
  tasks_init();
  bool camera=camera_init(FRAME_SIZE,PIXFORMAT_GRAYSCALE);
  #if SERIAL
  showDiagnostics();
  Serial.println(camera ? "CAMERA READY AND OK" : "ERROR INITIATING CAMERA");
  Serial.println("[SCHEDULER]> Tasks setup succesfully");
  Serial.println("[STEPPER]> Stepper setup succesfully");
  #endif 

  #if ENABLE_WEB_SERVER
  // if enable web server is set, activate wifi and start server
  connect_to_WIFI();
  startCameraServer();
  #endif
  blinkFlash();
}

void loop() {
  tasks.startNow();
  #if ENABLE_WEB_SERVER
  // Mantiene il webserver attivo
  server.handleClient();
  #endif
  handle_jpg_stream();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// SERVER MANAGEMENT ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void startCameraServer() {
  server.on("/stream", HTTP_GET, handle_jpg_stream);  // Percorso per visualizzare il feed MJPEG
  server.begin();  // Avvio del server
  Serial.println("[SERVER]> Successfully started server");
}


// Gestione dello stream JPG
void handle_jpg_stream() {
  #if ENABLE_WEB_SERVER
  // Permette al sistema di fare altre operazioni
  WiFiClient client = server.client();  
  // Impostazione dell'header della telecamera.
  String header = "HTTP/1.1 200 OK\r\n";
  header += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  client.print(header);
  #endif

  while (true) {
    tasks.execute();
    camera_fb_t *fb = get_frame();
    if (!fb) {
      #if SERIAL
      Serial.println("[CAMERA]> Error while capturing frame");
      #endif
      #if ENABLE_WEB_SERVER
      server.send(503, "text/plain", "Impossible to capture frame");
      #endif
      return;
    }

    #if ENABLE_WEB_SERVER
    // sending single captured frame to webserver
    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    #endif
    // managing disconnection
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
    delay(30);  
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


void showDiagnostics(){
  Serial.println("[ESP32]> CPU Frequency :: "+String(getCpuFrequencyMhz())+" Mhz");
  Serial.println("[ESP32]> XTAL Frequency :: "+String(getXtalFrequencyMhz())+" Mhz");
  Serial.println("[ESP32]> APB Freq = "+String(getApbFrequency())+" Hz");
}

bool connect_to_WIFI(){
  // Connessione Wi-Fi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    #if SERIAL
    Serial.print("[WIFI]> Connecting to wifi: " + String(ssid) + " with pwd: " + String(password) + "...\n");
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

void stepper_init(){
  myStepper.setSpeed(10);
}

void tasks_init(){
  tasks.addTask(stepperTask);
  stepperTask.enable();
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

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// MOTION MANAGEMENT ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

int calculate_moveTP(int region_movement) {
  // 5 is the middle region
  int center_region = 5;
  int offset = (region_movement) - center_region;
  return offset * 10 * STEPS_PER_DEGREE;
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



/**
 * Captures a frame from the camera and processes it into blocks.
 *
 * The function retrieves a frame from the camera, down-samples it by dividing
 * the image into blocks, and calculates the average pixel value for each block.
 * The processed frame is stored in the `current_frame` buffer.
 *
 * @return A pointer to the captured frame buffer (camera_fb_t) if successful,
 *         or NULL if the frame could not be captured.
 *
 * The function performs the following steps:
 * 1. Captures a frame using the camera's frame buffer.
 * 2. Divides the frame into blocks and accumulates the pixel values for each block.
 * 3. Averages the pixel values in each block to produce a down-sampled image.
 */
camera_fb_t* get_frame() {
  camera_fb_t *frame_buffer = esp_camera_fb_get();
  if (!frame_buffer) {
    return NULL;
  }
  // down-sample image in blocks
  for (uint32_t i = 0; i < RES_WIDTH * RES_HEIGHT; i++) {
    const uint16_t x = i % RES_WIDTH;
    const uint16_t y = floor(i / RES_WIDTH);
    const uint8_t block_x = floor(x / FRAME_BLOCK_SIZE);
    const uint8_t block_y = floor(y / FRAME_BLOCK_SIZE);
    const uint8_t pixel = frame_buffer->buf[i];
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
  return frame_buffer;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// STEPPER MANAGEMENT /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
long calculate_region(long motionData[]){
  int maxMotionValue = 0;       // To track the maximum motion value for any region
  int regionWithMaxMotion = 0;  // To store the region with the most detected motion
  int motionValueInRegion = 0;  // Temporary variable for the motion value in the current region
  char binaryRepresentation[9]; // To hold the binary string representation for each region
  binaryRepresentation[8] = '\0';  // Null-terminate the string
  // Loop over all regions (WIM_REGIONS is the total number of regions)
  for(int regionIndex = 0; regionIndex < WIM_REGIONS; regionIndex++){
    // Loop over the pixels in the current region (PIXELS_IN_REGION is the number of pixels per region)
    for(int pixelIndex = 0; pixelIndex < PIXELS_IN_REGION; pixelIndex++){
      // If motion is detected in the pixel, set '1', otherwise '0'
      binaryRepresentation[pixelIndex] = (motionData[(regionIndex * PIXELS_IN_REGION) + pixelIndex] == 1) ? '1' : '0'; 
    } 
    // Convert the binary string of the current region to an integer value (base 2)
    motionValueInRegion = strtol(binaryRepresentation, NULL, 2);
    // Check if the current region's motion value is greater than the previously tracked maximum motion value
    if(motionValueInRegion > maxMotionValue){
      maxMotionValue = motionValueInRegion;  // Update the maximum motion value
      regionWithMaxMotion = regionIndex;     // Set the region with the most motion activity
    }
  }
  // Print the region with the most activity
  return regionWithMaxMotion;
}*/

int calculate_region(long motion_view[]){
  char binary_reppresentation[9];
  binary_reppresentation[8]='\0';
  int region_movement_array[WIM_REGIONS];
  int region_counter=0;
  for(int motion_element=0; motion_element<VIEWPORT_PIXELS; motion_element++){
    binary_reppresentation[motion_element%(PIXELS_IN_REGION-1)]=((motion_view[motion_element]==1)?'1':'0');
    if((motion_element%(PIXELS_IN_REGION-1)==0) && motion_element!=0){
      region_movement_array[region_counter]=(int)strtol(binary_reppresentation,NULL,2);
      region_counter++;
    }
  }
  return find_max_region(region_movement_array);

}

void moveStepperMotor() {
    // Sposta direttamente alla posizione target
    if (currentMP!=moveTP){
      #if SERIAL
      Serial.println("[STEPPER]> moveTP: " + String(moveTP));
      Serial.println("[STEPPER]> CURRENT: " + String(currentMP));
      #endif
      myStepper.step(moveTP);
    }
    moveTP=0;
    currentMP = 0; 
}


void mystepperTest(){
  #if SERIAL
    Serial.println("[MOTOR]> Rotating forward "+String(STEPS_PER_REVOLUTION)+" steps...");
  #endif
  myStepper.step(STEPS_PER_REVOLUTION);
  delay(2000);
  #if SERIAL
    Serial.println("[MOTOR]> Rotating backwards "+String(STEPS_PER_REVOLUTION)+" steps...");
  #endif
  myStepper.step(-STEPS_PER_REVOLUTION);
}



