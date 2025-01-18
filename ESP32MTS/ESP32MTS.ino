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

// STEPPER DEFINITIONS
#define STEPS_PER_DEGREE                6
#define STEPS_PER_REVOLUTION            2048
#define STEPPER_RPM                     10

//TEST
#define REGIONS                         11
#define MID_REGION                      floor(REGIONS/2)
#define CHANGE_THRESHOLD                65000
#define HORIZONTAL_PIXELS_PER_REGION    floor(RES_WIDTH/REGIONS)      
#define FOV_HORIZONTAL                  65

int current_region_sums[REGIONS];   
int previous_region_sums[REGIONS];  
int regions_diff[REGIONS]; 

// WebServer variables
WebServer server(80);
WiFiClient client;


// Frame variables
camera_fb_t *fb;

// motion variables
bool flag = false;
int moveTP = 0;
int region_movement = 0;

// stepper motor variables
int currentMP = 0;  // Current Motor Position
const uint8_t STEP_IN1 = 12;
const uint8_t STEP_IN2 = 13;
const uint8_t STEP_IN3 = 14;
const uint8_t STEP_IN4 = 15;
Stepper generic_stepper(STEPS_PER_REVOLUTION, STEP_IN1, STEP_IN3, STEP_IN2, STEP_IN4);

// Variabiles to track previous movement
int previous_moveTP = 0;    // Previous destination target position
int correction_factor = 5;  // Weighted correction factor

// laser stuff
bool youcantjustshootaholeintothesurfaceofmars = false;
const uint8_t LASER_IN = 2;

// Scheduler
Scheduler tasks;
void moveStepperMotor();
Task stepperTask(10, TASK_FOREVER, &moveStepperMotor);
void get_frame();
Task getframeTask(10, TASK_FOREVER, &get_frame);
void send_jpg_frame();
Task sendjpgframeTask(10, TASK_FOREVER, &send_jpg_frame);
void emit_nuclear_laser_beam();
Task emitnuclearlaserbeamTask(10, TASK_FOREVER, &emit_nuclear_laser_beam);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  stepper_init();
  //stepperTest();
  tasks_init();
  laser_init();
  bool camera = camera_init(FRAME_SIZE, PIXFORMAT_GRAYSCALE);
#if SERIAL
  showDiagnostics();
  Serial.println(camera ? "CAMERA READY AND OK" : "ERROR INITIATING CAMERA.");
  Serial.println(F("[SCHEDULER]> Tasks setup succesfully."));
  Serial.println(F("[STEPPER]> Stepper setup succesfully."));
  Serial.println(F("[BFG DIVISION]> Laser Beam Ready."));
#endif
#if ENABLE_WEB_SERVER
  // if ENABLE_WEB_SERVER is set, activate wifi and start server
  connect_to_WIFI();
  startCameraServer();
#endif

  // Final Blink to acknolwedge it's all good man
  blinkFlash();
  tasks.startNow();
}

void loop() {
  tasks.execute();
#if ENABLE_WEB_SERVER
  server.handleClient();
#endif
  bool motion = detect_motion();
  if (flag && motion) {
    region_movement = calculate_region(regions_diff);
    moveTP = calculate_moveTP(region_movement);
#if SERIAL
    Serial.println(F("==============================================="));
    Serial.println(F("[MOTION]> Motion detected"));
    Serial.println("[MOTION]> Motion in Region: " + String(region_movement));
    Serial.println("[MOTION]> Moving To: " + String(moveTP));
    Serial.println(F("==============================================="));
#endif
  }
  flag = true;
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
  server.begin();                                             // Avvio del server
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
    Serial.println(F("[SERVER]> Client connected, starting stream..."));
#endif
  }
}

void send_jpg_frame() {
  if (client && client.connected()) {
    if (!fb) {
      Serial.println(F("[CAMERA]> Error while capturing frame"));
      client.stop();
      return;
    }
    uint8_t *jpeg_buf = nullptr;
    size_t jpeg_len = 0;
    bool jpeg_converted = frame2jpg(fb, 40, &jpeg_buf, &jpeg_len);  // converting to jpg with 80% quality
    if (!jpeg_converted) {
      Serial.println(F("[CAMERA]> Error converting grayscale image to JPEG"));
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
    int relative_pixel_offset = (relative_region * HORIZONTAL_PIXELS_PER_REGION) + (HORIZONTAL_PIXELS_PER_REGION / 2);
    float angular_offset = (float(relative_pixel_offset) / RES_WIDTH) * FOV_HORIZONTAL;
    int steps_to_move = angular_offset * STEPS_PER_DEGREE;
    return steps_to_move;
}
bool detect_motion() {
    int region_width = RES_WIDTH / REGIONS;
    memset(current_region_sums, 0, sizeof(current_region_sums));
    for (int y = 0; y < RES_HEIGHT; y++) {
        for (int x = 0; x < RES_WIDTH; x++) {
            int pixel_index = (y * RES_WIDTH) + x;
            uint8_t pixel_value = fb->buf[pixel_index];
            int region_index = x / region_width;
            current_region_sums[region_index] += pixel_value;
        }
    }

    bool motion_detected = false;
    for (int i = 0; i < REGIONS; i++) {
        regions_diff[i] = abs(current_region_sums[i] - previous_region_sums[i]);
        if (regions_diff[i] > CHANGE_THRESHOLD) {
            motion_detected = true;
        }
    }
    memcpy(previous_region_sums, current_region_sums, sizeof(previous_region_sums));
    return motion_detected;
}

void get_frame() {
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println(F("Errore durante la cattura del frame"));
    return;
  }
  int region_width = RES_WIDTH / REGIONS;
  memset(current_region_sums, 0, sizeof(current_region_sums));
  for (int y = 0; y < RES_HEIGHT; y++) {
    for (int x = 0; x < RES_WIDTH; x++) {
      int pixel_index = (y * RES_WIDTH) + x;
      uint8_t pixel_value = fb->buf[pixel_index];
      int region_index = x / region_width;
      current_region_sums[region_index] += pixel_value;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// STEPPER MANAGEMENT /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


int calculate_region(int motion_view[]) {
    int max_value = 0;
    int max_index = -1;
    for (int i = 0; i < REGIONS; i++) {
        if (regions_diff[i] > max_value) {
            max_value = regions_diff[i];
            max_index = i;
        }
    }
    return max_index;
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
      step_to_take = distance_to_target;  // final step
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

void blinkFlash() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}


void showDiagnostics() {
  Serial.println("[ESP32]> CPU Frequency :: " + String(getCpuFrequencyMhz()) + " Mhz");
  Serial.println("[ESP32]> XTAL Frequency :: " + String(getXtalFrequencyMhz()) + " Mhz");
  Serial.println("[ESP32]> APB Freq = " + String(getApbFrequency()) + " Hz");
}

bool connect_to_WIFI() {
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
  Serial.println(F(""));
  Serial.println(F("[WIFI]> Wifi Connected"));
  Serial.println(F("[WIFI]> IP: "));
  Serial.println(WiFi.localIP());
#endif
  return true;
}

void emit_nuclear_laser_beam() {
  if (currentMP == moveTP) {
    digitalWrite(LASER_IN, HIGH);
  } else {
    digitalWrite(LASER_IN, LOW);
  }
}

void stepperTest() {
  generic_stepper.step(STEPS_PER_REVOLUTION / 2);
  delay(1000);
  generic_stepper.step(-STEPS_PER_REVOLUTION / 2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// GENERAL INITS /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void stepper_init() {
  generic_stepper.setSpeed(STEPPER_RPM);
}

void tasks_init() {
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


bool camera_init(framesize_t frameSize, pixformat_t PIXEL_FORMAT) {
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

void laser_init() {
  pinMode(LASER_IN, OUTPUT);
  digitalWrite(LASER_IN, HIGH);
  delay(2000);
  digitalWrite(LASER_IN, LOW);
  delay(2000);
}
