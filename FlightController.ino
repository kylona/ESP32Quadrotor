/*
Written by Kyle Storey as part of a project for a Robust Control Course at Brigham Young University
It is still a work in progress. It's not a complete perfect controller but it should serve as a good
starting off point for anyone hoping to do ESC control or quadrotor flight control using and ESP32

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"
#include <math.h>

#define ACTUATE_SENSITIVITY 80
#define ANGLE_CORRECT 2
#define AV_CORRECT 2
#define INTEGRAL_CORRECT 2
#define INTEGRAL_MEMORY_LEN 50
#define ORI_ROLL_TWEEK (4 * PI / 180)
#define ORI_PITCH_TWEEK (4 * PI / 180)
#define ORI_YAW_TWEEK 0
#define HOVER_ADJUST 20
#define FRONT_HOVER_SPEED (400 - HOVER_ADJUST)
#define BACK_HOVER_SPEED (420 - HOVER_ADJUST)
#define LEFT_HOVER_SPEED (435 - HOVER_ADJUST)
#define RIGHT_HOVER_SPEED (410 - HOVER_ADJUST)

#define FRONT_PIN 33
#define BACK_PIN 27
#define LEFT_PIN 25
#define RIGHT_PIN 26
#define FRONT_CHANNEL 0
#define BACK_CHANNEL 1
#define LEFT_CHANNEL 2
#define RIGHT_CHANNEL 3
#define MIN_SPEED_PWM 1960
#define MAX_SPEED_PWM 2960


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  pinMode(FRONT_PIN,OUTPUT);
  pinMode(BACK_PIN,OUTPUT);
  pinMode(LEFT_PIN,OUTPUT);
  pinMode(RIGHT_PIN,OUTPUT);
  ledcSetup(FRONT_CHANNEL, 50, 15);
  ledcSetup(BACK_CHANNEL, 50, 15);
  ledcSetup(LEFT_CHANNEL, 50, 15);
  ledcSetup(RIGHT_CHANNEL, 50, 15);
  ledcAttachPin(FRONT_PIN, FRONT_CHANNEL);
  ledcAttachPin(BACK_PIN, BACK_CHANNEL);
  ledcAttachPin(LEFT_PIN, LEFT_CHANNEL);
  ledcAttachPin(RIGHT_PIN, RIGHT_CHANNEL);
  setMotors(0,0,0,0);
  delay(10000);


}

struct Orient
{
  float roll;
  float pitch;
  float yaw;
};


void getOrient(struct Orient &o) {
   float x = IMU.getMagX_uT();
   float y = IMU.getMagY_uT();
   o.yaw = atan2(y-20, x-20) - ORI_YAW_TWEEK;
   x = IMU.getAccelX_mss();
   y = IMU.getAccelY_mss();
   float z = IMU.getAccelZ_mss();
   o.roll = atan2(-x, z) - ORI_ROLL_TWEEK;
   o.pitch = atan2(-y, z) - ORI_PITCH_TWEEK;
}

void getAngularVelocity(struct Orient &o) {
  o.roll = IMU.getGyroY_rads();
  o.pitch = -IMU.getGyroX_rads();
  o.yaw = IMU.getGyroZ_rads();
}


struct Actuate 
{
  float thrust;
  float roll;
  float pitch;
  float yaw;
};

struct MotorSpeeds
{
  float front;
  float back;
  float left;
  float right;
};

void addMotorDelta(struct MotorSpeeds &speeds, struct Actuate act) {
  speeds.front += act.thrust * ACTUATE_SENSITIVITY;
  speeds.back += act.thrust * ACTUATE_SENSITIVITY;
  speeds.left += act.thrust * ACTUATE_SENSITIVITY;
  speeds.right += act.thrust * ACTUATE_SENSITIVITY;

  speeds.left -= act.roll * ACTUATE_SENSITIVITY;
  speeds.right += act.roll * ACTUATE_SENSITIVITY;

  speeds.front -= act.pitch * ACTUATE_SENSITIVITY;
  speeds.back += act.pitch * ACTUATE_SENSITIVITY;

  speeds.front -= act.yaw * ACTUATE_SENSITIVITY;
  speeds.back -= act.yaw * ACTUATE_SENSITIVITY;
  speeds.left += act.yaw * ACTUATE_SENSITIVITY;
  speeds.right += act.yaw * ACTUATE_SENSITIVITY;
}

void setMotors(int front, int back, int left, int right) {
      int linearFront = int(1000*sqrt(front/3000.0));
      int linearBack = int(1000*sqrt(back/3000.0));
      int linearLeft = int(1000*sqrt(left/3000.0));
      int linearRight = int(1000*sqrt(right/3000.0));
      ledcWrite(FRONT_CHANNEL,linearFront + MIN_SPEED_PWM);
      ledcWrite(BACK_CHANNEL,linearBack + MIN_SPEED_PWM);
      ledcWrite(LEFT_CHANNEL,linearRight + MIN_SPEED_PWM);
      ledcWrite(RIGHT_CHANNEL,linearLeft + MIN_SPEED_PWM);
      /*
      Serial.println("Motors");
      Serial.print("\t");
      Serial.println(linearFront);  
      Serial.print(linearLeft);  
      Serial.print("\t\t");
      Serial.println(linearRight);  
      Serial.print("\t");
      Serial.println(linearBack);
      // */
}

Orient integralMemory[INTEGRAL_MEMORY_LEN] = {0};
int integralIndex = 0;

void addIntegral(Orient &ori, Orient &act) {
  integralMemory[integralIndex] = ori;
  integralIndex = (integralIndex + 1) % INTEGRAL_MEMORY_LEN;
  Orient sum;
  for (int i = 0; i < INTEGRAL_MEMORY_LEN; i++) {
    sum.roll += integralMemory[i].roll;
    sum.pitch += integralMemory[i].pitch;
    //sum.yaw += integralMemory[i].yaw;
  }
  sum.roll /= INTEGRAL_MEMORY_LEN;
  sum.pitch /= INTEGRAL_MEMORY_LEN;
  //sum.yaw /= INTEGRAL_MEMORY_LEN;
  
}
void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  /*Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getMagZ_uT(),6);
  */
  struct Orient ori;
  getOrient(ori);
  /*
  Serial.println("Ori");
  Serial.print(ori.roll, 6);  
  Serial.print("\t");
  Serial.print(ori.pitch, 6);  
  Serial.print("\t");
  Serial.println(ori.yaw, 6);
  */
  struct Orient av;
  getAngularVelocity(av);
  /*
  Serial.println("AV");
  Serial.print(av.roll, 6);  
  Serial.print("\t");
  Serial.print(av.pitch, 6);  
  Serial.print("\t");
  Serial.println(av.yaw, 6);
  */

  struct MotorSpeeds hover;
  hover.front = FRONT_HOVER_SPEED;
  hover.back = BACK_HOVER_SPEED;
  hover.left = LEFT_HOVER_SPEED;
  hover.right = RIGHT_HOVER_SPEED;
  struct Actuate actuate;
  actuate.thrust = 0;
  actuate.roll = (-ori.roll * ANGLE_CORRECT) + (-av.roll * AV_CORRECT);
  actuate.pitch = (-ori.pitch * ANGLE_CORRECT) + (-av.pitch * AV_CORRECT);
  actuate.yaw = -av.yaw * AV_CORRECT;
  addMotorDelta(hover, actuate);
  setMotors(hover.front, hover.back, hover.left, hover.right);
}
