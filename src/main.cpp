#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <math.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <Wire.h>
#include "FXOS8700Q_Basic.h"
#include "MadgwickAHRS.h"
#include "MS5837.h"
#include <mpu6050.hpp>
// #include <geometry_msgs/Quaternion.h>
#include "main.hpp"

float raw_a[3];
float raw_g[3];
float raw_m[3];
float cal_a[3];
float cal_g[3];
float cal_m[3];
float _ax_offset = 0.0, _ay_offset = 0.0, _az_offset = 0.0;
float _gx_offset = 0.0, _gy_offset = 0.0, _gz_offset = 0.0;
float _mx_offset = 59.94, _my_offset = 89.26, _mz_offset = 75.24;
float soft_iron_matrix[3][3]={{0.989,-0.026,-0.013},{-0.026,0.963,-0.009},{-0.013,-0.009,1.052}};
float yaw, pitch, roll, depth,roll_lpf=0,pitch_lpf=0,yaw_lpf=0;
float values[3];
// float qx,qy,qz,qw;
int prev_time_update, prev_time_publish;

std_msgs::Float64 depth_data;
geometry_msgs::Vector3 linear_acceleration;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 magnetic_field;
std_msgs::Float32MultiArray orientation;
geometry_msgs::Pose pose;
std_msgs::Int32MultiArray pwm_msg;

// void setThrusterThrottle(const int8_t *pwm_values);
void throttleCb(const std_msgs::Int32MultiArray& pwm_msg);

ros::NodeHandle nh;
ros::Subscriber <std_msgs::Int32MultiArray> sub("pwm_values", &throttleCb);
ros::Publisher pub1("linear_acceleration", &linear_acceleration);
ros::Publisher pub2("angular_velocity", &angular_velocity);
ros::Publisher pub3("magnetic_field", &magnetic_field);
ros::Publisher pub4("orientation", &orientation);
ros::Publisher pub5("depth_data", &depth_data);
// ros::Publisher pub6("pose", &pose);
MPU6050 gyro;
FXOS8700QBasic Magnetometer;
MS5837 Depth_Sensor;
Madgwick Filter;
Servo g_thrusters[NUMBER_OF_THRUSTERS];
const uint8_t g_kPinMap[NUMBER_OF_THRUSTERS] = {PA0, PA1, PA2, PA3, PA6, PB0, PA7};

void setup() 
{

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Serial.begin(9600);
  while(!Serial)
    delay(1);

  // bool status;
  // Wire.beginTransmission(0x68);
  // status=Wire.endTransmission(true);
  // if(status)void applyImuCalibration();"); 

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.advertise(pub3);
  nh.advertise(pub4);
  nh.advertise(pub5);
  // nh.advertise(pub6);
  initializeThrusters();
  delay(1000);
  initializeImu();
  initializeDepthSensor();
  calculateOffset();

  prev_time_update = millis();
  prev_time_publish = millis();
}

  
void loop() 
{
  if(millis() - prev_time_update >= UPDATE_RATE)
  {
    getSensorData();
    applyImuCalibration();   
    updateOrientation();
    // eulerToQuaternion();
    applyLowPassFilter();
    prev_time_update = millis();
  }

    if(millis() - prev_time_publish >= PUBLISH_RATE)
    {
    updatePublishData();
    pub1.publish(&linear_acceleration);
    pub2.publish(&angular_velocity);
    pub3.publish(&magnetic_field);
    pub4.publish(&orientation);
    pub5.publish(&depth_data);
    // pub6.publish(&pose);
    prev_time_publish = millis();
    }

    // Serial.print(roll_lpf);
    // Serial.print("/");
    // Serial.print(pitch_lpf);
    // Serial.print("/");
    // Serial.print(yaw_lpf);
    // Serial.print("raw_gx: ");
    // Serial.print(raw_g[0]);
    // Serial.print("raw_gy: ");
    // Serial.print(raw_g[1]);
    // Serial.print("raw_gz: ");
    // Serial.print(raw_g[2]);

    // Serial.print("raw_ax: ");
    // Serial.print(raw_a[0]);
    // Serial.print("raw_ay: ");
    // Serial.print(raw_a[1]);
    // Serial.print("raw_az: ");
    // Serial.print(raw_a[2]);

    // Serial.print("raw_mx: ");
    // Serial.print(raw_m[0]);
    // Serial.print("raw_my: ");
    // Serial.print(raw_m[1]);
    // Serial.print("raw_mz: ");
    // Serial.print(raw_m[2]);
    
    // Serial.println(" ");

    // Serial.print("cal_gx: ");
    // Serial.print(cal_g[0]);
    // Serial.print("cal_gy: ");
    // Serial.print(cal_g[1]);
    // Serial.print("cal_gz: ");
    // Serial.print(cal_g[2]);

    // Serial.print("cal_ax: ");
    // Serial.print(cal_a[0]);
    // Serial.print("cal_ay: ");
    // Serial.print(cal_a[1]);
    // Serial.print("cal_az: ");
    // Serial.print(cal_a[2]);

    // Serial.print("cal_mx: ");
    // Serial.print(cal_m[0]);
    // Serial.print("cal_my: ");
    // Serial.print(cal_m[1]);
    // Serial.print("cal_mz: ");
    // Serial.print(cal_m[2]);

    // Serial.println(" ");   
    
    // Serial.print("yaw:");
    // Serial.print(yaw);
    // Serial.print("pitch:");
    // Serial.print(pitch);
    // Serial.print("roll:");
    // Serial.print(roll);   
    // Serial.println(" ");

    // Serial.print("qx: ");
    // Serial.print(qx);
    // Serial.print("qy: ");
    // Serial.print(qy);
    // Serial.print("qz: ");
    // Serial.print(qz);
    // Serial.print("qw: ");
    // Serial.println(qw);

    nh.spinOnce();
    delay(10);
  
}

void initializeThrusters(){
  for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS; thruster_index++){
    g_thrusters[thruster_index].attach(g_kPinMap[thruster_index]);
    g_thrusters[thruster_index].writeMicroseconds(INIT_THRUSTER_PWM);
  }
}

// void setThrusterThrottle(const int8_t *pwm_values){
//   int pwm_value;
//   for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS; thruster_index++){
//     pwm_value = pwm_values[thruster_index];
//     for(int value=1474; value<pwm_values[thruster_index]; value+=10){
//       g_thrusters[thruster_index].writeMicroseconds(pwm_value);
//     }
//   }
// }

void throttleCb(const std_msgs::Int32MultiArray& pwm_msg){
    int pwm_value;
    for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS; thruster_index ++)
    {
        pwm_value = pwm_msg.data[thruster_index];
        g_thrusters[thruster_index].writeMicroseconds(pwm_value);
    }
}


void initializeImu()
{
  gyro.begin();
  gyro.setAccelerometerRange(ACCELERO_METER_RANGE_2);
  gyro.setGyroscopeRange(GYROSCOPE_RANGE_250);
  gyro.setSampleRateDivider(0);
  gyro.disableSleepMode(); 
  Magnetometer = FXOS8700QBasic(1, MAGNETOMETER_ADDRESS, &Wire);
}

void initializeDepthSensor()
{
  Depth_Sensor.init(&Wire);
  Depth_Sensor.setFluidDensity(997);
}

void calculateOffset()
{
  for (int  sample_no = 0; sample_no < NO_OF_SAMPLES ; sample_no++)
    {
         gyro.getSensorsReadings(raw_a[0], raw_a[1], raw_a[2], raw_g[0], raw_g[1], raw_g[2],false);
        _ax_offset += raw_a[0];
        _ay_offset += raw_a[1];
        _az_offset += raw_a[2];
        _gx_offset += raw_g[0];
        _gy_offset += raw_g[1];
        _gz_offset += raw_g[2];
        delay(50);
    }
    _ax_offset /= NO_OF_SAMPLES;
    _ay_offset /= NO_OF_SAMPLES;
    _az_offset /= NO_OF_SAMPLES;
    _gx_offset /= NO_OF_SAMPLES;
    _gy_offset /= NO_OF_SAMPLES;
    _gz_offset /= NO_OF_SAMPLES;
    _az_offset -= g;

}

void getSensorData()
{
    Depth_Sensor.read();
    depth = Depth_Sensor.depth();
    Magnetometer.updateMagData(raw_m);
    gyro.getSensorsReadings(raw_a[0], raw_a[1], raw_a[2], raw_g[0], raw_g[1], raw_g[2]);
}

void applyImuCalibration()
{
    cal_a[0]=raw_a[0]-_ax_offset;
    cal_a[1]=raw_a[1]-_ay_offset;
    cal_a[2]=raw_a[2]-_az_offset;

    cal_g[0]=raw_g[0]-_gx_offset;
    cal_g[1]=raw_g[1]-_gy_offset;
    cal_g[2]=raw_g[2]-_gz_offset;

    cal_m[0]=raw_m[0]-_mx_offset;
    cal_m[1]=raw_m[1]-_my_offset;
    cal_m[2]=raw_m[2]-_mz_offset;

    cal_m[0]=cal_m[0]*soft_iron_matrix[0][0]+cal_m[1]*soft_iron_matrix[0][1]+cal_m[2]*soft_iron_matrix[0][2];
    cal_m[1]=cal_m[0]*soft_iron_matrix[1][0]+cal_m[1]*soft_iron_matrix[1][1]+cal_m[2]*soft_iron_matrix[1][2];
    cal_m[2]=cal_m[0]*soft_iron_matrix[2][0]+cal_m[1]*soft_iron_matrix[2][1]+cal_m[2]*soft_iron_matrix[2][2];
}

void updateOrientation()
{
    Filter.update(cal_g[0], cal_g[1], cal_g[2], cal_a[0], cal_a[1], cal_a[2], cal_m[0], cal_m[1], cal_m[2]);
    yaw = Filter.getYaw();
    values[2]=yaw;
    pitch = Filter.getPitch();
    values[1]=pitch;
    roll = Filter.getRoll();
    values[0]=roll;

    prev_time_update = millis();
}

// void eulerToQuaternion()
// {
//     qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
//     qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
//     qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
//     qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
// }

void applyLowPassFilter()
{
    roll_lpf = 0.94 * roll_lpf + 0.06 * roll;
    pitch_lpf = 0.94 * pitch_lpf + 0.06 * pitch;
    yaw_lpf = 0.94 * yaw_lpf + 0.06 * yaw;
}

void updatePublishData()
{
    depth_data.data = depth;
    
    //publishing raw imu data
    linear_acceleration.x= raw_a[0];
    linear_acceleration.y= raw_a[1];
    linear_acceleration.z= raw_a[2];

    angular_velocity.x= raw_g[0];
    angular_velocity.y= raw_g[1];
    angular_velocity.z= raw_g[2];

    magnetic_field.x= raw_m[0];
    magnetic_field.y= raw_m[1];
    magnetic_field.z= raw_m[2];

    orientation.data_length = 4;
    orientation.data=values;

    // pose.position.x = roll_lpf;
    // pose.position.y = pitch_lpf;
    // pose.position.z = yaw_lpf;

    // pose.orientation.x=qx;
    // pose.orientation.y=qy;
    // pose.orientation.z=qz;
    // pose.orientation.w=qw;
}
