#ifndef MADGWICK_AHRS
#define MADGWICK_AHRS

typedef struct madgwickAhrs_t 
{
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	            // quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
} madgwickAhrs_t;

void madgwickInit(madgwickAhrs_t *sensor, float beta, float freq);
float getRoll(madgwickAhrs_t *sensor);
float getPitch(madgwickAhrs_t *sensor);
float getYaw(madgwickAhrs_t *sensor);
float getRollRadians(madgwickAhrs_t *sensor);
float getPitchRadians(madgwickAhrs_t *sensor);
float getYawRadians(madgwickAhrs_t *sensor);
void madgwickUpdateIMU(madgwickAhrs_t *sensor, float gx, float gy, float gz, float ax, float ay, float az);
void madgwickUpdate(madgwickAhrs_t *sensor, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#endif //MADGWICK_AHRS