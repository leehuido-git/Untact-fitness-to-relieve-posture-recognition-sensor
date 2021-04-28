//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

#define GYRO_SAMPLES_IN_RADS_PER_SEC

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
#ifdef GYRO_SAMPLES_IN_RADS_PER_SEC
    void update(float gRadsPerSecx, float gRadsPerSecy, float gRadsPerSecz, float ax, float ay, float az, float mx, float my, float mz);
#else
    void update(float gDegsPerSecx, float gDegsPerSecx, float gDegsPerSecx, float ax, float ay, float az, float mx, float my, float mz);
#endif
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    //float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    //float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    //float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
    float getq0() {
//        if (!anglesComputed)
//        return (int)((q0+1)*100);
        return q0;
    }
    float getq1() {
//        if (!anglesComputed)
//        return (int)((q1+1)*100);
        return q1;
    }
    float getq2() {
//        if (!anglesComputed)
//        return (int)((q2+1)*100);
        return q2;
    }
    float getq3() {
//        if (!anglesComputed)
//        return (int)((q3+1)*100);
        return q3;
    }
};
#endif

