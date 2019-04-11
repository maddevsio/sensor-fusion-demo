package org.hitlabnz.sensor_fusion_demo.orientationProvider;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import org.hitlabnz.sensor_fusion_demo.representation.MatrixF4x4;

/**
 * Created by lezh1k on 1/5/18.
 */

public class MadgwickProvider extends OrientationProvider {

    private float gain;
    private float sampleFreq;
    private float qW, qX, qY, qZ; //quaternion

    private float acc[] = new float[4];
    private float gyr[] = new float[4];

    /**
     * Initialises a new OrientationProvider
     *
     * @param sensorManager The android sensor manager
     */
    public MadgwickProvider(SensorManager sensorManager,
                            float gain,
                            float sampleFreq) {
        super(sensorManager);
        this.gain = gain;
        this.sampleFreq = sampleFreq;
        qW = 1.0f;
        qX = 0.0f;
        qY = 0.0f;
        qZ = 1.0f;
        sensorList.add(sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER));
        sensorList.add(sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE));
    }

    private float invSqrt(float x) {
        return (float) (1.0f / Math.sqrt(x));
    }

    public void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-qX * gx - qY * gy - qZ * gz);
        qDot2 = 0.5f * (qW * gx + qY * gz - qZ * gy);
        qDot3 = 0.5f * (qW * gy - qX * gz + qZ * gx);
        qDot4 = 0.5f * (qW * gz + qX * gy - qY * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * qW;
            _2q1 = 2.0f * qX;
            _2q2 = 2.0f * qY;
            _2q3 = 2.0f * qZ;
            _4q0 = 4.0f * qW;
            _4q1 = 4.0f * qX;
            _4q2 = 4.0f * qY;
            _8q1 = 8.0f * qX;
            _8q2 = 8.0f * qY;
            q0q0 = qW * qW;
            q1q1 = qX * qX;
            q2q2 = qY * qY;
            q3q3 = qZ * qZ;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * qX - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * qY + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * qZ - _2q1 * ax + 4.0f * q2q2 * qZ - _2q2 * ay;
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= gain * s0;
            qDot2 -= gain * s1;
            qDot3 -= gain * s2;
            qDot4 -= gain * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        qW += qDot1 * (1.0f / sampleFreq);
        qX += qDot2 * (1.0f / sampleFreq);
        qY += qDot3 * (1.0f / sampleFreq);
        qZ += qDot4 * (1.0f / sampleFreq);

        // Normalise quaternion
        recipNorm = invSqrt(qW * qW + qX * qX + qY * qY + qZ * qZ);
        qW *= recipNorm;
        qX *= recipNorm;
        qY *= recipNorm;
        qZ *= recipNorm;
    }

    boolean init0 = false;
    boolean init1 = false;

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch(event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                System.arraycopy(event.values, 0, acc, 0, 3);
                init0 = true;
                break;
            case Sensor.TYPE_GYROSCOPE:
                System.arraycopy(event.values, 0, gyr, 0, 3);
                init1 = true;
                break;
            default:
                return;
        }

        if (init0 && init1) {
            MadgwickAHRSupdateIMU(gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2]);
            currentOrientationQuaternion.setXYZW(qX, qY, qZ, -qW); //-q for cube rotation inversion
            init0 = init1 = false;
        }
    }
}
