package org.hitlabnz.sensor_fusion_demo.orientationProvider;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.util.Log;

/**
 * The orientation provider that delivers the current orientation from the {@link Sensor#TYPE_ROTATION_VECTOR Android
 * Rotation Vector sensor}.
 * 
 * @author Alexander Pacha
 * 
 */
public class RotationVectorProvider extends OrientationProvider {

    /**
     * Temporary quaternion to store the values obtained from the SensorManager
     */
    final private float[] temporaryQuaternion = new float[4];

    /**
     * Initialises a new RotationVectorProvider
     * 
     * @param sensorManager The android sensor manager
     */
    public RotationVectorProvider(SensorManager sensorManager) {
        super(sensorManager);

        //The rotation vector sensor that is being used for this provider to get device orientation
        sensorList.add(sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR));
        sensorList.add(sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY));
    }

    private float R[] = new float[16];
    private float RI[] = new float[16];
    private float accAxis[] = new float[4];
    private float acc[] = new float[4];
    @Override
    public void onSensorChanged(SensorEvent event) {
        // we received a sensor event. it is a good practice to check
        // that we received the proper event
        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            // convert the rotation-vector to a 4x4 matrix. the matrix
            // is interpreted by Open GL as the inverse of the
            // rotation-vector, which is what we want.
//            SensorManager.getRotationMatrixFromVector(currentOrientationRotationMatrix.matrix, event.values);

            // Get Quaternion
            // Calculate angle. Starting with API_18, Android will provide this value as event.values[3], but if not, we have to calculate it manually.

            SensorManager.getRotationMatrixFromVector(R, event.values);


            SensorManager.getQuaternionFromVector(temporaryQuaternion, event.values);
//            Log.i("RotVect", String.format("%f %f %f %f\n", temporaryQuaternion[0],
//                    temporaryQuaternion[1], temporaryQuaternion[2], temporaryQuaternion[3]));
            currentOrientationQuaternion.setXYZW(temporaryQuaternion[1], temporaryQuaternion[2],
                    temporaryQuaternion[3], -temporaryQuaternion[0]);
        } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
            System.arraycopy(event.values, 0, acc, 0, event.values.length);
            android.opengl.Matrix.multiplyMV(accAxis, 0, R,
                    0, acc, 0);
//            Log.i("RotVect", String.format("%f %f %f %f", accAxis[0], accAxis[1], accAxis[2], accAxis[3]));
        }
    }
}
