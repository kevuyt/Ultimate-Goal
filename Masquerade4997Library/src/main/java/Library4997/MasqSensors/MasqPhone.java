package Library4997.MasqSensors;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * Created by Archishmaan Peyyety on 5/18/18.
 * Project: MasqLib
 */

public class MasqPhone implements SensorEventListener {
    private SensorManager senSensorManager;
    private Sensor senAccelerometer;
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
}
