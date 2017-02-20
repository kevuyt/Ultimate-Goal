package BasicLib4997.Sensors;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * Created by Archish on 10/28/16.
 */

public class ODS implements Sensor_Thresholds {
    private OpticalDistanceSensor ods;
    private String nameODSSensor;

    public ODS(String name){
        this.nameODSSensor = name;
        ods = FtcOpModeRegister.opModeManager.getHardwareMap().opticalDistanceSensor.get(name);
    }
    public void enableLED() {
        ods.enableLed(true);
    }
    public void disableLED() {
        ods.enableLed(false);
    }
    public double lightDetected () {
        return ods.getLightDetected();
    }
    public double rawLight () {
        return ods.getRawLightDetected();
    }
    public double rawLightMax () {
        return ods.getLightDetected();
    }
    public boolean isWhite () {
        return lightDetected() <= 0.7;
    }
    public boolean isBlack () {
        return lightDetected() >= 0.3;
    }
    public void telemetryRun () {
        TankDrive.getTelemetry().addTelemetry(nameODSSensor);
        TankDrive.getTelemetry().addTelemetry("Light Detected", lightDetected());
        TankDrive.getTelemetry().addTelemetry("Raw Light", rawLight());
        TankDrive.getTelemetry().addTelemetry("Raw Light Max", rawLightMax());
    }
}
