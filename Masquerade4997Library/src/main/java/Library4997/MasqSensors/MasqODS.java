package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqUtils;


/**
 * Created by Archish on 10/28/16.
 */

public class MasqODS implements MasqHardware {
    private OpticalDistanceSensor ods;
    private String nameODSSensor;

    public MasqODS(String name, HardwareMap hardwareMap){
        this.nameODSSensor = name;
        ods = hardwareMap.opticalDistanceSensor.get(name);
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
    public boolean isWhite () {
        return lightDetected() <= MasqUtils.ODS_WHITE;
    }
    public boolean isBlack () {
        return lightDetected() >= MasqUtils.ODS_BLACK;
    }
    public String getName() {
        return nameODSSensor;
    }
    public String[] getDash() {
        return new String[]{
                "Light Detected" + lightDetected(),
                "Raw Ligh" + Double.toString(rawLight()),
                "is Black:" + Boolean.toString(isBlack()),
                "IS White:" + Boolean.toString(isWhite())
        };
    }
}
