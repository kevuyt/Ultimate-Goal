package MasqueradeLibrary.MasqSensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import static MasqueradeLibrary.MasqResources.MasqUtils.*;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqODS {
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
        return lightDetected() <= ODS_WHITE;
    }
    public boolean isBlack () {
        return lightDetected() >= ODS_BLACK;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nLightDetected: %.2f\nRaw Light: %.2f\nIs Black: %b\nIsWhite: %b", nameODSSensor, lightDetected(), rawLight(), isBlack(), isWhite());
    }
}
