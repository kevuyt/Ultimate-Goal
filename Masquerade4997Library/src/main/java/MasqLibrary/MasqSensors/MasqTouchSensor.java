package MasqLibrary.MasqSensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.TouchSensor;

import static MasqLibrary.MasqResources.MasqUtils.getHardwareMap;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqTouchSensor {
    private TouchSensor touchSensor;
    private String name;

    public MasqTouchSensor(String name) {
        this.name = name;
        touchSensor = getHardwareMap().get(TouchSensor.class, name);
    }

    public boolean isPressed() {return touchSensor.isPressed();}

    @NonNull
    @Override
    public String toString() {return String.format("%s:\nPressed: %b", name, isPressed());}
}