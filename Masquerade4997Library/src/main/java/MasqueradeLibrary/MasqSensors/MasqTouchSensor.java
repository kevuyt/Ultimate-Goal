package MasqueradeLibrary.MasqSensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqTouchSensor {
    private final DigitalChannel limitSwitch;
    private final String name;

    public MasqTouchSensor(String name, HardwareMap hardwareMap) {
        this.name = name;
        limitSwitch = hardwareMap.digitalChannel.get(name);
        limitSwitch.setMode(INPUT);
    }
    public boolean isPressed() {return limitSwitch.getState();}

    @NonNull
    @Override
    public String toString() {return String.format("%s:\nPressed: %b", name, isPressed());}
}
