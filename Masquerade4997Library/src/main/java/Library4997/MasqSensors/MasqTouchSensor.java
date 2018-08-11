package Library4997.MasqSensors;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqResources.MasqHelpers.MasqHardware;

public class MasqTouchSensor implements MasqHardware {
    private DigitalChannel touchSensor;
    private String nameTouchSensor;
    public MasqTouchSensor(String name, HardwareMap hardwareMap){
        this.nameTouchSensor = name;
        touchSensor = hardwareMap.digitalChannel.get(name);
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }
    public boolean isPressed () {
        return touchSensor.getState();
    }

    public String getName() {
        return nameTouchSensor;
    }
    public String[] getDash() {
        return new String[]{
                "Pressed: " + Boolean.toString(isPressed())
        };
    }


    public boolean stop() {
        return !isPressed();
    }
}