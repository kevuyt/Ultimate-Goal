package Library4997.MasqSensors;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import Library4997.MasqExternal.MasqHardware;
import Library4997.MasqExternal.MasqSensor;

public class MasqTouchSensor implements MasqHardware, MasqSensor {
    TouchSensor touchSensor;
    String nameTouchSensor;
    public MasqTouchSensor(String name, HardwareMap hardwareMap){
        this.nameTouchSensor = name;
        touchSensor = hardwareMap.touchSensor.get(name);
    }
    public boolean isPressed () {
        return touchSensor.isPressed();
    }
    public double getValue () {
        return touchSensor.getValue();
    }

    public String getName() {
        return nameTouchSensor;
    }
    public String[] getDash() {
        return new String[]{
                "IsPressed" + Boolean.toString(isPressed())
        };
    }

    public boolean stop() {
        return !isPressed();
    }
}