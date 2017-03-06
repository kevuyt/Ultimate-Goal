package BasicLib4997.MasqSensors;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import BasicLib4997.MasqHardware;

public class MasqTouch implements MasqHardware {
    TouchSensor touchSensor;
    String nameTouchSensor;
    public MasqTouch(String name){
        this.nameTouchSensor = name;
        touchSensor = FtcOpModeRegister.opModeManager.getHardwareMap().touchSensor.get(name);
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
    public String getDash() {
        return "IsPressed" + Boolean.toString(isPressed());
    }
}