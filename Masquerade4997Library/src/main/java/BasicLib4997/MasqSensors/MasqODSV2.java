package BasicLib4997.MasqSensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.MasqHardware;

/**
 * Created by Archish on 2/28/17.
 */

public class MasqODSV2 implements MasqHardware{
    private AnalogSensor ods;
    private String nameODS;

    public MasqODSV2(String name){
        this.nameODS = name;
        ods = FtcOpModeRegister.opModeManager.getHardwareMap().get(AnalogSensor.class, name);
    }
    public int getDistance() {
        return (int) ods.readRawVoltage() * 16;
    }
    public String getName() {
        return nameODS;
    }
    public String getDash() {
        return "Distance" + Double.toString(getDistance());
    }
}
























































