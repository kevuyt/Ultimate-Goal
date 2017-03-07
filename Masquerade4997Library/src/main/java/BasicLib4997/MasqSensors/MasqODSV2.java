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
    private int scale;
    private int defaultScale = 200;
    public MasqODSV2(String name){
        this.nameODS = name;
        this.scale = defaultScale;
        ods = FtcOpModeRegister.opModeManager.getHardwareMap().get(AnalogSensor.class, name);
    }
    public MasqODSV2(String name, int scale){
        this.nameODS = name;
        this.scale = scale;
        ods = FtcOpModeRegister.opModeManager.getHardwareMap().get(AnalogSensor.class, name);
    }
    public int getDistance() {
        return (int) (ods.readRawVoltage() * scale);
    }
    public String getName() {
        return nameODS;
    }
    public String[] getDash() {
       return new String[]{ "Distance" + Double.toString(getDistance())};
    }
}
























































