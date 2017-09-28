package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import Library4997.MasqHardware;
import Library4997.MasqSensor;

/**
 * Created by Archish on 2/28/17.
 */

public class MasqMatiboxUltraSensor implements MasqHardware, MasqSensor {
        private AnalogInput ds;
        private String nameDS;
    private int stopThresh = 30;
    public MasqMatiboxUltraSensor(String name, HardwareMap hardwareMap){
            this.nameDS = name;
            ds = hardwareMap.get(AnalogInput.class, name);
        }
    public double getDistance() {
            return ds.getVoltage();
        }
    public void setStopThresh(int thresh) {
        stopThresh = thresh;
    }
    public String getName() {
            return nameDS;
        }
    public String[] getDash() {
            return new String[]{
                    "Distance" + Double.toString(getDistance())
            };
    }

    @Override
    public boolean stop() {
        return getDistance() > stopThresh;
    }
}
























































