package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqUtilities.MasqHardware;
import Library4997.MasqUtilities.MasqSensor;


/**
 * Created by Archish on 2/28/17.
 */

public class MasqMatiboxUltraSensor implements MasqHardware, MasqSensor {
    private AnalogInput ds;
    private String nameDS;
    private int scale = 225;
    private double stopThresh = 60;
    public MasqMatiboxUltraSensor(String name, HardwareMap hardwareMap){
            this.nameDS = name;
            ds = hardwareMap.get(AnalogInput.class, name);
        }
    public double getDistance() {return ds.getVoltage() * scale;}
    public void setStopThresh(int thresh) {
        stopThresh = thresh;
    }
    public String getName() {
            return nameDS;
        }
    public String[] getDash() {
            return new String[]{
                    "Distance " + Double.toString(getDistance())
            };
    }

    @Override
    public boolean stop() {
        return getDistance() > stopThresh;
    }
}