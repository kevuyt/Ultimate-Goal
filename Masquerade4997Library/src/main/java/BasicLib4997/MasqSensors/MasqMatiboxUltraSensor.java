package BasicLib4997.MasqSensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.MasqHardware;

/**
 * Created by Archish on 2/28/17.
 */

public class MasqMatiboxUltraSensor implements MasqHardware{
        private AnalogSensor ds;
        private String nameDS;

        public MasqMatiboxUltraSensor(String name){
            this.nameDS = name;
            ds = FtcOpModeRegister.opModeManager.getHardwareMap().get(AnalogSensor.class, name);
        }
        public double getDistance() {
            return ds.readRawVoltage();
        }
        public String getName() {
            return nameDS;
        }
        public String[] getDash() {
            return new String[]{
                    "Distance" + Double.toString(getDistance())
            };
        }
}
























































