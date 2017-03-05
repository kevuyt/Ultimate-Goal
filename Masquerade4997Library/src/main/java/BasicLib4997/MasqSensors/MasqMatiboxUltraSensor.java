package BasicLib4997.MasqSensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.MasqHardware;

/**
 * Created by Archish on 2/28/17.
 */

public class MasqMatiboxUltraSensor implements MasqHardware{
        private AnalogInput ds;
        private String nameDS;

        public MasqMatiboxUltraSensor(String name){
            this.nameDS = name;
            ds = FtcOpModeRegister.opModeManager.getHardwareMap().analogInput.get(name);
        }
        public double getData() {
            return ds.getVoltage();
        }
        public String getName() {
            return nameDS;
        }
        public String getDash() {
            return "Distance" + Double.toString(getData());
        }
}
























































