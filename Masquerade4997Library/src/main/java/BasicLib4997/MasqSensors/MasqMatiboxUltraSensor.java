package BasicLib4997.MasqSensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.MasqHardware;
import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Created by Archish on 2/28/17.
 */

public class MasqMatiboxUltraSensor implements MasqHardware{
        private AnalogInput ds;
        private String nameODSSensor;

        public MasqMatiboxUltraSensor(String name){
            this.nameODSSensor = name;
            ds = FtcOpModeRegister.opModeManager.getHardwareMap().analogInput.get(name);
        }
        public double getData() {
            return ds.getVoltage();
        }
        public String getName() {
            return nameODSSensor;
        }
        public String getDash() {
            return "Distance" + Double.toString(getData());
        }
}
























































