package Library4997.MasqSensors;


import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RollingAverage;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

public class MasqVoltageSensor {
    public final static int samples = 2000000;
    VoltageSensor sensor;
    RollingAverage average;

    public MasqVoltageSensor() {
        sensor = FtcOpModeRegister.opModeManager.getHardwareMap().voltageSensor.iterator().next();
        average = new RollingAverage(samples);

    }

    public void update() {
        average.addNumber(getVoltageInstantaneous());
    }

    public double getVoltage() {
        return average.getAverage();
    }

    public int getVoltageInstantaneous() {
        return (int) sensor.getVoltage();
    }
}