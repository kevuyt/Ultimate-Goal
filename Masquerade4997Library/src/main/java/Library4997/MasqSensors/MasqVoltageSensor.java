package Library4997.MasqSensors;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RollingAverage;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import java.util.ArrayList;
import java.util.List;

public class MasqVoltageSensor {
    VoltageSensor sensor;
    List<Double> average;

    public MasqVoltageSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.voltageSensor.iterator().next();
        average = new ArrayList<>();
    }

    public void update() {average.add(getVoltageInstantaneous());}

    public double getVoltage() {
        double sum =0, num = 0;
        for (Double sample: average){num++; sum += sample;}
        return sum/num;
    }

    public double getVoltageInstantaneous() {return sensor.getVoltage();}
}