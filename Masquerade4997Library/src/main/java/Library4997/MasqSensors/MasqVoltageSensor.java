package Library4997.MasqSensors;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RollingAverage;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import java.util.ArrayList;
import java.util.List;

public class MasqVoltageSensor {
    VoltageSensor sensor;
    double sum;
    double count;

    public MasqVoltageSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.voltageSensor.iterator().next();
        sum = count = 0;
    }

    public void update() {sum+=getVoltageInstantaneous(); count++; }

    public double getVoltage() {
        return sum/count;
    }

    public double getVoltageInstantaneous() {return sensor.getVoltage();}
}
