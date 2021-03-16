package MasqueradeLibrary.MasqSensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.*;

import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqMatiboxUltraSensor {
    private AnalogInput ds;
    private String name;
    private int scale = 225;
    private double stopThresh = 60;

    public MasqMatiboxUltraSensor(String name, HardwareMap hardwareMap){
        this.name = name;
        ds = hardwareMap.get(AnalogInput.class, name);
    }

    public double getDistance() {return ds.getVoltage() * scale;}
    public void setStopThresh(double thresh) {stopThresh = thresh;}
    public double getStopThresh() {return stopThresh;}
    public void setScale(int scale) {this.scale = scale;}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nDistance: %.2f", name, getDistance());
    }
}