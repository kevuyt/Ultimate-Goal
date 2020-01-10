package Library4997.MasqSensors.MasqPositionTracker;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqPositionTracker.MasqDeadwheel.Measurement;

/**
 * Created by Archishmaan Peyyety on 2020-01-08.
 * Project: MasqLib
 */
public class MasqPositionTracker {
    private List<MasqDeadwheel> wheels = new ArrayList<>();
    private double globalX, globalY, prevHeading;
    private MasqAdafruitIMU imu;
    private int xWheels, yWheels;

    public MasqPositionTracker(HardwareMap hardwareMap) {
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        prevHeading = imu.getAbsoluteHeading();
        reset();
    }
    public MasqPositionTracker(String imuName, HardwareMap hardwareMap) {
        imu = new MasqAdafruitIMU(imuName, hardwareMap);
        reset();
    }

    public void updateSystem() {
        double heading = Math.toRadians(imu.getRelativeYaw());
        double dX, dY, dH, angularComponentX, angularComponentY;
        dX = dY = dH = angularComponentY = angularComponentX = 0;
        if (getWheelsType(Measurement.X).size() > 1) {

        } else {

        }
        if (getWheelsType(Measurement.Y).size() > 1) {
            double dW1 = getWheelsType(Measurement.Y).get(1).getVelocity();
            double dW2 = getWheelsType(Measurement.Y).get(2).getVelocity();
        } else {

        }
        double dTranslationalX = dX - angularComponentX;
        double dTranslationalY = dY + angularComponentY;
        double dGlobalX = dTranslationalX * Math.cos(heading) - dTranslationalY * Math.sin(heading);
        double dGlobalY = dTranslationalX * Math.sin(heading) + dTranslationalY * Math.cos(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    public void reset() {
        for (MasqDeadwheel masqDeadwheel : wheels) {
            masqDeadwheel.reset();
        }
        imu.reset();
    }
    public double getGlobalX() {
        return globalX;
    }

    public double getGlobalY() {
        return globalY;
    }

    public List<MasqDeadwheel> getWheelsType(Measurement m) {
        List<MasqDeadwheel> deadwheels = new ArrayList<>();
        for (MasqDeadwheel masqDeadwheel : wheels) {
            if (masqDeadwheel.getMeasurement() == m) deadwheels.add(masqDeadwheel);
        }
        return deadwheels;
    }

    public void addWheel(MasqDeadwheel deadwheel) {
        if (deadwheel.getMeasurement() == MasqDeadwheel.Measurement.X) xWheels++;
        else yWheels++;
        wheels.add(deadwheel);
    }
}
