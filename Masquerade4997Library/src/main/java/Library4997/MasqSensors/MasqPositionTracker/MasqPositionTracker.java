package Library4997.MasqSensors.MasqPositionTracker;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqSensors.MasqAdafruitIMU;

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
        double dX, dY, angularComponentX, angularComponentY;
        dX = dY = angularComponentY = angularComponentX = 0;
        if (wheels.size() == 4) {

        } else if (wheels.size() == 3) {
            if (xWheels > yWheels) {

            } else {

            }
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

    public void addWheel(MasqDeadwheel deadwheel) {
        if (deadwheel.getMeasurement() == MasqDeadwheel.Measurement.X) xWheels++;
        else yWheels++;
        wheels.add(deadwheel);
    }
}
