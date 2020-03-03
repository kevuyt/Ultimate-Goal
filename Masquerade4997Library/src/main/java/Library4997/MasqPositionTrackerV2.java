package Library4997;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqSensors.MasqAdafruitIMU;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public class MasqPositionTrackerV2 implements MasqHardware {
    private MasqMotor xSystem, yLSystem, yRSystem;
    public MasqAdafruitIMU imu;
    private double heading, globalX, globalY;
    private boolean running;

    private double prevX, prevYR, prevYL, xRadius, trackWidth;

    public MasqPositionTrackerV2(MasqMotor xSystem, MasqMotor yLSystem, MasqMotor yRSystem, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.yLSystem = yLSystem;
        this.yRSystem = yRSystem;
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        reset();
    }

    public double getHeading () {
        return imu.getRelativeYaw();
    }

    public void reset() {
        xSystem.resetEncoder();
        yLSystem.resetEncoder();
        yRSystem.resetEncoder();
        imu.reset();
    }

    public void updateSystem() {
        three();
    }

    private void three() {
        double xPosition = xSystem.getInches();
        double yLPosition = yLSystem.getInches();
        double yRPosition = yRSystem.getInches();
        double heading = (yLPosition - yRPosition) / trackWidth;
        double dX = xPosition - prevX;
        prevX = xPosition;
        double dYR = yRPosition - prevYR;
        prevYR = yRPosition;
        double dYL = yLPosition - prevYL;
        prevYL = yLPosition;
        double dH = (dYL - dYR) / trackWidth;
        double dTranslationalY = (dYR + dYL) / 2;
        double angularComponentX = xRadius * dH;
        double dTranslationalX = dX - angularComponentX;
        double dGlobalX = dTranslationalX * Math.cos(heading) - dTranslationalY * Math.sin(heading);
        double dGlobalY = dTranslationalX * Math.sin(heading) + dTranslationalY * Math.cos(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    public double getGlobalX() {
        return globalX;
    }
    public double getGlobalY() {
        return globalY;
    }

    public void setXRadius(double xRadius) {
        this.xRadius = xRadius;
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public void setRunning(boolean running) {
        this.running = running;
    }

    @Override
    public String getName() {
        return "Tracker";
    }

    @Override
    public String[] getDash() {
        return new String[] {
                getName() +
                "GlobalX: " + globalX,
                "GlobalY: " + globalY,
                "Heading: " + getHeading(),
        };
    }
}
