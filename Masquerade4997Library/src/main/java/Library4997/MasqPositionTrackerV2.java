package Library4997;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqSensors.MasqAdafruitIMU;

import static Library4997.MasqResources.MasqUtils.adjustAngle;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public abstract class MasqPositionTrackerV2 implements MasqHardware {
    public enum WheelPosition {
        TOP, BOTTOM, LEFT, RIGHT, CENTER
    }
    int xWheels = 0, yWheels = 0;
    public MasqMotor systemXR, systemXL, systemYR, systemYL;
    public MasqAdafruitIMU imu;
    private double globalX = 0, globalY = 0,
            prevXR = 0,     prevXL,         prevYR = 0,     prevYL = 0, prevHeading,
            radiusXR = 0,   radiusXL = 0,   radiusYR = 0,   radiusYL = 0,
            trackWidth = 0;

    private WheelPosition positionXR, positionXL, positionYR, positionYL;
    public MasqPositionTrackerV2(HardwareMap hardwareMap) {
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        prevHeading = imu.getAbsoluteHeading();
        reset();
    }
    public MasqPositionTrackerV2(String imuName, HardwareMap hardwareMap) {
        imu = new MasqAdafruitIMU(imuName, hardwareMap);
        reset();
    }
    public void updateSystem () {
        double heading = Math.toRadians(getHeading());
        double dH = 0;
        if (xWheels + yWheels == 2) dH = Math.toRadians(getDHeading(heading));
        else if (xWheels == 2 && systemXR != null && systemXL != null) {
            dH = (systemXR.getCurrentPosition() - systemXL.getCurrentPosition()) / trackWidth;
        } else if (yWheels == 2 && systemYR != null && systemYL != null) {
            dH = (systemYR.getCurrentPosition() - systemYL.getCurrentPosition()) / trackWidth;
        }
        /*double dTranslationalX = dX - angularComponentX;
        double dTranslationalY = dY + angularComponentY;*/
        /*double dGlobalX = dTranslationalX * Math.cos(heading) - dTranslationalY * Math.sin(heading);
        double dGlobalY = dTranslationalX * Math.sin(heading) + dTranslationalY * Math.cos(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;*/
    }

    public double getDHeading(double current) {
        double change = current - prevHeading;
        prevHeading = current;
        return adjustAngle(Math.toDegrees(change));
    }

    public double getHeading () {
        return imu.getRelativeYaw();
    }

    public double getGlobalX() {
        return globalX;
    }
    public double getGlobalY() {
        return globalY;
    }

    public void addWheel(WheelPosition position) {

    }

    public abstract double getXPosition();
    public abstract double getYPosition();
    public abstract void reset();

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
