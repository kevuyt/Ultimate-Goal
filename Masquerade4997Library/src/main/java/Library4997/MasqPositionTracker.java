package Library4997;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqSensors.MasqAdafruitIMU;

import static Library4997.MasqResources.MasqUtils.adjustAngle;
import static Library4997.MasqResources.MasqUtils.sleep;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public abstract class MasqPositionTracker implements MasqHardware {
    public MasqMotor xSystem, ySystem;
    public MasqAdafruitIMU imu;
    private double prevHeading;
    private double globalX = 0, globalY = 0, prevX = 0, prevY = 0, prevYR = 0, prevYL = 0, xRadius = 0, yRadius = 0, trackWidth = 0;

    public enum DeadWheelPosition {
        BOTH_CENTER, BOTH_PERPENDICULAR, THREE
    }

    public MasqPositionTracker(MasqMotor xSystem, MasqMotor ySystem, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        prevHeading = imu.getAbsoluteHeading();
        reset();
    }
    public MasqPositionTracker(MasqMotor xSystem, MasqMotor ySystem, String imuName, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        imu = new MasqAdafruitIMU(imuName, hardwareMap);
        reset();
    }
    public double getHeading () {
        return imu.getRelativeYaw();
    }
    public void updateSystem (DeadWheelPosition position) {
        switch (position) {
            case BOTH_CENTER: bothCenter(); break;
            case BOTH_PERPENDICULAR: bothPerpendicular(); break;
            case THREE: three(); break;
            default: break;
        }
    }

    public void reset() {
        xSystem.resetEncoder();
        ySystem.resetEncoder();
        imu.reset();
    }

    private void bothCenter() {
        double deltaX = (getXPosition() - prevX);
        double deltaY = (getYPosition() - prevY);
        double heading = Math.toRadians(getHeading());
        double x = deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        double y = deltaX * Math.sin(heading) + deltaY * Math.cos(heading);
        globalX += x;
        globalY += y;
        prevY = getYPosition();
        prevX = getXPosition();
    }

    private void bothPerpendicular() {
        double heading = Math.toRadians(getHeading());
        double xPosition = getXPosition();
        double yPosition = getYPosition();
        double dH = Math.toRadians(getDHeading(heading));
        double dX = xPosition - prevX;
        prevX = xPosition;
        double dY = yPosition - prevY;
        prevY = yPosition;
        double angularComponentY = yRadius * dH;
        double angularComponentX = xRadius * dH;
        double dTranslationalX = dX - angularComponentX;
        double dTranslationalY = dY + angularComponentY;
        double dGlobalX = dTranslationalX * Math.cos(heading) - dTranslationalY * Math.sin(heading);
        double dGlobalY = dTranslationalX * Math.sin(heading) + dTranslationalY * Math.cos(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    private void three() {
        double heading = Math.toRadians(getHeading());
        double xPosition = getXPosition();
        double yRPosition = getYPosition();
        double yLPosition = getYPosition();
        double dX = xPosition - prevX;
        prevX = xPosition;
        double dYR = yRPosition - prevYR;
        prevYR = yRPosition;
        double dYL = yLPosition - prevYL;
        prevYL = yLPosition;
        double dH = (dYR - dYL) / (2 * trackWidth);
        double dTranslationalY = (yRPosition + yLPosition) / 2;
        double angularComponentX = xRadius * dH;
        double dTranslationalX = dX - angularComponentX;
        double dGlobalX = dTranslationalX * Math.cos(heading) - dTranslationalY * Math.sin(heading);
        double dGlobalY = dTranslationalX * Math.sin(heading) + dTranslationalY * Math.cos(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    public double getDHeading(double current) {
        double change = current - prevHeading;
        prevHeading = current;
        sleep(10);
        return adjustAngle(Math.toDegrees(change));
    }

    public double getGlobalX() {
        return globalX;
    }
    public double getGlobalY() {
        return globalY;
    }

    public abstract double getXPosition();
    public abstract double getYPosition();

    public void setXRadius(double xRadius) {
        this.xRadius = xRadius;
    }

    public void setYRadius(double yRadius) {
        this.yRadius = yRadius;
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
