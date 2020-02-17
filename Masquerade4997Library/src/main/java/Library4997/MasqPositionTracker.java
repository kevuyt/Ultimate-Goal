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

public class MasqPositionTracker implements MasqHardware {
    private MasqMotor xSystem, yLSystem, yRSystem, ySystem;
    public MasqAdafruitIMU imu;
    private double prevHeading, heading;

    private double globalX, globalY, prevX, prevY, prevYR, prevYL, xRadius, yRadius, trackWidth;
    private DeadWheelPosition position;

    public enum DeadWheelPosition {
        BOTH_CENTER, BOTH_PERPENDICULAR, THREE
    }

    public MasqPositionTracker(MasqMotor xSystem, MasqMotor yLSystem, MasqMotor yRSystem, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.yLSystem = yLSystem;
        this.yRSystem = yRSystem;
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
    public MasqPositionTracker(MasqMotor xSystem, MasqMotor ySystem, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        prevHeading = imu.getAbsoluteHeading();
        reset();
    }
    public MasqPositionTracker(MasqMotor xSystem, MasqMotor yLSystem, MasqMotor yRSystem, String imuName, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.yLSystem = yLSystem;
        this.yRSystem = yRSystem;
        imu = new MasqAdafruitIMU(imuName, hardwareMap);
        reset();
    }
    public MasqPositionTracker(HardwareMap hardwareMap) {
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        imu.reset();
    }

    public double getHeading () {
        return imu.getRelativeYaw();
    }
    public void updateSystem () {
        switch (position) {
            case BOTH_CENTER: bothCenter(); break;
            case BOTH_PERPENDICULAR: bothPerpendicular(); break;
            case THREE: three(); break;
            default: break;
        }
    }

    public void reset() {
        xSystem.resetEncoder();
        yLSystem.resetEncoder();
        yRSystem.resetEncoder();
        imu.reset();
    }

    private void bothCenter() {
        double deltaX = (xSystem.getInches() - prevX);
        double deltaY = (ySystem.getInches() - prevY);
        double heading = Math.toRadians(getHeading());
        double x = deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        double y = deltaX * Math.sin(heading) + deltaY * Math.cos(heading);
        globalX += x;
        globalY += y;
        prevY = ySystem.getInches();
        prevX = xSystem.getInches();
    }

    private void bothPerpendicular() {
        double heading = Math.toRadians(getHeading());
        double xPosition = xSystem.getInches();
        double yPosition = ySystem.getInches();
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
        double xPosition = xSystem.getInches();
        double yLPosition = yLSystem.getInches();
        double yRPosition = yRSystem.getInches();
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

    private void threev2() {
        double xPosition = xSystem.getInches();
        double yLPosition = yLSystem.getInches();
        double yRPosition = yRSystem.getInches();
        double dX = xPosition - prevX;
        prevX = xPosition;
        double dYR = yRPosition - prevYR;
        prevYR = yRPosition;
        double dYL = yLPosition - prevYL;
        prevYL = yLPosition;
        double dH = (dYL - dYR) / trackWidth;
        heading += dH;
        double dTranslationalY = (dYR + dYL) / 2;
        double angularComponentX = xRadius * dH;
        double dTranslationalX = dX - angularComponentX;
        double dGlobalX = dTranslationalX * Math.cos(heading) - dTranslationalY * Math.sin(heading);
        double dGlobalY = dTranslationalX * Math.sin(heading) + dTranslationalY * Math.cos(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    public double getDHeading(double current) {
        double change = (current - prevHeading);
        prevHeading = current;
        sleep(10);
        return adjustAngle(Math.toDegrees(change));
    }

    public void setYRadius(double yRadius) {
        this.yRadius = yRadius;
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

    public void setPosition(MasqPositionTracker.DeadWheelPosition position) {
        this.position = position;
    }

    @Override
    public String getName() {
        return "Tracker";
    }

    @Override
    public String[] getDash() {
        return new String[] {
            "GlobalX: " + globalX,
            "GlobalY: " + globalY,
            "Heading: " + getHeading(),
        };
    }
}
