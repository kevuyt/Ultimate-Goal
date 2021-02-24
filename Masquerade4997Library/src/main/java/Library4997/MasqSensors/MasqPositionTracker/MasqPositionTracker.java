package Library4997.MasqSensors.MasqPositionTracker;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHardware;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqResources.DashBoard;
import Library4997.MasqUtils;

import static Library4997.MasqSensors.MasqClock.Resolution.SECONDS;
import static Library4997.MasqUtils.adjustAngle;
import static Library4997.MasqUtils.sleep;
import static java.lang.Math.*;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public class MasqPositionTracker implements MasqHardware {
    private MasqMotor xSystem;
    private MasqMotor yLSystem;
    private MasqMotor yRSystem;
    private MasqMotor ySystem;
    public MasqAdafruitIMU imu;
    private double prevHeading, xDrift, yDrift;
    private double globalX, globalY, prevX, prevY, prevYR, prevYL, xRadius, yRadius, trackWidth;
    private DeadWheelPosition position;
    private DashBoard dash = DashBoard.getDash();

    public enum DeadWheelPosition {
        BOTH_CENTER, BOTH_PERPENDICULAR, THREE
    }

    public MasqPositionTracker(MasqMotor xSystem, MasqMotor yLSystem, MasqMotor yRSystem, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.yLSystem = yLSystem;
        this.yRSystem = yRSystem;
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        prevHeading = imu.getAbsoluteHeading();
        MasqUtils.setTracker(this);
        reset();
    }
    public MasqPositionTracker(MasqMotor xSystem, MasqMotor ySystem, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        prevHeading = imu.getAbsoluteHeading();
        MasqUtils.setTracker(this);
        reset();
    }
    public MasqPositionTracker(HardwareMap hardwareMap) {
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        MasqUtils.setTracker(this);
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
        }
    }
    public void updateOverTime(double time) {
        MasqClock clock = new MasqClock();
        while (clock.hasNotPassed(time, SECONDS)) {
            updateSystem();
            dash.create(this);
            dash.update();
        }
    }

    public void reset() {
        if(xSystem != null) {
            xSystem.resetEncoder();
            xSystem.setWheelDiameter(2);
        }
        if(ySystem != null) {
            ySystem.resetEncoder();
            ySystem.setWheelDiameter(2);
        }
        if(yLSystem != null && yRSystem != null) {
            yLSystem.resetEncoder();
            yRSystem.resetEncoder();
            yLSystem.setWheelDiameter(2);
            yRSystem.setWheelDiameter(2);
        }

        imu.reset();

        globalX = 0;
        globalY = 0;
    }

    private void bothCenter() {
        double deltaX = (xSystem.getInches() - prevX);
        double deltaY = (ySystem.getInches() - prevY);
        double heading = toRadians(getHeading());
        double x = deltaX * cos(heading) - deltaY * sin(heading);
        double y = deltaX * sin(heading) + deltaY * cos(heading);
        globalX += x;
        globalY += y;
        prevY = ySystem.getInches();
        prevX = xSystem.getInches();
    }
    private void bothPerpendicular() {
        double heading = toRadians(getHeading());
        double xPosition = xSystem.getInches();
        double yPosition = ySystem.getInches();
        double dH = getDHeading();
        double dX = xPosition - prevX;
        prevX = xPosition;
        double dY = yPosition - prevY;
        prevY = yPosition;
        double angularComponentY = yRadius * dH;
        double angularComponentX = xRadius * dH;
        double dTranslationalX = dX - angularComponentX;
        double dTranslationalY = dY + angularComponentY;
        double dGlobalX = dTranslationalX * cos(heading) - dTranslationalY * sin(heading);
        double dGlobalY = dTranslationalX * sin(heading) + dTranslationalY * cos(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }
    private void three() {
        double heading = toRadians(getHeading());
        double xPosition = xSystem.getInches();
        double yLPosition = yLSystem.getInches();
        double yRPosition = yRSystem.getInches();
        double dX = xPosition - prevX;
        prevX = xPosition;
        double dYR = yRPosition - prevYR;
        prevYR = yRPosition;
        double dYL = yLPosition - prevYL;
        prevYL = yLPosition;
        double dH = (dYR - dYL) / trackWidth;
        double dTranslationalY = (dYR + dYL) / 2;
        double angularComponentX = xRadius * dH;
        double dTranslationalX = dX - angularComponentX;
        double dGlobalX = dTranslationalX * cos(heading) + dTranslationalY * sin(heading);
        double dGlobalY = dTranslationalY * cos(heading) - dTranslationalX * sin(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    public double getDHeading() {
        double current = toRadians(getHeading());
        double change = (current - prevHeading);
        prevHeading = current;
        sleep(10);
        return adjustAngle(change, RADIANS);
    }

    public double getGlobalX() {
        return globalX + xDrift;
    }
    public double getGlobalY() {
        return globalY + yDrift;
    }

    public void setXRadius(double xRadius) {this.xRadius = xRadius;}
    public void setYRadius(double yRadius) {this.yRadius = yRadius;}
    public void setTrackWidth(double trackWidth) {this.trackWidth = trackWidth;}

    public void setPosition(MasqPositionTracker.DeadWheelPosition position) {this.position = position;}

    @Override
    public String getName() {return "Tracker";}

    @Override
    public String[] getDash() {
        return new String[] {
                "GlobalX: " + globalX,
                "GlobalY: " + globalY,
                "Heading: " + getHeading(),
        };
    }
}