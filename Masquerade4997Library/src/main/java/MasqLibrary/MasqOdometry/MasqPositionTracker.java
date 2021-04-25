package MasqLibrary.MasqOdometry;

import androidx.annotation.NonNull;

import MasqLibrary.MasqMotion.MasqDriveTrain;
import MasqLibrary.MasqMotion.MasqMotor;
import MasqLibrary.MasqResources.MasqUtils;
import MasqLibrary.MasqSensors.MasqIMU;

import static MasqLibrary.MasqResources.MasqUtils.adjustAngle;
import static java.lang.Math.*;
import static java.util.Locale.US;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqPositionTracker {
    private MasqMotor xSystem, yLSystem, yRSystem, ySystem;
    private MasqDriveTrain driveTrain;
    public MasqIMU imu;
    private double prevHeading;
    private double globalX, globalY, prevX, prevY, prevYR, prevYL, xRadius, yRadius, trackWidth;
    private DeadWheelPosition position;

    public enum DeadWheelPosition {
        TWO_CENTER("Center"), TWO_BACK_RIGHT("Two Right"), TWO_BACK_LEFT("Two Left"), THREE("Three"), TANK("Tank");
        String name;
        DeadWheelPosition(String name) {this.name = name;}
        public String getName() {return name;}
    }

    public MasqPositionTracker(MasqMotor xSystem, MasqMotor yLSystem, MasqMotor yRSystem) {
        this.xSystem = xSystem;
        this.yLSystem = yLSystem;
        this.yRSystem = yRSystem;
        imu = new MasqIMU();
        prevHeading = imu.getAbsoluteHeading();
        MasqUtils.setTracker(this);
        position = DeadWheelPosition.THREE;
        reset();
    }
    public MasqPositionTracker(MasqMotor xSystem, MasqMotor ySystem, boolean left) {
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        imu = new MasqIMU();
        prevHeading = imu.getAbsoluteHeading();
        MasqUtils.setTracker(this);
        if(left) position = DeadWheelPosition.TWO_BACK_LEFT;
        else position = DeadWheelPosition.TWO_BACK_RIGHT;
        reset();
    }
    public MasqPositionTracker(MasqDriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        imu = new MasqIMU();
        prevHeading = imu.getAbsoluteHeading();
        MasqUtils.setTracker(this);
        position = DeadWheelPosition.TANK;
        reset();
    }

    public double getHeading () {return imu.getHeading();}

    public void updateSystem () {
        switch (position) {
            case TWO_CENTER: twoCenter(); break;
            case TWO_BACK_RIGHT: twoPerpendicular(false); break;
            case TWO_BACK_LEFT: twoPerpendicular(true); break;
            case THREE: three(); break;
            case TANK: tank(); break;
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

    private void twoCenter() {
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
    private void twoPerpendicular(boolean left) {
        double heading = toRadians(getHeading());
        double xPosition = xSystem.getInches();
        double yPosition = ySystem.getInches();
        double dH = getDHeading();
        double dX = xPosition - prevX;
        prevX = xPosition;
        double dY = yPosition - prevY;
        prevY = yPosition;
        double angularComponentY = yRadius * dH;
        if(left) angularComponentY *= -1;
        double angularComponentX = xRadius * dH;
        double dTranslationalX = dX + angularComponentX;
        double dTranslationalY = dY + angularComponentY;
        double dGlobalX = dTranslationalX * cos(heading) + dTranslationalY * sin(heading);
        double dGlobalY = dTranslationalY * cos(heading) - dTranslationalX * sin(heading);
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

    private void tank() {
        double heading = toRadians(getHeading());
        double yPosition = driveTrain.getInches();
        double dY = yPosition - prevY;
        prevY = yPosition;
        double dGlobalX = dY * sin(heading);
        double dGlobalY = dY * cos(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    public double getDHeading() {
        double current = toRadians(getHeading());
        double change = (current - prevHeading);
        prevHeading = current;
        return adjustAngle(change, RADIANS);
    }

    public double getGlobalX() {return globalX;}
    public double getGlobalY() {return globalY;}

    public void setXRadius(double xRadius) {this.xRadius = xRadius;}
    public void setYRadius(double yRadius) {this.yRadius = yRadius;}
    public void setTrackWidth(double trackWidth) {this.trackWidth = trackWidth;}

    public DeadWheelPosition getPosition() {return position;}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "Tracker:\nGlobal X: %.2f\nGlobalY: %.2f\nHeading: %.2f\nMode: %s",
                globalX, globalY, getHeading(), position.getName());
    }
}