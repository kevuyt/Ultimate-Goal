package Library4997.MasqControlSystems.MasqPurePursuit;

import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqSensors.MasqAdafruitIMU;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public class MasqPositionTracker {
    private MasqMotorSystem lSystem, rSystem;
    public MasqAdafruitIMU imu;
    private double deltaY;
    private double previousTime = 0;
    private double prevY = 0;
    private double globalX = 0, globalY = 0;

    public MasqPositionTracker(MasqMotorSystem lSystem, MasqMotorSystem rSystem, MasqAdafruitIMU imu) {
        this.imu = imu;
        this.lSystem = lSystem;
        this.rSystem = rSystem;
        this.lSystem.resetEncoders();
        this.rSystem.resetEncoders();
    }


    public double getLeftInches () {
        return lSystem.getCurrentPosition() /
                lSystem.motor1.getEncoder().getClicksPerInch();
    }

    public double getRightInches () {
        return rSystem.getCurrentPosition() /
                rSystem.motor1.getEncoder().getClicksPerInch();
    }

    public double getRelativeYInches() {
        return (getLeftInches() + getRightInches()) / 2;
    }

    public double getHeading () {
        return imu.getAbsoluteHeading();
    }

    public double getGlobalX () {
        return globalX;
    }

    public double getGlobalY () {
        return globalY;
    }

    public void updateSystem () {
        double tChange = System.nanoTime() - previousTime;
        deltaY = (getRelativeYInches() - prevY) / tChange;
        double heading = getHeading();
        globalX += (Math.sin(Math.toRadians(heading)) * deltaY) * tChange;
        globalY += (Math.cos(Math.toRadians(heading)) * deltaY) * tChange;
        prevY = getRelativeYInches();
        previousTime = System.nanoTime();
    }
}
