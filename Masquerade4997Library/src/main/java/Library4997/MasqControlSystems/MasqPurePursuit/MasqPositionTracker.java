package Library4997.MasqControlSystems.MasqPurePursuit;

import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqSensors.MasqAdafruitIMU;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public class MasqPositionTracker implements MasqHardware {
    private MasqMotorSystem lSystem, rSystem;
    public MasqAdafruitIMU imu;
    private double deltaY;
    private double xAddition, yAddition;
    private double prevXAddition = 0, prevYAddition = 0;
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
        return lSystem.getInches();
    }

    public double getRightInches () {
        return rSystem.getInches();
    }

    public double getRawYInches() {
        return (getLeftInches() + getRightInches()) / 2;
    }

    public double getHeading () {
        return imu.getRelativeYaw();
    }

    public double getGlobalX () {
        return -globalX;
    }

    public double getGlobalY () {
        return globalY;
    }

    public void reset() {
        imu.reset();
        lSystem.resetEncoders();
        rSystem.resetEncoders();
    }

    public MasqPoint getPosition() {
        return new MasqPoint(getGlobalX(), getGlobalY());
    }

    public void updateSystem () {
        double tChange = System.nanoTime() - previousTime;
        deltaY = (getRawYInches() - prevY) / tChange;
        double heading = getHeading();
        globalX += (Math.sin(Math.toRadians(heading)) * deltaY) * tChange;
        globalY += (Math.cos(Math.toRadians(heading)) * deltaY) * tChange;
        prevY = getRawYInches();
        previousTime = System.nanoTime();
    }
    public void updateSystemV2 () {
        double tChange = System.nanoTime() - previousTime;
        deltaY = (getRawYInches() - prevY) / tChange;
        double heading = getHeading();
        xAddition = (Math.sin(Math.toRadians(heading)) * deltaY);
        yAddition = (Math.cos(Math.toRadians(heading)) * deltaY);
        globalX += tChange * (prevXAddition + (0.5 * (xAddition - prevXAddition)));
        globalY += tChange * (prevYAddition + (0.5 * (yAddition - prevYAddition)));
        prevY = deltaY;
        previousTime = System.nanoTime();
        prevXAddition = xAddition;
        prevYAddition = yAddition;
    }

    @Override
    public String getName() {
        return "Tracker";
    }

    @Override
    public String[] getDash() {
        return new String[] {
                "X: " + Double.toString(getGlobalX()),
                "Y: " + Double.toString(getGlobalY())
        };
    }
}
