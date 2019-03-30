package Library4997.MasqControlSystems.MasqPurePursuit;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqSensors.MasqAdafruitIMU;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public class MasqPositionTracker implements MasqHardware {
    private MasqMotor xSystem, ySystem;
    public MasqAdafruitIMU imu;
    private double globalX, globalY, prevXSystem = 0, prevYSystem = 0;

    public MasqPositionTracker(MasqMotor xSystem, MasqMotor ySystem, MasqAdafruitIMU imu) {
        this.imu = imu;
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        this.xSystem.resetEncoder();
        this.ySystem.resetEncoder();
    }

    public double getHeading () {
        return imu.getRelativeYaw();
    }

    public void updateSystem () {
        double deltaXSystem = (xSystem.getCurrentPosition() - prevXSystem);
        double deltaYSystem = (-ySystem.getCurrentPosition() - prevYSystem);
        double heading = imu.getRelativeYaw();
        double ySystemXAddition = Math.sin(Math.toRadians(heading)) * deltaYSystem;
        double ySystemYAddition = Math.cos(Math.toRadians(heading)) * deltaYSystem;
        double xSystemXAddition = Math.cos(Math.toRadians(heading)) * deltaXSystem;
        double xSystemYAddition = Math.sin(Math.toRadians(heading)) * deltaXSystem;
        globalX += ySystemXAddition + xSystemXAddition;
        globalY += ySystemYAddition + xSystemYAddition;
        prevYSystem = -ySystem.getCurrentPosition();
        prevXSystem = xSystem.getCurrentPosition();
    }

    public double getGlobalX() {
        return globalX;
    }

    public double getGlobalY() {
        return globalY;
    }

    @Override
    public String getName() {
        return "Tracker";
    }

    @Override
    public String[] getDash() {
        return new String[] {

        };
    }
}
