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
    private double globalX, globalY, prevX = 0, prevY = 0;

    public MasqPositionTracker(MasqMotor xSystem, MasqMotor ySystem, MasqAdafruitIMU imu) {
        this.imu = imu;
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        this.xSystem.resetEncoder();
        this.ySystem.resetEncoder();
        imu.reset();
    }
    public double getHeading () {
        return imu.getRelativeYaw();
    }

    public void updateSystem () {
        double deltaX = (-xSystem.getCurrentPosition() - prevX);
        double deltaY = (-ySystem.getCurrentPosition() - prevY);
        double heading = Math.toRadians(imu.getRelativeYaw());
        double x = deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        double y = deltaX * Math.sin(heading) + deltaY * Math.cos(heading);
        globalX += x;
        globalY += y;
        prevY = -ySystem.getCurrentPosition();
        prevX = -xSystem.getCurrentPosition();
    }

    public void reset() {
        xSystem.resetEncoder();
        ySystem.resetEncoder();
        imu.reset();
    }

    public double getGlobalX() {
        return globalX * ((2 * Math.PI) / 1440);
    }
    public double getGlobalY() {
        return globalY * ((2 * Math.PI) / 1440);
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
