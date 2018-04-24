package SubSystems4997.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqEncoder;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqSensors.MasqAdafruitIMU;

/**
 * Created by Archish on 4/22/18.
 */

public class PositionTracker {
    public MasqAdafruitIMU imu;
    public MasqEncoder yWheel, xWheel;
    private double xStart = 0, xEnd = 0, ignoreXTicks = 0;
    private double yStart = 0, yEnd = 0, ignoreYTicks = 0;
    HardwareMap hardwareMap;
    public PositionTracker (HardwareMap hardwareMap, MasqMotor yWheelMotor, double yPPR, MasqMotor xWheelMotor, double xPPR) {
        this.hardwareMap = hardwareMap;
        imu = new MasqAdafruitIMU("imuHubOne", this.hardwareMap);
        yWheel = new MasqEncoder(yWheelMotor, yPPR);
        xWheel = new MasqEncoder(xWheelMotor, xPPR);
    }
    public double getX () {
        return xWheel.getPosition() - ignoreXTicks;
    }
    public double getY () {
        return yWheel.getPosition() - ignoreYTicks;
    }
    public double getRotation () {
        return imu.getRelativeYaw();
    }
    public void resetSystem () {
        xWheel.resetEncoder();
        yWheel.resetEncoder();
        imu.reset();
    }
    public void startIgnoringRotation () {
        xStart = getX();
        yStart = getY();
    }
    public void endIgnoringRotation () {
        xEnd = getX();
        yEnd = getY();
        ignoreXTicks = ignoreXTicks + (xEnd - xStart);
        ignoreYTicks = ignoreYTicks + (yEnd - yStart);
    }
}
