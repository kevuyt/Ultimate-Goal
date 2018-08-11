package Library4997.MasqControlSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqEncoder;
import Library4997.MasqResources.MasqHelpers.MasqEncoderModel;
import Library4997.MasqResources.MasqHelpers.MasqHardware;

/**
 * Created by Archish on 4/22/18.
 */

public class MasqPositionTracker implements MasqHardware {
    public MasqAdafruitIMU imu;
    public MasqEncoder yWheel, xWheel, leftWheel, rightWheel;
    HardwareMap hardwareMap;
    public MasqPositionTracker(HardwareMap hardwareMap, MasqMotor yWheelMotor, MasqEncoderModel yPPR, MasqMotor xWheelMotor, MasqEncoderModel xPPR) {
        this.hardwareMap = hardwareMap;
        imu = new MasqAdafruitIMU("imu", this.hardwareMap);
        yWheel = new MasqEncoder(yWheelMotor, yPPR);
        xWheel = new MasqEncoder(xWheelMotor, xPPR);
    }

    public void addLeftWheel (MasqEncoderModel l, MasqMotor lw) {
        leftWheel = new MasqEncoder(lw, l);
    }

    public void addRightWheel (MasqEncoderModel r, MasqMotor rw) {
        rightWheel = new MasqEncoder(rw, r);
    }

    public double getX () {
        return xWheel.getRelativePosition();
    }
    public double getY () {
        return yWheel.getRelativePosition();
    }
    public double getXInches () {
        return xWheel.getInches();
    }
    public double getYInches () {
        return yWheel.getInches();
    }
    public double getRotation () {
        return imu.getRelativeYaw();
    }
    public void resetSystem () {
        xWheel.resetEncoder();
        yWheel.resetEncoder();
        imu.reset();
    }
    @Override
    public String getName() {
        return "Position Tracker";
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "X "+ getX(),
                "Y " + getY(),
                "θ " + getRotation()
        };
    }
}
