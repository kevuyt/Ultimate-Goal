package Library4997.MasqDriveTrains;

import com.qualcomm.robotcore.hardware.HardwareMap;
import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqResources.MasqUtils;


public class MasqMechanumDriveTrain extends MasqDriveTrain implements MasqHardware {
    MasqPIDController turnController = new MasqPIDController(MasqUtils.KP.TURN);
    MasqPositionTracker tracker;
    public MasqMechanumDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        super(name1, name2, name3, name4, hardwareMap);
    }
    public MasqMechanumDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap, MasqMotorModel masqMotorModel) {
        super(name1, name2, name3, name4, hardwareMap, masqMotorModel);
    }
    public MasqMechanumDriveTrain(HardwareMap hardwareMap){
        super(hardwareMap);
    }
    public MasqMechanumDriveTrain(HardwareMap hardwareMap,
            MasqMotorModel motorModel){
        super(hardwareMap, motorModel);
    }

    public void setPowerMECH(double angle, double speed, double targetHeading, double output) {
        double turnPower = turnController.getOutput(tracker.getHeading(), targetHeading);
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (Math.sin(adjustedAngle) * speed * MasqUtils.MECH_DRIVE_MULTIPLIER) - turnPower * MasqUtils.MECH_ROTATION_MULTIPLIER;
        double leftBack = (Math.cos(adjustedAngle) * speed * MasqUtils.MECH_DRIVE_MULTIPLIER) - turnPower  * MasqUtils.MECH_ROTATION_MULTIPLIER;
        double rightFront = (Math.cos(adjustedAngle) * speed * MasqUtils.MECH_DRIVE_MULTIPLIER) + turnPower * MasqUtils.MECH_ROTATION_MULTIPLIER;
        double rightBack = (Math.sin(adjustedAngle) * speed * MasqUtils.MECH_DRIVE_MULTIPLIER) + turnPower * MasqUtils.MECH_ROTATION_MULTIPLIER;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        leftDrive.motor1.setVelocity(leftFront - output);
        leftDrive.motor2.setVelocity(leftBack - output);
        rightDrive.motor1.setVelocity(rightFront + output);
        rightDrive.motor2.setVelocity(rightBack + output);
    }

    public MasqPositionTracker getTracker() {
        return tracker;
    }

    public void setTracker(MasqPositionTracker tracker) {
        this.tracker = tracker;
    }
}
