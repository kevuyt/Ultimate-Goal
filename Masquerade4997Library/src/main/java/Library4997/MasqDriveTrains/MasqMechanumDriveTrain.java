package Library4997.MasqDriveTrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqUtilities.MasqHardware;
import Library4997.MasqUtilities.MasqUtils;

/**
 * Created by Archish on 4/23/18.
 */

public class MasqMechanumDriveTrain extends MasqDriveTrain implements MasqHardware {
    public MasqMechanumDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        super(name1, name2, name3, name4, hardwareMap);
    }

    public MasqMechanumDriveTrain(HardwareMap hardwareMap) {
        super(hardwareMap);
    }
    public void setPowerAtAngle(double angle, double speed, double turnPower) {
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
        leftDrive.motor1.setVelocity(leftFront);
        leftDrive.motor2.setVelocity(leftBack);
        rightDrive.motor1.setVelocity(rightFront);
        rightDrive.motor2.setVelocity(rightBack);
    }
}
