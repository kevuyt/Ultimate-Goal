package SubSystems4997.SubSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqServos.MasqCRServoSystem;
import Library4997.MasqUtilities.MasqHardware;
import Library4997.MasqWrappers.MasqController;
import SubSystems4997.MasqSubSystem;

/**
 * Created by Archish on 2/14/18.
 */

public class Intake implements MasqSubSystem {
    private MasqCRServoSystem intakeExtender;
    private MasqMotorSystem intake;
    public Intake (HardwareMap hardwareMap) {
        intakeExtender = new MasqCRServoSystem("redExtender", CRServo.Direction.REVERSE, "blueExtender", CRServo.Direction.FORWARD, hardwareMap);
        intake = new MasqMotorSystem("leftIntake", DcMotor.Direction.REVERSE, "rightIntake", DcMotor.Direction.FORWARD, "INTAKE", hardwareMap);
    }
    public enum Power {
        OUTAKE(new double[]{-1, -.75}),
        INTAKE(new double[]{1, .75}),
        STOP(new double[]{0, 0});
        public final double[] power;
        Power (double[] power) {this.power = power;}
    }
    public void setPower (Power power) {
        intake.setPower(power.power[0]);
        intakeExtender.setPower(power.power[1]);
    }
    @Override
    public void DriverControl(MasqController controller) {

    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[]{intakeExtender, intake};
    }
}
