package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import team4997lib.MasqOpMode;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name = "Test: Infinite Loop (Iterative)", group = "Tests")
public class NeutTeleOp extends MasqOpMode {


    static final double SCALEDOWN = 0.15;
    double shooterPower = 0;
    boolean rightOpenPressed = false;
    boolean leftOpenPressed = false;
    boolean sweepPressed = false;
    boolean shooterPressed = false;
    boolean swervePressed = false;
    boolean switchPressed = false;
    boolean slowPressed = false;
    boolean hookPressed = false;
    double sweepPower = 0;
    int counter = 0;
    double finalPosition;


    public NeutTeleOp() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        robot.leftPresser.setPosition(1);
        robot.rightPresser.setPosition(.03);
        float right = -controller1.left_stick_y();
        float left = controller1.right_stick_y();

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);


        float speedCheck = controller1.left_trigger();

        speedCheck = Range.clip(speedCheck, 0, 1);

        if (speedCheck > 0.5) {
            robot.driveTrain.setPowerRight(right * SCALEDOWN);
            robot.driveTrain.setPowerLeft(left * SCALEDOWN);
        } else {
            robot.driveTrain.setPowerRight(-1 * right);
            robot.driveTrain.setPowerLeft(-1 * left);
        }
        if (controller2.b()) {
            robot.indexer.setPosition(.3);

            waitNow(400);
            robot.indexer.setPosition(0);

        }
        if (controller2.dpad_down() && !sweepPressed) {
            sweepPower = 0.9;
            sweepPressed = true;
        } else if (controller2.dpad_up() && !sweepPressed) {
            sweepPower = -0.9;
            sweepPressed = true;
        } else if (controller2.dpad_left() && sweepPressed) {
            sweepPower = 0;
            sweepPressed = false;
        }
        robot.collector.setPower(sweepPower);

        if (controller1.dpad_down() && !sweepPressed) {
            sweepPower = 1;
            sweepPressed = true;
        } else if (controller1.dpad_up() && !sweepPressed) {
            sweepPower = -1;
            sweepPressed = true;
        } else if (controller1.dpad_left() && sweepPressed) {
            sweepPower = 0;
            sweepPressed = false;
        }
        robot.collector.setPower(sweepPower);

        if (controller2.right_bumper()) {
            shooterPower = .8;
        } else if (controller2.left_bumper()) {
            shooterPower = 0;
        }
        robot.shooter2.setPower(shooterPower);
        robot.shooter1.setPower(shooterPower);
    }

    public void waitNow(long waitTime)
    {
        try {
            Thread.sleep(waitTime);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();

        }


    }
}