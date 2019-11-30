package org.firstinspires.ftc.teamcode.Robots.TestRobot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@TeleOp(name = "LibraryTestsTeleop", group = "Test")
public class LibraryTestsTeleop extends MasqLinearOpMode {
    private TestRobot robot = new TestRobot();
    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }
        waitForStart();

        //No Drivetrain on robot so can't run tests
        /*MasqMotor_Tests masqMotorTests = new MasqMotor_Tests(robot.driveTrain.leftDrive.motor1) {
            @Override
            public void RunAll(HardwareMap hardwareMap) {

            }
        };
        MasqPIDController_Tests pidControllerTests = new MasqPIDController_Tests();
        while (opModeIsActive()) {
            try {
                masqMotorTests.RunAll();
                pidControllerTests.RunAll();
            }
            catch (AssertionError error)  {
                dash.create(error);
            }
        }
        dash.update();*/
    }
}