package org.firstinspires.ftc.teamcode.Robots.Reserection.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqResources.MasqMath.MasqVector;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/27/18.
 * Project: MasqLib
 */
@TeleOp(name = "MECH_AUTO", group = "NFS")
public class MECH_AUTO extends MasqLinearOpMode implements Constants {
    private Resurrection resurrection = new Resurrection();
    private String status = "";
    private double maxSpeed = 0.7;
    private MasqPIDController speedController = new MasqPIDController(0.03, 0.0, 0.00001);
    private double scoreExtension = -2800;
    private double collectionExtension = -3500;
    private double scoreRotation = -1302;
    private double midRotation = -600;
    private double collectRotation = 300;
    private double targetHeading = 0;
    private MasqVector target = new MasqVector(0, 0);
    private MasqVector current = new MasqVector(0, 0);
    private double prevRotatorPosition = 0;
    @Override
    public void runLinearOpMode()  {
        robotInit();
        current = new MasqVector(resurrection.tracker.getGlobalX(), resurrection.tracker.getGlobalY());
        while (!opModeIsActive()) {
            resurrection.tracker.updateSystem();
            dash.create("X: ", resurrection.tracker.getGlobalX());
            dash.create("Y: ", resurrection.tracker.getGlobalY());
            dash.create("H: ", resurrection.tracker.getHeading());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            if (controller1.leftTriggerPressed()) resurrection.collector.setPower(-.5);
            else if (controller1.leftBumper() || controller2.b()) resurrection.collector.setPower(.5);
            else resurrection.collector.setPower(0);

            if (controller2.b()) resurrection.particleDumper.setPosition(PARTICLE_DUMPER_OUT);
            else resurrection.particleDumper.setPosition(PARTICLE_DUMPER_IN);

            if (controller2.leftStickY() < 0 && !resurrection.hangTopSwitch.isPressed()) resurrection.hang.setPower(-1);
            else if (controller2.leftStickY() > 0 && !resurrection.hangBottomSwitch.isPressed()) resurrection.hang.setPower(1);
            else resurrection.hang.setBreakMode();

            if (controller1.x()) {
                target = new MasqVector(resurrection.tracker.getGlobalX(), resurrection.tracker.getGlobalY());
                targetHeading = resurrection.tracker.getHeading();
            }
        }
    }
    public void robotInit() {
        resurrection.setStartOpenCV(false);
        resurrection.mapHardware(hardwareMap);
        resurrection.initializeTeleop();
        resurrection.hang.setClosedLoop(true);
        resurrection.driveTrain.setClosedLoop(true);
        resurrection.particleDumper.setPosition(PARTICLE_DUMPER_IN);
    }
    public void updateDrivetrainToPath() {
        MasqVector displacement = current.displacement(target);
        double speed = speedController.getOutput(displacement.getMagnitude());
        if (Math.abs(speed) > maxSpeed) speed = maxSpeed * (speed / Math.abs(speed));
        double pathAngle = 90 - Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX()));
        resurrection.driveTrain.setPowerMECH(pathAngle + resurrection.tracker.getHeading(), speed, targetHeading);
        dash.create("Speed: ", speed);
    }
}
