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
@TeleOp(name = "CRATER_MECH", group = "NFS")
public class CRATER_MECH extends MasqLinearOpMode implements Constants {

    private Resurrection resurrection = new Resurrection();

    private boolean bOverride = true, aOverride = true;

    private MasqPIDController speedController = new MasqPIDController(0.03, 0.0, 0.00001);

    private double maxAutoPositioningSpeed = 0.7;

    private MasqVector scorePosition = new MasqVector(0, 0);
    private double scoreHeading = 0;

    private MasqVector collectPosition = new MasqVector(0, 0);
    private double collectHeading = 0;

    private MasqVector current = new MasqVector(0, 0);
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
            resurrection.MECH(controller1);

            if (controller1.leftTriggerPressed()) resurrection.collector.setPower(.5);
            else if (controller1.leftBumper()) resurrection.collector.setPower(-.5);
            else resurrection.collector.setPower(0);


            if (controller1.x()) resurrection.collectorDumper.setPower(-1);
            else if (controller1.y()) resurrection.collectorDumper.setPower(1);
            else resurrection.collectorDumper.setPower(0);

            resurrection.collectionLift.DriverControl(controller1);

            aOverride =
                    controller1.leftTriggerPressed() || controller1.leftBumper()
                    || controller1.rightBumper() || controller1.rightTriggerPressed()
                    || controller1.x() || controller1.y();

            if (controller2.b()) resurrection.particleDumper.setPosition(PARTICLE_DUMPER_OUT);
            else resurrection.particleDumper.setPosition(PARTICLE_DUMPER_IN);

            if (controller2.leftStickY() < 0 && resurrection.hangTopSwitch.isPressed()) resurrection.hang.setPower(-1);
            else if (controller2.leftStickY() > 0 && resurrection.hangBottomSwitch.isPressed()) resurrection.hang.setPower(1);
            else resurrection.hang.setBreakMode();

            resurrection.scoreLift.DriverControl(controller2);

            bOverride = controller2.b() || Math.abs(controller2.leftStickY()) > 0 || controller2.leftBumper()
                    || controller2.leftTriggerPressed();

            if (controller2.x() && !bOverride) automateScoring();

            if (controller1.b() && !aOverride) gotoScore();

            resurrection.tracker.updateSystem();
            dash.update();
        }
    }
    public void gotoScore() {
        MasqVector displacement = current.displacement(scorePosition);
        double speed = speedController.getOutput(displacement.getMagnitude());
        if (Math.abs(speed) > maxAutoPositioningSpeed) speed = maxAutoPositioningSpeed;
        double pathAngle = 90 - Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX()));
        resurrection.driveTrain.setPowerMECH(pathAngle + resurrection.tracker.getHeading(), speed, scoreHeading);
        if (!resurrection.collectionLiftSwitch.isPressed()) resurrection.collectionLift.lift.setPower(-1);
        else if (!resurrection.collectionDumpTopSwitch.isPressed()) resurrection.collectorDumper.setPower(-1);
    }
    public void automateScoring() {

    }
    public void robotInit() {
        resurrection.setStartOpenCV(false);
        resurrection.mapHardware(hardwareMap);
        resurrection.initializeTeleop();
        resurrection.hang.setClosedLoop(true);
        resurrection.driveTrain.setClosedLoop(true);
        resurrection.particleDumper.setPosition(PARTICLE_DUMPER_IN);
    }
}
