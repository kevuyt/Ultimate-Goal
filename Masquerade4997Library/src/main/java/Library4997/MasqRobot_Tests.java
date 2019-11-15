package Library4997;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqWrappers.Tests;

/**
 * Created by Keval Kataria on 9/28/2019
 */
public class MasqRobot_Tests implements Tests {
    private MasqRobot robot = new MasqRobot() {

        @Override
        public void mapHardware(HardwareMap hardwareMap) {
            driveTrain = new MasqMechanumDriveTrain(hardwareMap);
            tracker = new MasqPositionTracker(driveTrain.leftDrive.motor1, driveTrain.rightDrive.motor1, hardwareMap);
        }

        @Override
        public void init(HardwareMap hardwareMap) {
            mapHardware(hardwareMap);
        }
    };

    void driveTest() {
        robot.drive(1);
    }

    @Override
    public void RunAll() {

    }

    @Override
    public void RunAll(HardwareMap hardwareMap) {
        robot.mapHardware(hardwareMap);
        driveTest();
    }
}
