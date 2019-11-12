package Library4997;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqWrappers.Tests;

/**
 * Created by Keval Kataria on 9/28/2019
 */
public class MasqRobot_Tests implements Tests {
    MasqRobot robot = new MasqRobot() {
        BNO055IMU imu;

        @Override
        public void init(HardwareMap hardwareMap) {
            driveTrain = new MasqMechanumDriveTrain(hardwareMap);
            imu = initializeIMU(hardwareMap);
            tracker = new MasqPositionTracker(driveTrain.leftDrive.motor1, driveTrain.rightDrive.motor1,imu);
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
        robot.init(hardwareMap);
        driveTest();
    }
}
