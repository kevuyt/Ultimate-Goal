package Library4997;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqWrappers.Tests;

/**
 * Created by Keval Kataria on 9/28/2019
 */
public class MasqRobot_Tests implements Tests {
    MasqRobot masqRobot = new MasqRobot() {
        @Override
        public void mapHardware(HardwareMap hardwareMap) {
            driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        }

        @Override
        public MasqPIDPackage pidPackage() {
            return new MasqPIDPackage();
        }
    };

    @Override
    public void RunAll() {

    }

    @Override
    public void RunAll(HardwareMap hardwareMap) {

    }
}
