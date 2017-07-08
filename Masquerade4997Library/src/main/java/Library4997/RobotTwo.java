package Library4997;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqTankDrive;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqColorSensor;
import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqSensors.MasqODS;
import Library4997.MasqServos.MasqCRServo;
import Library4997.MasqServos.MasqServo;

/**
 * Created by Archish on 6/17/17.
 */

public class RobotTwo {
    public MasqTankDrive driveTrain = new MasqTankDrive("leftFront", "leftBack", "rightFront", "rightBack");
}
