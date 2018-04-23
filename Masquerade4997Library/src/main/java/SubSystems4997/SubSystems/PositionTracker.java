package SubSystems4997.SubSystems;

import Library4997.MasqMotors.MasqEncoder;
import Library4997.MasqSensors.MasqAdafruitIMU;

/**
 * Created by Archish on 4/22/18.
 */

public class PositionTracker {
    public MasqAdafruitIMU imu;
    public MasqEncoder yWheel, xWheel;
    public PositionTracker () {

    }
}
