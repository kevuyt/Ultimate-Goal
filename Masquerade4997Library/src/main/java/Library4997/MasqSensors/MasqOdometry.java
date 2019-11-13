package Library4997.MasqSensors;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;

/**
 * Created by Keval Kataria on 11/13/2019
 */
public class MasqOdometry implements MasqHardware {
    MasqMotor xSystem, ySystem;

    public MasqOdometry(MasqMotor xSystem, MasqMotor ySystem) {
        this.xSystem = xSystem;
        this.ySystem = ySystem;
    }




    @Override
    public String getName() {
        return "Odometry";
    }

    @Override
    public String[] getDash() {
        return new String[0];
    }
}
