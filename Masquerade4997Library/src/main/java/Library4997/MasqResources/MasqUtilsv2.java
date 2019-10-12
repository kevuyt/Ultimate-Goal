package Library4997.MasqResources;

import com.qualcomm.robotcore.util.Range;

import Library4997.MasqControlSystems.MasqPID.MasqPIDConstants;
import Library4997.MasqResources.MasqHelpers.API_KEYS;

/**
 * Created by Keval Kataria on 10/11/2019
 */
public class MasqUtilsv2 implements API_KEYS {
    private MasqPIDConstants turnConstants;
    private MasqPIDConstants driveConstants;

    public void setTurnConstants(int kP, int kI, int kD) {
        turnConstants = new MasqPIDConstants(kP, kI, kD);
    }
    public void setDriveConstants(int kP, int kI, int kD) {
        driveConstants = new MasqPIDConstants(kP, kI, kD);
    }

    public MasqPIDConstants getTurnConstants() {
        return turnConstants;
    }

    public MasqPIDConstants getDriveConstants() {
        return driveConstants;
    }
}