package Library4997.MasqExternal.MasqExceptions;

import Library4997.MasqExternal.MasqHardware;

/**
 * Created by Archish on 10/10/17.
 */

public class DashBoardException extends Exception {
    public DashBoardException (MasqHardware hardware){
        super(hardware.getName() + "failed to have dash");
    }
}
