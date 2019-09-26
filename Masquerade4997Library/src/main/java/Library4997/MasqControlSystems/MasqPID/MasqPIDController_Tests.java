package Library4997.MasqControlSystems.MasqPID;

import Library4997.MasqWrappers.Tests;

/**
 * Created by Keval Kataria on 9/25/2019
 */
public class MasqPIDController_Tests implements Tests {
    private MasqPIDController pidController = new MasqPIDController(0.1,0.1,0.1);

    private void getOutputTest() {
        if (pidController.getOutput(1,.1) != 1) throw new AssertionError();
    }

    @Override
    public void RunAll() {
        getOutputTest();
    }
}
