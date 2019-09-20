package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;

import Library4997.MasqWrappers.Tests;

/**
 * Created by Keval Kataria on 8/21/2019
 */
public class MasqMotor_Tests implements Tests {
    static MasqMotor masqMotor;
    public double velocity;
    public MasqMotor_Tests(MasqMotor m){
        masqMotor = m;
    }


    public static void RunWithoutEncoderTest() {
        masqMotor.runWithoutEncoders();
        if (masqMotor.motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            throw new AssertionError("RunWithoutEncoderTest failed");
    }
    public static void RunUsingEncoderTest() {
        masqMotor.runUsingEncoder();
        if (masqMotor.motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
            throw new AssertionError("RunUsingEncoderTest failed");
    }
    public static void setDistanceTest() {
        masqMotor.setDistance(10);
        if (masqMotor.destination != 10) throw new AssertionError("setDistanceTest failed");
    }
    public static void setBreakModeTest() {
        masqMotor.setBreakMode();
        if (masqMotor.motor.getPower() != 0) throw new AssertionError("setBreakModeTest1 Failed");
        if (masqMotor.motor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
            throw new AssertionError("setBreakModeTest2 Failed");
    }
    public static void unBreakModeTest() {
        masqMotor.unBreakMode();
        if (masqMotor.motor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT)
            throw new AssertionError("unBreakModeTest Failed");
    }
    public static void getCurrentPositionTest() {
        if (masqMotor.getCurrentPosition() != masqMotor.encoder.getRelativePosition())
            throw new AssertionError("getCurrentPositionTest Failed");
    }
    public static void getAbsolutePositionTest() {
        if (masqMotor.getAbsolutePosition() != masqMotor.motor.getCurrentPosition())
            throw new AssertionError("getAbsolutePositionTest Failed");
    }
    public static void getVelocityTest() {
        if (!(masqMotor.getVelocity(10, 1e9, 5) == 120)) throw new AssertionError();
    }
    public static void getAngleTest() {
        if (masqMotor.getAngle(30, 12) != 1) throw new AssertionError();
    }
    public static void setPowerTest() {
        if (masqMotor.setPower(0.5) != 0.5) throw new AssertionError();
    }
    public static void setVelocityTest() {
        if (masqMotor.setVelocity(0.5, 5, 100) != 1) throw new AssertionError();
    }
    public static void getAccelerationTest() {
        if (masqMotor.getAcceleration(10, 1e9) != 10) throw new AssertionError();
    }

    @Override
    public void RunAll() {
        RunWithoutEncoderTest();
        RunUsingEncoderTest();
        setDistanceTest();
        setBreakModeTest();
        unBreakModeTest();
        getCurrentPositionTest();
        getAbsolutePositionTest();
        getVelocityTest();
        getAngleTest();
        setPowerTest();
        setVelocityTest();
        getAccelerationTest();
    }
}
