package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqWrappers.Tests;

/**
 * Created by Keval Kataria on 8/21/2019
 */
 public class MasqMotor_Tests implements Tests {
    private MasqMotor masqMotor;
     public MasqMotor_Tests(MasqMotor masqMotor){this.masqMotor = masqMotor;}


     private void RunWithoutEncoderTest() {
        masqMotor.runWithoutEncoders();
        if (masqMotor.motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            throw new AssertionError("RunWithoutEncoderTest failed");
    }
     private void RunUsingEncoderTest() {
        masqMotor.runUsingEncoder();
        if (masqMotor.motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
            throw new AssertionError("RunUsingEncoderTest failed");
    }
     private void setDistanceTest() {
        masqMotor.setDistance(10);
        if (masqMotor.destination != 10) throw new AssertionError("setDistanceTest failed");
    }
     private void setBreakModeTest() {
        masqMotor.setBreakMode();
        if (masqMotor.motor.getPower() != 0) throw new AssertionError("setBreakModeTest1 Failed");
        if (masqMotor.motor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE)
            throw new AssertionError("setBreakModeTest2 Failed");
    }
     private void unBreakModeTest() {
        masqMotor.unBreakMode();
        if (masqMotor.motor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT)
            throw new AssertionError("unBreakModeTest Failed");
    }
     private void getCurrentPositionTest() {
        if (masqMotor.getCurrentPosition() != masqMotor.encoder.getRelativePosition())
            throw new AssertionError("getCurrentPositionTest Failed");
    }
     private void getAbsolutePositionTest() {
        if (masqMotor.getAbsolutePosition() != masqMotor.motor.getCurrentPosition())
            throw new AssertionError("getAbsolutePositionTest Failed");
    }
     private void getVelocityTest() {
        if (!(masqMotor.getVelocity(10, 1e9, 5) == 120)) throw new AssertionError("getVelocityTest Failed");
    }
     private void getAngleTest() {
        if (masqMotor.getAngle(30, 12) != 1) throw new AssertionError("getAngleTest Failed");
    }
     private void setPowerTest() {
        if (masqMotor.setPower(0.5) != 0.5) throw new AssertionError("setPowerTest Failed");
    }
     private void setVelocityTest() {
        if (masqMotor.setVelocity(0.5, 5, 100) != 1) throw new AssertionError("setVelocityTest Failed");
    }
     private void getStalledTest() {
        masqMotor.setStalledRPMThreshold(1);
        if (!masqMotor.getStalled(10, 1e9, 1200)) throw new AssertionError("getStalledTest failed");
    }
    private void enableStallDetectionTest(){
        masqMotor.setStalledRPMThreshold(1);
        masqMotor.setUnStalledAction(() -> {throw new AssertionError("enableStallDetectionTest failed"); });
        masqMotor.enableStallDetection(10,1e9,1200);
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
        getStalledTest();
        enableStallDetectionTest();
    }

    @Override
    public void RunAll(HardwareMap hardwareMap) {

    }
}
