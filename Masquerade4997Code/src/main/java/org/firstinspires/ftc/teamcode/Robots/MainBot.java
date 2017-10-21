package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Library4997.MasqExternal.MasqAnnotations.MasqRobotClass;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqTankDrive;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqMRColorSensor;
import Library4997.MasqSensors.MasqVoltageSensor;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqServos.MasqServoSystem;

/**
 * Created by Archish on 10/21/17.
 */
@MasqRobotClass(id = "MainBot")
public class MainBot extends MasqRobot {
    public MasqTankDrive driveTrain;
    public MasqMotor lift;
    public MasqAdafruitIMU imu;
    public MasqServo jewelArm;
    public MasqServoSystem glyphSystem;
    public MasqMRColorSensor jewelColor;
    private HardwareMap hardwareMap;
    public MainBot(HardwareMap hardwareMap){
        super();
        this.hardwareMap = hardwareMap;
    }
    @Override
    public void mapHardware() {
        lift = new MasqMotor("lift", DcMotor.Direction.REVERSE, this.hardwareMap);
        driveTrain = new MasqTankDrive("leftFront", "leftBack", "rightFront", "rightBack", this.hardwareMap);
        glyphSystem = new MasqServoSystem("letGlyph", Servo.Direction.FORWARD, "rightGlyph", Servo.Direction.REVERSE, this.hardwareMap);
        imu = new MasqAdafruitIMU("imu", this.hardwareMap);
        voltageSensor = new MasqVoltageSensor(this.hardwareMap);
        jewelArm = new MasqServo("jewelArm", this.hardwareMap);
        jewelColor = new MasqMRColorSensor("jewelColor", this.hardwareMap);
    }
}
