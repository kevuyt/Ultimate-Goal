package Library4997.MasqControlSystems.MasqPID;

import Library4997.MasqResources.MasqUtils;

/**
 * Created by Archishmaan Peyyety on 12/14/18.
 * Project: MasqLib
 */
public class MasqPIDPackage {
    private double kpTurn, kiTurn, kdTurn,
                   kpDriveAngular, kiDriveAngular, kdDriveAngular,
                   kpDriveEncoder, kiDriveEncoder, kdDriveEncoder,
                   kpMotorTeleOpLeft, kiMotorTeleOpLeft, kdMotorTeleOpLeft,
                   kpMotorAutoLeft, kiMotorAutoLeft, kdMotorAutoLeft,
                   kpMotorTeleOpRight, kiMotorTeleOpRight, kdMotorTeleOpRight,
                   kpMotorAutoRight, kiMotorAutoRight, kdMotorAutoRight;


    public MasqPIDPackage(double kpTurn, double kiTurn, double kdTurn,
                          double kpDriveAngular, double kiDriveAngular, double kdDriveAngular,
                          double kpDriveEncoder, double kiDriveEncoder, double kdDriveEncoder,
                          double kpMotorTeleOpLeft, double kiMotorTeleOpLeft, double kdMotorTeleOpLeft,
                          double kpMotorAutoLeft, double kiMotorAutoLeft, double kdMotorAutoLeft,
                          double kpMotorTeleOpRight, double kiMotorTeleOpRight, double kdMotorTeleOpRight,
                          double kpMotorAutoRight, double kiMotorAutoRight, double kdMotorAutoRight) {
        this.kpTurn = kpTurn;
        this.kiTurn = kiTurn;
        this.kdTurn = kdTurn;
        this.kpDriveAngular = kpDriveAngular;
        this.kiDriveAngular = kiDriveAngular;
        this.kdDriveAngular = kdDriveAngular;
        this.kpDriveEncoder = kpDriveEncoder;
        this.kiDriveEncoder = kiDriveEncoder;
        this.kdDriveEncoder = kdDriveEncoder;
        this.kpMotorTeleOpLeft = kpMotorTeleOpLeft;
        this.kiMotorTeleOpLeft = kiMotorTeleOpLeft;
        this.kdMotorTeleOpLeft = kdMotorTeleOpLeft;
        this.kpMotorAutoLeft = kpMotorAutoLeft;
        this.kiMotorAutoLeft = kiMotorAutoLeft;
        this.kdMotorAutoLeft = kdMotorAutoLeft;
        this.kpMotorTeleOpRight = kpMotorTeleOpRight;
        this.kiMotorTeleOpRight = kiMotorTeleOpRight;
        this.kdMotorTeleOpRight = kdMotorTeleOpRight;
        this.kpMotorAutoRight = kpMotorAutoRight;
        this.kiMotorAutoRight = kiMotorAutoRight;
        this.kdMotorAutoRight = kdMotorAutoRight;
    }

    public MasqPIDPackage () {
        this.kpTurn = MasqUtils.KP.TURN;
        this.kiTurn = MasqUtils.KI.TURN;
        this.kdTurn = MasqUtils.KD.TURN;
        this.kpDriveAngular = MasqUtils.KP.DRIVE_ANGULAR;
        this.kiDriveAngular = MasqUtils.KI.DRIVE;
        this.kdDriveAngular = MasqUtils.KD.DRIVE;
        this.kpDriveEncoder = MasqUtils.KP.DRIVE_ENCODER;
        this.kiDriveEncoder = 0;
        this.kdDriveEncoder = 0;
        this.kpMotorTeleOpLeft = MasqUtils.KP.MOTOR_TELEOP;
        this.kiMotorTeleOpLeft = MasqUtils.KP.MOTOR_TELEOP;
        this.kdMotorTeleOpLeft = MasqUtils.KP.MOTOR_TELEOP;
        this.kpMotorAutoLeft = MasqUtils.KP.MOTOR_AUTONOMOUS;
        this.kiMotorAutoLeft = MasqUtils.KP.MOTOR_AUTONOMOUS;
        this.kdMotorAutoLeft = MasqUtils.KP.MOTOR_AUTONOMOUS;
        this.kpMotorTeleOpRight = MasqUtils.KP.MOTOR_TELEOP;
        this.kiMotorTeleOpRight = MasqUtils.KP.MOTOR_TELEOP;
        this.kdMotorTeleOpRight = MasqUtils.KP.MOTOR_TELEOP;
        this.kpMotorAutoRight = MasqUtils.KP.MOTOR_AUTONOMOUS;
        this.kiMotorAutoRight = MasqUtils.KP.MOTOR_AUTONOMOUS;
        this.kdMotorAutoRight = MasqUtils.KP.MOTOR_AUTONOMOUS;
    }

    public double getKpTurn() {
        return kpTurn;
    }

    public void setKpTurn(double kpTurn) {
        this.kpTurn = kpTurn;
    }

    public double getKiTurn() {
        return kiTurn;
    }

    public void setKiTurn(double kiTurn) {
        this.kiTurn = kiTurn;
    }

    public double getKdTurn() {
        return kdTurn;
    }

    public void setKdTurn(double kdTurn) {
        this.kdTurn = kdTurn;
    }

    public double getKpDriveAngular() {
        return kpDriveAngular;
    }

    public void setKpDriveAngular(double kpDriveAngular) {
        this.kpDriveAngular = kpDriveAngular;
    }

    public double getKiDriveAngular() {
        return kiDriveAngular;
    }

    public void setKiDriveAngular(double kiDriveAngular) {
        this.kiDriveAngular = kiDriveAngular;
    }

    public double getKdDriveAngular() {
        return kdDriveAngular;
    }

    public void setKdDriveAngular(double kdDriveAngular) {
        this.kdDriveAngular = kdDriveAngular;
    }

    public double getKpDriveEncoder() {
        return kpDriveEncoder;
    }

    public void setKpDriveEncoder(double kpDriveEncoder) {
        this.kpDriveEncoder = kpDriveEncoder;
    }

    public double getKiDriveEncoder() {
        return kiDriveEncoder;
    }

    public void setKiDriveEncoder(double kiDriveEncoder) {
        this.kiDriveEncoder = kiDriveEncoder;
    }

    public double getKdDriveEncoder() {
        return kdDriveEncoder;
    }

    public void setKdDriveEncoder(double kdDriveEncoder) {
        this.kdDriveEncoder = kdDriveEncoder;
    }

    public double getKpMotorTeleOpLeft() {
        return kpMotorTeleOpLeft;
    }

    public void setKpMotorTeleOpLeft(double kpMotorTeleOpLeft) {
        this.kpMotorTeleOpLeft = kpMotorTeleOpLeft;
    }

    public double getKiMotorTeleOpLeft() {
        return kiMotorTeleOpLeft;
    }

    public void setKiMotorTeleOpLeft(double kiMotorTeleOpLeft) {
        this.kiMotorTeleOpLeft = kiMotorTeleOpLeft;
    }

    public double getKdMotorTeleOpLeft() {
        return kdMotorTeleOpLeft;
    }

    public void setKdMotorTeleOpLeft(double kdMotorTeleOpLeft) {
        this.kdMotorTeleOpLeft = kdMotorTeleOpLeft;
    }

    public double getKpMotorAutoLeft() {
        return kpMotorAutoLeft;
    }

    public void setKpMotorAutoLeft(double kpMotorAutoLeft) {
        this.kpMotorAutoLeft = kpMotorAutoLeft;
    }

    public double getKiMotorAutoLeft() {
        return kiMotorAutoLeft;
    }

    public void setKiMotorAutoLeft(double kiMotorAutoLeft) {
        this.kiMotorAutoLeft = kiMotorAutoLeft;
    }

    public double getKdMotorAutoLeft() {
        return kdMotorAutoLeft;
    }

    public void setKdMotorAutoLeft(double kdMotorAutoLeft) {
        this.kdMotorAutoLeft = kdMotorAutoLeft;
    }

    public double getKpMotorTeleOpRight() {
        return kpMotorTeleOpRight;
    }

    public void setKpMotorTeleOpRight(double kpMotorTeleOpRight) {
        this.kpMotorTeleOpRight = kpMotorTeleOpRight;
    }

    public double getKiMotorTeleOpRight() {
        return kiMotorTeleOpRight;
    }

    public void setKiMotorTeleOpRight(double kiMotorTeleOpRight) {
        this.kiMotorTeleOpRight = kiMotorTeleOpRight;
    }

    public double getKdMotorTeleOpRight() {
        return kdMotorTeleOpRight;
    }

    public void setKdMotorTeleOpRight(double kdMotorTeleOpRight) {
        this.kdMotorTeleOpRight = kdMotorTeleOpRight;
    }

    public double getKpMotorAutoRight() {
        return kpMotorAutoRight;
    }

    public void setKpMotorAutoRight(double kpMotorAutoRight) {
        this.kpMotorAutoRight = kpMotorAutoRight;
    }

    public double getKiMotorAutoRight() {
        return kiMotorAutoRight;
    }

    public void setKiMotorAutoRight(double kiMotorAutoRight) {
        this.kiMotorAutoRight = kiMotorAutoRight;
    }

    public double getKdMotorAutoRight() {
        return kdMotorAutoRight;
    }

    public void setKdMotorAutoRight(double kdMotorAutoRight) {
        this.kdMotorAutoRight = kdMotorAutoRight;
    }
}
