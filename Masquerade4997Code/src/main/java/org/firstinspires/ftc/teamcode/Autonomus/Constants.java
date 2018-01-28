package org.firstinspires.ftc.teamcode.Autonomus;

/**
 * Constants in autonomous
 */

public interface Constants extends org.firstinspires.ftc.teamcode.TeleOp.Constants {
    double POWER_LOW = 0.3,
           POWER_OPTIMAL = 0.6,
           POWER_HIGH = .7;
    double SLEEPTIME_HIGH = 1000,
           SLEEPTIME_OPTIMAL = 500,
           SLEEPTIME_LOW = 250;
    double ROTATOR_RED_NOT_SEEN = .82,
           ROTATOR_RED_SEEN = 0.52,
           ROTATOR_RED_CENTER = .73,
           ROTATOR_BLUE_CENTER = .56;
    double ROTATOR_BLUE_NOT_SEEN = .73,
           ROTATOR_BLUE_SEEN = .33,
           ROTATOR_BLUE_INIT = 0.38;
    double CRYPTOBOX_SEPERATION_DISTANCE = 7.63; //this is in inches.
    double DISTANCE_TO_LEFT_BOX = 36 - (CRYPTOBOX_SEPERATION_DISTANCE / 2),
           DISTANCE_TO_RIGHT_BOX = 36 + (CRYPTOBOX_SEPERATION_DISTANCE / 2) + CRYPTOBOX_SEPERATION_DISTANCE,
           DISTANCE_TO_CENTER_BOX = 36 + (CRYPTOBOX_SEPERATION_DISTANCE / 2);

    String LEFT = "LEFT",
           RIGHT = "RIGHT",
           CENTER = "CENTER";
}
