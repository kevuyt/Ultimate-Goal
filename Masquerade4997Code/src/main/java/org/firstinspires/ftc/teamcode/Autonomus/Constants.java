package org.firstinspires.ftc.teamcode.Autonomus;

/**
 * Constants in autonomous
 */

public interface Constants {
    double POWER_LOW = 0.3,
           POWER_OPTIMAL = 0.5,
           POWER_HIGH = 0.7;
    double SLEEPTIME_HIGH = 3000,
           SLEEPTIME_OPTIMAL = 1000,
           SLEEPTIME_LOW = 500;
    double INDEXER_CLOSED = 0;
    double INDEXER_OPENED = 0.6;

    double TARGET_POWER = -0.8;
    double REV_UP = 0.1;
    double REV_DOWN = 0.01;
    double LOW_POWER_FACTOR = 0.2;
    double SHOOTER_ERROR = 0.05;

    double BEACON_OUT = -1;
    double BEACON_IN = 1;

    double COLLECTOR_IN = -1.5;
    double COLLECTOR_OUT = 1.5;
    int cornerTurn = 43;
}
