package org.firstinspires.ftc.teamcode.ConstantsAndStuff;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rpmToVelocity;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Write out Documentation Code
 */

public class Constants {

    // Servo constants
    public static double lOpen = .8;
    public static double rOpen = .2;

    public static double rClose = 0;
    public static double lClose = .96;

    public static double lArmIn = 0;
    public static double rArmIn = 1;

    public static double lArmHold = .334444444444;
    public static double rArmHold = .6627777777777;

    public static double lArmOut = .58;
    public static double rArmOut = .4161111111111;

    // dr4b constants
    public static int rLinkDown = 0;
    public static int lLinkDown = 0;

    public static int rLinkLow = 0;
    public static int lLinkLow = 0;

    public static int rLinkMedium = 0;
    public static int lLinkMedium = 0;

    public static int rLinkHigh = 0;
    public static int lLinkHigh = 0;

    public boolean deployed = false;
    public boolean intake = true;


    // Dt constants
    // 19.2:1 gear ratio
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;

    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    // Wheel
    public static double WHEEL_R_MM = 48;
    public static final double MM_IN = 25.4;

    public static double WHEEL_RADIUS = WHEEL_R_MM / MM_IN;
    public static double GEAR_RATIO = 1;
    public static double TRACK_WIDTH = 13.57;

    public static double kV = 12.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    //public static double MAX_VEL = (MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * (Math.PI * 2);
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);



}
