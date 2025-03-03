package org.firstinspires.ftc.teamcode.ConstantsAndStuff;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rpmToVelocity;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Write out Documentation Code
 */

public class Constants {

    // Servo constants
    public static double lOpen = .86;
    public static double rOpen = .1;

    public static double rClose = 0.005;
    public static double lClose = 0.955;

    public static double lArmIn = .07;
    public static double rArmIn = 1;

    public static double lArmHold = .37;
    public static double rArmHold = .7077777777777777777;

    public static double lArmOut = .495;
    public static double rArmOut = .580555;

    public static double lArmIntakePrep = .1866666666666666666;
    public static double rArmIntakePrep = .8772222222222222;

    public static double lArmIntakeLimit = .217222222;
    public static double rArmintakeLimit = .845555555;

    // dr4b constants
    public static int LINKAGE_DOWN = -100;

    public static int LINKAGE_LOW = -200;

    public static int LINKAGE_MEDIUM = -360;

    public static int LINKAGE_HIGH = -555;



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
