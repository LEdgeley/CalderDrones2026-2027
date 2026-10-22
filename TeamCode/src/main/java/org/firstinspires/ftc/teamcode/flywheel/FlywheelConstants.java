package org.firstinspires.ftc.teamcode.flywheel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Flywheel settings shared by FlyMeToTheWheel and FlywheelTuner.
 * Values changed in Panels last until the robot restarts; copy tuned values here to keep them.
 */
@Configurable
public class FlywheelConstants {
    public static String motorName = "Launcher";
    public static DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    /** Encoder counts per motor revolution (the HD Hex motor's built-in encoder). */
    public static double countsPerRev = 28.0;

    // Feedback gains (power per RPM of error)
    public static double Kp = 0.005;
    public static double Ki = 0.000002;
    public static double Kd = 0.000004;

    // Feedforward gains: kV is power per RPM of target, kS is the constant power to overcome friction
    public static double kV = 0.0;
    public static double kS = 0.0;

    /** Largest change in motor power allowed per second (2.0 = zero to full power in 0.5 s). */
    public static double maxPowerRate = 2.0;
    /** Low-pass filter time constant for the measured RPM in seconds (0 = no filtering). */
    public static double rpmFilterTimeConstant = 0.03;
}
