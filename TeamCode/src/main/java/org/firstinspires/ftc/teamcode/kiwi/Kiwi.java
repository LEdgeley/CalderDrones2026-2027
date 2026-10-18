package org.firstinspires.ftc.teamcode.kiwi;

import com.pedropathing.drivetrain.CustomDrivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


import java.util.Arrays;
import java.util.List;

/**
 * Kiwi Drivetrain implementation.
 * Angles are in radians and positive rotation is to the left (CCW, top-down).
 *
 * @author Kabir Goyal
 * @author Baron Henderson
 */
public class Kiwi extends CustomDrivetrain {
    private final KiwiConstants constants;

    private final DcMotorEx motorA;
    private final DcMotorEx motorB;
    private final DcMotorEx motorC;
    private final List<DcMotorEx> motors;
    private final double[] lastMotorPowers;
    private final VoltageSensor voltageSensor;
    private double motorCachingThreshold;

    protected double lastHeading = 0;

    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;
    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastRotation = 0;
    private double lastAvgScaling = 0;

    public Kiwi(HardwareMap hardwareMap, KiwiConstants constants) {
        this.constants = constants;

        this.maxPowerScaling = constants.maxPower;
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        motorA = hardwareMap.get(DcMotorEx.class, constants.motorAName);
        motorB = hardwareMap.get(DcMotorEx.class, constants.motorBName);
        motorC = hardwareMap.get(DcMotorEx.class, constants.motorCName);

        motors = Arrays.asList(motorA, motorB, motorC);
        lastMotorPowers = new double[] {0,0,0};

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
        breakFollowing();

        updateConstants();
    }

    /**
     * This method takes in forward, strafe, and rotation values and applies them to
     * the 3-wheel Kiwi drivetrain.
     *
     * @param forward  the forward power value (+X is forward)
     * @param strafe   the strafe power value (+Y is left, matching Pedro's convention)
     * @param rotation the rotation power value (Positive is CCW)
     */
    @Override
    public void arcadeDrive(double forward, double strafe, double rotation) {
        lastForward = forward;
        lastStrafe = strafe;
        lastRotation = rotation;

        // 3-Wheel Kiwi Kinematics (Spaced 120 degrees apart)
        // Assumes Motor A's driving force is parallel to the Y-axis (Left/Right)
        // Motor B is at 120 degrees (front-left), Motor C is at 240 degrees (front-right)
        double powerA = -strafe + rotation;
        double powerB = (-Math.sqrt(3) / 2.0 * forward) + (0.5 * strafe) + rotation;
        double powerC = (Math.sqrt(3) / 2.0 * forward) + (0.5 * strafe) + rotation;

        // Apply voltage compensation if enabled
        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            powerA *= voltageNormalized;
            powerB *= voltageNormalized;
            powerC *= voltageNormalized;
        }

        // Normalize motor powers if any magnitude exceeds maxPowerScaling
        // This preserves the structural integrity of your translation/rotation vector
        double max = Math.max(Math.abs(powerA), Math.max(Math.abs(powerB), Math.abs(powerC)));
        if (max > maxPowerScaling) {
            powerA = (powerA / max) * maxPowerScaling;
            powerB = (powerB / max) * maxPowerScaling;
            powerC = (powerC / max) * maxPowerScaling;
        }

        // Output to the hardware
        motorA.setPower(powerA);
        motorB.setPower(powerB);
        motorC.setPower(powerC);

        // Cache the powers for tracking/debugging
        lastMotorPowers[0] = powerA;
        lastMotorPowers[1] = powerB;
        lastMotorPowers[2] = powerC;
    }

    /**
     * Updates cached values from the constants object.
     */
    @Override
    public void updateConstants() {
        motorA.setDirection(constants.motorADirection);
        motorB.setDirection(constants.motorBDirection);
        motorC.setDirection(constants.motorCDirection);
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.voltageCompensation = constants.useVoltageCompensation;
        this.nominalVoltage = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
        this.useBrakeModeInTeleOp = constants.getUseBrakeModeInTeleOp();
        this.maxPowerScaling = constants.getMaxPower(); // inherited from Drivetrain, used by CustomDrivetrain
        this.voltageCompensation = constants.getUseVoltageCompensation(); // inherited from Drivetrain
        this.nominalVoltage = constants.getNominalVoltage(); // inherited from Drivetrain
        this.staticFrictionCoefficient = constants.getStaticFrictionCoefficient();

    }

    /**
     * Stops following and holds pod angles while floating drive motors.
     */
    @Override
    public void breakFollowing() {
        for (int i = 0; i < motors.size(); i++) {
            lastMotorPowers[i] = 0;
            motors.get(i).setPower(0);
        }
        setMotorsToFloat();
    }

    /**
     * Starts teleop drive with the configured brake mode.
     */
    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    /**
     * @param brakeMode set to true to enable brake mode in teleop
     */
    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            setMotorsToBrake();
        } else {
            setMotorsToFloat();
        }
    }

    /**
     * @return maximum x velocity
     */
    @Override
    public double xVelocity() {
        return constants.getXVelocity();
    }

    /**
     * @return maximum y velocity
     */
    @Override
    public double yVelocity() {
        return constants.getYVelocity();
    }

    /**
     * @param xMovement maximum x velocity
     */
    @Override
    public void setXVelocity(double xMovement) {
        constants.setXVelocity(xMovement);
    }

    /**
     * @param yMovement maximum y velocity
     */
    @Override
    public void setYVelocity(double yMovement) {
        constants.setYVelocity(yMovement);
    }

    /**
     * @return static friction coefficient used for voltage compensation
     */
    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    /**
     * @return current battery voltage
     */
    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    /**
     * @return normalized voltage for voltage compensation
     */
    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage
                - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    /**
     * @return debug string for drivetrain state
     */
    @Override
    public String debugString() {
        return "Kiwi{" +
                " motorA=" + motorA +
                ", motorB=" + motorB +
                ", motorC=" + motorC +
                ", motors=" + motors +
                ", motorCachingThreshold=" + motorCachingThreshold +
                ", useBrakeModeInTeleOp=" + useBrakeModeInTeleOp +
                '}';
    }

    /**
     * This sets the motors to the zero power behavior of brake.
     */
    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * This sets the motors to the zero power behavior of float.
     */
    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}
