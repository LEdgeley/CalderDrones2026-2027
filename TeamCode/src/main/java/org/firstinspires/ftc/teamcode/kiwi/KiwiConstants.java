package org.firstinspires.ftc.teamcode.kiwi;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class KiwiConstants {
    /**
     * The Forward Velocity of the Robot - Different for each robot
     * Default Value: 81.34056
     */
    public double xVelocity = 81.34056;

    /**
     * The Lateral Velocity of the Robot - Different for each robot
     * Default Value: 65.43028
     */
    public double yVelocity = 65.43028;

    private double[] convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);

    /**
     * The actual drive vector for the front left wheel, if the robot is facing a heading of 0 radians with the wheel centered at (0,0)
     * Default Value: new Vector(convertToPolar[0], convertToPolar[1])
     *
     * @implNote This vector should not be changed, but only accessed.
     */
    public Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
    public double maxPower = 1;
    public String motorAName = "motorA";
    public String motorBName = "motorB";
    public String motorCName = "motorC";

    public DcMotorSimple.Direction motorADirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction motorBDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction motorCDirection = DcMotorSimple.Direction.FORWARD;
    public double motorCachingThreshold = 0.01;
    public boolean useBrakeModeInTeleOp = false;
    public boolean useVoltageCompensation = false;
    public double nominalVoltage = 12.0;
    public double staticFrictionCoefficient = 0.1;

    public KiwiConstants() {
        defaults();
    }

    public KiwiConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public KiwiConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public KiwiConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public KiwiConstants motorAName(String motorAName) {
        this.motorAName = motorAName;
        return this;
    }

    public KiwiConstants motorBName(String motorBName) {
        this.motorBName = motorBName;
        return this;
    }

    public KiwiConstants motorCName(String motorCName) {
        this.motorCName = motorCName;
        return this;
    }


    public KiwiConstants motorADirection(DcMotorSimple.Direction motorADirection) {
        this.motorADirection = motorADirection;
        return this;
    }

    public KiwiConstants motorBDirection(DcMotorSimple.Direction motorBDirection) {
        this.motorBDirection = motorBDirection;
        return this;
    }

    public KiwiConstants motorCDirection(DcMotorSimple.Direction motorCDirection) {
        this.motorCDirection = motorCDirection;
        return this;
    }

    public KiwiConstants motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public KiwiConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    public KiwiConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public KiwiConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public KiwiConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public Vector getFrontLeftVector() {
        return frontLeftVector;
    }

    public void setFrontLeftVector(Vector frontLeftVector) {
        this.frontLeftVector = frontLeftVector;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public String getMotorAName() {
        return motorAName;
    }

    public void setMotorAName(String motorAName) {
        this.motorAName = motorAName;
    }

    public String getMotorBName() {
        return motorBName;
    }

    public void setMotorBName(String motorBName) {
        this.motorBName = motorBName;
    }

    public String getMotorCName() {
        return motorCName;
    }

    public void setMotorCName(String motorCName) {
        this.motorCName = motorCName;
    }

    public DcMotorSimple.Direction getMotorADirection() {
        return motorADirection;
    }

    public void setMotorADirection(DcMotorSimple.Direction motorADirection) {
        this.motorADirection = motorADirection;
    }

    public DcMotorSimple.Direction getMotorBDirection() {
        return motorBDirection;
    }

    public void setMotorBDirection(DcMotorSimple.Direction motorBDirection) {
        this.motorBDirection = motorBDirection;
    }


    public DcMotorSimple.Direction getMotorCDirection() {
        return motorCDirection;
    }

    public void setMotorCDirection(DcMotorSimple.Direction motorCDirection) {
        this.motorCDirection = motorCDirection;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public boolean isUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    public void setUseBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
    }

    public boolean getUseBrakeModeInTeleOp() {
        return this.useBrakeModeInTeleOp;
    }


    /**
     * This method sets the default values for the KiwiConstants class.
     * It is called in the constructor of the KiwiConstants class.
     */
    public void defaults() {
        xVelocity = 81.34056;
        yVelocity = 65.43028;
        convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
        frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
        maxPower = 1;
        motorAName = "motorA";
        motorBName = "motorB";
        motorCName = "motorC";
        motorADirection = DcMotorSimple.Direction.FORWARD;
        motorBDirection = DcMotorSimple.Direction.FORWARD;
        motorCDirection = DcMotorSimple.Direction.FORWARD;
        motorCachingThreshold = 0.01;
        useBrakeModeInTeleOp = false;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
    }

    public boolean getUseVoltageCompensation() {
        return this.useVoltageCompensation;
    }

    public double getNominalVoltage() {
        return this.nominalVoltage;
    }

    public double getStaticFrictionCoefficient() {
        return  this.staticFrictionCoefficient;
    }

}


