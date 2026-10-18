package org.firstinspires.ftc.teamcode.flywheel;

import com.pedropathing.controllers.Controller;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.flywheel.FlywheelConstants.*;

/**
 * Flywheel velocity control using Pedro Pathing 3 controllers: PID feedback on the filtered RPM error
 * plus feedforward on the target RPM, with a per-second limit on how fast the power can change.
 * Call one of the run methods once per loop.
 */
public class Flywheel {
    /** Longest loop time used for the slew limit, so a stalled loop can't allow a large jump. */
    private static final double MAX_LOOP_SECONDS = 0.05;

    private final DcMotorEx motor;
    private final Controller controller;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double rawRPM = 0;
    private double filteredRPM = 0;
    private double rpmRate = 0;
    private double power = 0;
    private double targetRPM = 0;

    public Flywheel(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Suppliers let the gains be changed from Panels while running
        controller = Controller.sum(
                Controller.proportional(() -> Kp),
                Controller.integral(() -> Ki),
                Controller.derivative(() -> Kd),
                Controller.proportionalFeedforward(() -> kV),
                Controller.staticTargetFeedforward(() -> kS)
        );
    }

    /** Call before starting closed-loop control, e.g. after waitForStart or when switching modes. */
    public void reset() {
        loopTimer.reset();
        controller.reset();
        rawRPM = readRPM();
        filteredRPM = rawRPM;
        rpmRate = 0;
    }

    /** Drives the flywheel towards targetRPM using the controllers. */
    public void runToRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        double dt = measure();
        double error = targetRPM - filteredRPM;
        // Passing the filtered RPM's rate of change makes the D term act on the measurement
        applyPower(controller.calculate(targetRPM, error, rpmRate), dt);
    }

    /** Sends a fixed power (still rate-limited) with no feedback, for feedforward testing. */
    public void runOpenLoop(double requestedPower) {
        targetRPM = 0;
        applyPower(requestedPower, measure());
    }

    /** Cuts power; with FLOAT the wheel coasts to a stop. */
    public void stop() {
        targetRPM = 0;
        power = 0;
        motor.setPower(0);
        measure();
    }

    public double rawRPM() {
        return rawRPM;
    }

    public double filteredRPM() {
        return filteredRPM;
    }

    public double power() {
        return power;
    }

    public double targetRPM() {
        return targetRPM;
    }

    /** Reads the RPM and updates the low-pass filter. Returns this loop's duration in seconds. */
    private double measure() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        rawRPM = readRPM();
        double alpha = rpmFilterTimeConstant > 0 ? dt / (rpmFilterTimeConstant + dt) : 1.0;
        double lastFilteredRPM = filteredRPM;
        filteredRPM += alpha * (rawRPM - filteredRPM);
        if (dt > 1e-3) {
            rpmRate = (filteredRPM - lastFilteredRPM) / dt;
        }
        return dt;
    }

    private void applyPower(double requested, double dt) {
        double out = Range.clip(requested, 0.0, 1.0);
        // Limit how quickly the power can change, scaled by this loop's duration
        double maxStep = maxPowerRate * Math.min(dt, MAX_LOOP_SECONDS);
        out = Range.clip(out, power - maxStep, power + maxStep);
        motor.setPower(out);
        power = out;
    }

    private double readRPM() {
        return motor.getVelocity() / countsPerRev * 60.0;
    }
}
