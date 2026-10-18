package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SmoothMotionProfiler {

    public static class MotionState {
        public double position;
        public double velocity;

        public MotionState(double pos, double vel) {
            this.position = pos;
            this.velocity = vel;
        }
    }

    private final double max_v;
    private final double max_a;

    // Profile timing and tracking
    private ElapsedTime timer = new ElapsedTime();
    private MotionState startState = new MotionState(0, 0);
    private double targetPosition = 0;

    private double t_accel, t_cruise, t_decel, t_total;
    private double v_peak;
    private double direction = 1.0;

    public SmoothMotionProfiler(double maxVelocity, double maxAcceleration) {
        this.max_v = maxVelocity;
        this.max_a = maxAcceleration;
    }

    /**
     * Call this whenever the target changes—even mid-move!
     */
    public synchronized void startNewProfile(double newTarget, MotionState currentState) {
        this.targetPosition = newTarget;
        this.startState = currentState; // Captures actual mid-move pos and vel!

        double distanceRemaining = newTarget - currentState.position;
        this.direction = Math.signum(distanceRemaining);
        double absDist = Math.abs(distanceRemaining);

        // Standard profiles assume v0 = 0. For a true v0 != 0 profile, the logic expands.
        // As a robust workaround for mid-flight adjustments, we calculate the time
        // to safely decelerate or accelerate from currentState.velocity.

        this.t_accel = (max_v - Math.abs(currentState.velocity)) / max_a;
        if (this.t_accel < 0) this.t_accel = 0; // Already going faster than max_v

        double d_accel = Math.abs(currentState.velocity) * t_accel + 0.5 * max_a * (t_accel * t_accel);
        double d_decel = (max_v * max_v) / (2 * max_a);

        if (d_accel + d_decel > absDist) {
            // Short trip: Triangular profile adjustment
            this.v_peak = Math.sqrt((2 * max_a * absDist + currentState.velocity * currentState.velocity) / 2.0);
            this.t_accel = Math.max(0, (v_peak - Math.abs(currentState.velocity)) / max_a);
            this.t_cruise = 0;
            this.t_decel = v_peak / max_a;
        } else {
            // Long trip: Traditional Trapezoid
            this.v_peak = max_v;
            this.t_cruise = (absDist - (d_accel + d_decel)) / max_v;
            this.t_decel = max_v / max_a;
        }

        this.t_total = t_accel + t_cruise + t_decel;
        timer.reset(); // Restart clock for the new profile
    }

    /**
     * Generates the smooth setpoint for your PIDF loop
     */
    public synchronized MotionState update() {
        double t = timer.seconds();

        if (t >= t_total) {
            return new MotionState(targetPosition, 0.0);
        }

        double p, v;
        double v0 = Math.abs(startState.velocity);

        if (t < t_accel) {
            // Phase 1: Adjusting acceleration from initial velocity
            v = v0 + (max_a * t);
            p = (v0 * t) + (0.5 * max_a * t * t);
        } else if (t < t_accel + t_cruise) {
            // Phase 2: Cruising
            double t_inside = t - t_accel;
            v = v_peak;
            double d_accel = (v0 * t_accel) + (0.5 * max_a * t_accel * t_accel);
            p = d_accel + (v_peak * t_inside);
        } else {
            // Phase 3: Decelerating to 0
            double t_inside = t - t_accel - t_cruise;
            v = v_peak - (max_a * t_inside);
            double d_accel = (v0 * t_accel) + (0.5 * max_a * t_accel * t_accel);
            double d_cruise = v_peak * t_cruise;
            p = d_accel + d_cruise + (v_peak * t_inside) - (0.5 * max_a * t_inside * t_inside);
        }

        // Apply direction sign relative to starting state
        return new MotionState(
                startState.position + (p * direction),
                v * direction
        );
    }
}