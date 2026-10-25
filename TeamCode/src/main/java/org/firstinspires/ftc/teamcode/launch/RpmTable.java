package org.firstinspires.ftc.teamcode.launch;

import org.firstinspires.ftc.teamcode.vision.HiveTarget;

/**
 * Flywheel RPM against distance for one launcher, worked out with LaunchModel when it's created (at INIT),
 * so the OpMode loop only does a lookup. Distances are horizontal, from the launcher's axle to
 * LimelightCamera's target point.
 */
public class RpmTable {
    public static final double MIN_DISTANCE = 0.30;
    public static final double MAX_DISTANCE = 2.30;
    public static final double STEP = 0.05;

    private final LaunchModel.Launcher launcher;
    private final double[] rpm;

    public RpmTable(LaunchModel.Launcher launcher) {
        this.launcher = launcher;
        int n = (int) Math.round((MAX_DISTANCE - MIN_DISTANCE) / STEP) + 1;
        rpm = new double[n];
        for (int i = 0; i < n; i++) {
            rpm[i] = LaunchModel.solveRPM(launcher, MIN_DISTANCE + i * STEP, LaunchModel.FACE_HEIGHT);
        }
        // Distances with no scoring RPM (e.g. too close) take the nearest distance the model solved
        double[] solved = rpm.clone();
        for (int i = 0; i < n; i++) {
            if (!Double.isNaN(solved[i])) continue;
            for (int d = 1; d < n; d++) {
                if (i - d >= 0 && !Double.isNaN(solved[i - d])) { rpm[i] = solved[i - d]; break; }
                if (i + d < n && !Double.isNaN(solved[i + d])) { rpm[i] = solved[i + d]; break; }
            }
        }
    }

    /** RPM to hit a target the camera sees, from this launcher's axle. */
    public double rpmFor(HiveTarget target) {
        return rpmAt(target.horizontalDistanceFrom(launcher.axleForward, launcher.axleLeft));
    }

    /** RPM for an axle-to-target distance, interpolated, held at the table's ends. NaN if nothing scores anywhere. */
    public double rpmAt(double distance) {
        double x = (distance - MIN_DISTANCE) / STEP;
        if (x <= 0) return rpm[0];
        if (x >= rpm.length - 1) return rpm[rpm.length - 1];
        int i = (int) Math.floor(x);
        double f = x - i;
        return rpm[i] * (1 - f) + rpm[i + 1] * f;
    }

    public double distanceAt(int i) {
        return MIN_DISTANCE + i * STEP;
    }

    public double rpmAtIndex(int i) {
        return rpm[i];
    }

    public int size() {
        return rpm.length;
    }
}
