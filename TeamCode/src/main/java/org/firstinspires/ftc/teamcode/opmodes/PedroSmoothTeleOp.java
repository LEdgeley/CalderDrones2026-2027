package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.controllers.PIDController;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.SmoothMotionProfiler;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Pedro Field Centric Smooth Heading", group = "TeleOp")
public class PedroSmoothTeleOp extends LinearOpMode {

    private Follower follower;
    private SmoothMotionProfiler profiler;
    private Controller headingPIDController;
    private Controller feedForwardController;


    public static Pose startingPose = new Pose(72,72); //See ExampleAuto to understand how to use this
    public static double kP = 0.8;
    public static double kI = 0.0;
    public static double kD = 0.08;
    private static double kF = 0.11; // Our manual velocity feedforward modifier

    // Tuning parameters - Adjust these to fit your robot's physical capabilities
    private static double MAX_ANGULAR_VEL = 7.1;  // Max turn speed (radians/sec)
    private static double MAX_ANGULAR_ACCEL = 120; // Smooth ramping acceleration (radians/sec²)

    private double targetHeading = 0.0;
    private double lastLinearizedTarget = 0.0;
    private ElapsedTime loopTimer = new ElapsedTime();

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize PedroPathing Follower (which auto-initializes your Pinpoint)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.startTeleopDrive();

        // 2. Initialize our Motion Profiler and PIDF
        profiler = new SmoothMotionProfiler(MAX_ANGULAR_VEL, MAX_ANGULAR_ACCEL);
        // Start with a conservative Kp and a healthy Kf/Kd for the Pinpoint
        headingPIDController = Controller.pid(kP, kI, kD);

        feedForwardController = Controller.proportionalFeedforward(kF);

        teleOpInit();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            follower.update();

            double dt = loopTimer.seconds();
            loopTimer.reset();

            double currentHeading = follower.pose().heading();

            double driveX = -gamepad1.left_stick_y;
            double driveY = -gamepad1.left_stick_x;
            double turnInput = -gamepad1.right_stick_x;

            // 1. Heading Lock Logic
            if (Math.abs(turnInput) > 0.05) {
                targetHeading += turnInput * MAX_ANGULAR_VEL * dt;
            }

            // 2. Wrap the angles smoothly
            double linearizedTarget = getTargetLinearized(currentHeading, targetHeading);

            // 3. Update the profile targets if the driver altered the course
            if (Math.abs(linearizedTarget - lastLinearizedTarget) > 0.001) {
                SmoothMotionProfiler.MotionState currentProfileState = profiler.update();

                double deltaTarget = linearizedTarget - lastLinearizedTarget;

                if (deltaTarget < -Math.PI) {
                    currentProfileState.position -= 2 * Math.PI;
                } else if (deltaTarget > Math.PI) {
                    currentProfileState.position += 2 * Math.PI;
                }

                profiler.startNewProfile(linearizedTarget, currentProfileState);
                lastLinearizedTarget = linearizedTarget;
            }

            // 4. Extract the smooth trajectory targets for this frame
            SmoothMotionProfiler.MotionState profileGoal = profiler.update();

            // 5. Calculate Error for PedroPathing
            // Error is where the profile says we should be right now minus where we actually are
            double positionError = profileGoal.position - currentHeading;

            // 6. Run Pedro's PID calculation using the error
            headingPIDController.calculate(profileGoal.position, positionError);
            feedForwardController.calculate(profileGoal.position, positionError);

            // 7. Combine Pedro's PID with our Profile's Velocity Feedforward

            double turnPower = pidOutput + feedforward;

            // 8. Command the drive vectors
            follower.setTeleOpDrive(driveX, driveY, turnPower, false);


            // 1. Graph Positions (Degrees)
            telemetryM.addData("Target Heading", Math.toDegrees(profileGoal.position));
            telemetryM.addData("Actual Heading", Math.toDegrees(currentHeading));

            // 2. Graph Velocities (Degrees per Second)
            telemetryM.addData("Target Velocity", Math.toDegrees(profileGoal.velocity));

            // In Pedro Pathing, follower.getVelocity() returns a Pose where getHeading() represents angular velocity
            double actualAngularVel = Math.toDegrees(follower.getAngularVelocity());
            telemetryM.addData("Actual Velocity", actualAngularVel);

            // 3. Optional: Graph your PID output to see how hard it's fighting
            telemetryM.addData("PID Control Output", pidOutput);

            // --- UPDATE BOTH THE DRIVER STATION AND THE PANELS DASHBOARD ---
            // Passing 'telemetry' here ensures it mirrors to your driver station screen too
            telemetryM.update(telemetry);

            // Optional: Provide telemetry data to watch the tracking performance
            //telemetry.addData("Linearized Target Angle", Math.toDegrees(linearizedTarget));
            //telemetry.addData("Actual Angle", Math.toDegrees(currentHeading));
            //telemetry.addData("Profile Velocity Setpoint", profileGoal.velocity);
            //telemetry.addData("Current Angular Velocity (rads/s)", follower.getAngularVelocity());
            telemetry.update();
        }
    }

    /**
     * Realigns the target angle to prevent the robot from spinning the long way around
     */
    private double getTargetLinearized(double current, double target) {
        double delta = target - current;
        while (delta > Math.PI) delta -= 2 * Math.PI;
        while (delta <= -Math.PI) delta += 2 * Math.PI;
        return current + delta;
    }

    private void teleOpInit() {
        telemetry.addData("Status", "Configured for PedroPathing Field Centric Turn Lock.");
        telemetry.update();
    }
}