package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Driving test TeleOp using Pedro Pathing 3's manual drive and heading lock.
 *
 * Left stick translates, right stick turns. Releasing the right stick brakes the rotation
 * and then holds the heading the robot settled at. Circle toggles robot/field-centric.
 * Gains are live-tunable from Panels.
 */
@TeleOp(name = "Driving Test", group = "TeleOp")
@Configurable
public class DrivingTestTeleOp extends OpMode {

    public static Pose startingPose = new Pose(72, 72, 0);

    // Heading lock gains (turn power per radian of error, and per rad/s of angular velocity for kD)
    public static double kP = 0.8;
    public static double kI = 0.0;
    public static double kD = 0.08;
    public static double kS = 0.0; // Static feedforward to overcome turning friction

    // Driver input tuning
    public static double turnDeadband = 0.05;
    public static double turnScale = 1.0;
    /** Angular velocity (rad/s) below which the robot is considered settled and the heading is locked. */
    public static double settleRate = 0.3;

    private enum HeadingState { TURNING, SETTLING, LOCKED }

    private Follower follower;
    private TelemetryManager telemetryM;
    private Controller headingController;

    private HeadingState headingState = HeadingState.LOCKED;
    private double targetHeading = 0.0;
    private boolean fieldCentric = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Suppliers let the gains be changed from Panels while running
        headingController = Controller.sum(
                Controller.proportional(() -> kP),
                Controller.integral(() -> kI),
                Controller.derivative(() -> kD),
                Controller.staticFeedforward(() -> kS)
        );

        telemetry.addData("Status", "Initialized. Circle toggles robot/field-centric.");
        telemetry.update();
    }

    @Override
    public void start() {
        lockHeading();
    }

    @Override
    public void loop() {
        follower.update();

        if (gamepad1.circleWasPressed()) {
            fieldCentric = !fieldCentric;
        }

        double heading = follower.pose().heading();
        double angularVelocity = follower.twist().omega;

        // +forward is forward, +strafe is left and +turn is CCW, so the X axes are negated
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turnInput = -gamepad1.right_stick_x;

        DrivePowers powers = new DrivePowers(forward, strafe, turnInput * turnScale);

        if (fieldCentric) {
            powers = ManualDrive.fieldCentric(powers, heading);
        }

        if (Math.abs(turnInput) > turnDeadband) {
            // Driver is turning: pass the stick straight through
            headingState = HeadingState.TURNING;
            targetHeading = heading;
        } else {
            if (headingState != HeadingState.LOCKED) {
                headingState = HeadingState.SETTLING;
                targetHeading = heading;

                if (Math.abs(angularVelocity) < settleRate) {
                    lockHeading();
                }
            }

            // While settling the error is zero, so only kD acts and brakes the rotation
            powers = ManualDrive.headingLock(follower, headingController, powers, targetHeading);
        }

        follower.manual(powers);

        telemetryM.addData("Drive Mode", fieldCentric ? "Field-centric" : "Robot-centric");
        telemetryM.addData("Heading State", headingState);
        telemetryM.addData("Target Heading", Math.toDegrees(targetHeading));
        telemetryM.addData("Actual Heading", Math.toDegrees(heading));
        telemetryM.addData("Angular Velocity", Math.toDegrees(angularVelocity));
        telemetryM.addData("Turn Power", powers.turn());
        telemetryM.addData("X", follower.pose().x());
        telemetryM.addData("Y", follower.pose().y());
        telemetryM.update(telemetry);
    }

    private void lockHeading() {
        targetHeading = follower.pose().heading();
        headingController.reset();
        headingState = HeadingState.LOCKED;
    }
}
