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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.gates.Gates;
import org.firstinspires.ftc.teamcode.launch.LaunchModel;
import org.firstinspires.ftc.teamcode.launch.RpmTable;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.firstinspires.ftc.teamcode.vision.HiveCells;
import org.firstinspires.ftc.teamcode.vision.HiveTarget;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;

/**
 * Driving test TeleOp using Pedro Pathing 3's manual drive and heading lock.
 *
 * Left stick translates, right stick turns. Releasing the right stick brakes the rotation
 * and then holds the heading the robot settled at. Circle toggles robot/field-centric.
 * Hold the left bumper for slow mode or the right bumper for sprint.
 * Gains are live-tunable from Panels.
 */
@TeleOp(name = "More Than Driving Test", group = "TeleOp")
@Configurable
public class MoreThanDrivingTestTeleOp extends OpMode {

    public static Pose startingPose = new Pose(72, 10.44, Math.toRadians(90));
    /** Which HIVE to aim at. */
    public static Alliance alliance = Alliance.RED;

    // Heading lock gains (turn power per radian of error, and per rad/s of angular velocity for kD)
    public static double kP = 0.8;
    public static double kI = 0.0;
    public static double kD = 0.08;
    public static double kS = 0.0; // Static feedforward to overcome turning friction

    // Driver input tuning
    public static double turnDeadband = 0.05;
    public static double turnScale = 1.0;

    // Speed scales: hold left bumper for slow, right bumper for sprint, neither or both for normal
    public static double slowSpeed = 0.25;
    public static double normalSpeed = 0.66;
    public static double sprintSpeed = 1.0;

    /** Angular velocity (rad/s) below which the robot is considered settled and the heading is locked. */
    public static double settleRate = 0.3;

    private enum HeadingState { TURNING, SETTLING, LOCKED }

    private Follower follower;
    private TelemetryManager telemetryM;
    private Controller headingController;

    private HeadingState headingState = HeadingState.LOCKED;

    /** Launch speed used when the camera can't see our tags. */
    private static final double DEFAULT_LAUNCH_SPEED_MPS = 6.0;

    private Flywheel pollenLauncher;
    private Flywheel nectarLauncher;
    private double pollenLauncherTargetRPM = LaunchModel.rpmForSpeed(DEFAULT_LAUNCH_SPEED_MPS);
    private double nectarLauncherTargetRPM = LaunchModel.rpmForSpeed(DEFAULT_LAUNCH_SPEED_MPS);
    // RPM against distance for each launcher, worked out with the flight model at INIT
    private RpmTable pollenRpmTable;
    private RpmTable nectarRpmTable;

    private Gates gates;
    private LimelightCamera camera;
    /** Square aims at our HIVE until the driver turns. */
    private boolean autoAim = false;
    private boolean aimFromVision = false;
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

        nectarLauncher = new Flywheel(hardwareMap, "NectarFlywheel", DcMotorSimple.Direction.FORWARD);
        pollenLauncher = new Flywheel(hardwareMap, "PollenFlywheel", DcMotorSimple.Direction.REVERSE);

        nectarLauncher.reset();
        pollenLauncher.reset();

        gates = new Gates(hardwareMap);
        camera = new LimelightCamera(hardwareMap, alliance);

        // Takes about a second on the Control Hub
        pollenRpmTable = new RpmTable(LaunchModel.Launcher.POLLEN);
        nectarRpmTable = new RpmTable(LaunchModel.Launcher.NECTAR);

        telemetry.addData("Status", "Initialized. Circle toggles robot/field-centric.");
        telemetry.update();
    }

    @Override
    public void start() {

        lockHeading();
        nectarLauncher.reset();
        pollenLauncher.reset();
    }

    @Override
    public void loop() {
        camera.setAlliance(alliance);
        HiveTarget hiveTarget = camera.findTarget();

        // Flywheel speeds from the distance to the CELL, or the default speed when no tags are in view
        if (hiveTarget != null) {
            pollenLauncherTargetRPM = pollenRpmTable.rpmAt(hiveTarget.horizontalDistanceFrom(
                    LaunchModel.Launcher.POLLEN.axleForward, LaunchModel.Launcher.POLLEN.axleLeft));
            nectarLauncherTargetRPM = nectarRpmTable.rpmAt(hiveTarget.horizontalDistanceFrom(
                    LaunchModel.Launcher.NECTAR.axleForward, LaunchModel.Launcher.NECTAR.axleLeft));
        } else {
            pollenLauncherTargetRPM = LaunchModel.rpmForSpeed(DEFAULT_LAUNCH_SPEED_MPS);
            nectarLauncherTargetRPM = LaunchModel.rpmForSpeed(DEFAULT_LAUNCH_SPEED_MPS);
        }

        nectarLauncher.runToRPM(nectarLauncherTargetRPM);
        pollenLauncher.runToRPM(pollenLauncherTargetRPM);
        gates.update();
        follower.update();

        if (gamepad1.circleWasPressed()) {
            fieldCentric = !fieldCentric;
        }

        // Only open a gate when its flywheel is at speed; presses while it isn't are ignored
        if (gamepad1.rightTriggerWasPressed() && pollenLauncher.isReady()) {
            gates.feedPollen();
        }

        if (gamepad1.leftTriggerWasPressed() && nectarLauncher.isReady()) {
            gates.feedNectar();
        }

        double heading = follower.pose().heading();
        double angularVelocity = follower.twist().omega;

        if (gamepad1.squareWasPressed()) {
            autoAim = true;
            aimFromVision = false;
            headingController.reset();
        }

        String speedMode;
        double speed;
        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            speedMode = "Slow";
            speed = slowSpeed;
        } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            speedMode = "Sprint";
            speed = sprintSpeed;
        } else {
            speedMode = "Normal";
            speed = normalSpeed;
        }

        // +forward is forward, +strafe is left and +turn is CCW, so the X axes are negated
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turnInput = -gamepad1.right_stick_x;

        // Speed scaling applies to driver input only, so the heading lock keeps full authority
        DrivePowers powers = new DrivePowers(forward * speed, strafe * speed, turnInput * turnScale * speed);

        if (fieldCentric) {
            powers = ManualDrive.fieldCentric(powers, heading);
        }

        if (Math.abs(turnInput) > turnDeadband) {
            // Driver is turning: pass the stick straight through
            headingState = HeadingState.TURNING;
            targetHeading = heading;
            autoAim = false;
        } else {
            if (autoAim) {
                if (hiveTarget != null) {
                    // Tags in view: point straight at the CELL's open face
                    targetHeading = hiveTarget.targetHeading(heading);
                    aimFromVision = true;
                } else if (!aimFromVision) {
                    // No tags yet: point roughly at our nearest CELL using odometry. Once the camera has
                    // given a heading, keep it if the tags drop out rather than going back to the rough one.
                    targetHeading = HiveCells.headingToNearestCell(alliance, follower.pose().x(), follower.pose().y());
                }
                headingState = HeadingState.LOCKED;
            } else if (headingState != HeadingState.LOCKED) {
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
        telemetryM.addData("Speed Mode", speedMode);
        telemetryM.addData("Alliance", alliance);
        telemetryM.addData("Auto Aim", !autoAim ? "Off" : (aimFromVision ? "Vision" : "Odometry"));
        telemetryM.addData("Tags Seen", hiveTarget != null ? hiveTarget.tagCount : 0);
        if (hiveTarget != null) {
            telemetryM.addData("Target CELL", hiveTarget.cell);
            telemetryM.addData("Target Distance (m)", hiveTarget.horizontalDistance());
            telemetryM.addData("Target Bearing", Math.toDegrees(hiveTarget.bearing()));
        }
        telemetryM.addData("Heading State", headingState);
        telemetryM.addData("Target Heading", Math.toDegrees(targetHeading));
        telemetryM.addData("Actual Heading", Math.toDegrees(heading));
        telemetryM.addData("Angular Velocity", Math.toDegrees(angularVelocity));
        telemetryM.addData("Turn Power", powers.turn());
        telemetryM.addData("X", follower.pose().x());
        telemetryM.addData("Y", follower.pose().y());

        telemetryM.addData("nectar.targetRPM", nectarLauncher.targetRPM());
        telemetryM.addData("nectar.filteredRPM", nectarLauncher.filteredRPM());
        telemetryM.addData("nectar.power", nectarLauncher.power());
        telemetryM.addData("nectar.ready", nectarLauncher.isReady());
        telemetryM.addData("pollen.targetRPM", pollenLauncher.targetRPM());
        telemetryM.addData("pollen.filteredRPM", pollenLauncher.filteredRPM());
        telemetryM.addData("pollen.power", pollenLauncher.power());
        telemetryM.addData("pollen.ready", pollenLauncher.isReady());
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        camera.stop();
    }

    private void lockHeading() {
        targetHeading = follower.pose().heading();
        headingController.reset();
        headingState = HeadingState.LOCKED;
    }
}
