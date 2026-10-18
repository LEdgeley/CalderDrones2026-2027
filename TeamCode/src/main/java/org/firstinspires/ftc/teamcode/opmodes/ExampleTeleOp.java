package org.firstinspires.ftc.teamcode.opmodes;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class ExampleTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(72,72); //See ExampleAuto to understand how to use this
    public static PIDFCoefficients headingCoefficients = new PIDFCoefficients(1.2,0.0,0.08,0.01);
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private boolean robotCentric = false;
    private boolean driveReverse = false;
    private double slowModeMultiplier = 0.5;

    private PIDFController headingTeleOpPIDF;

    @Override
    public void init() {
        headingTeleOpPIDF = new PIDFController(headingCoefficients);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(54, 94))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                .build();
        //Lazy curve generation might look a little different to how we usually make paths,
        //This is because we are using a Lambda expression instead of calling follower.pathbuilder()
        //To use this though, we declare this as a Supplier<PathChain>. And we to call the path chain we instead do pathChain.get()
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            double speed = 1.0;

            if (slowMode) {
                speed = slowModeMultiplier;
            }

            double driveInput = speed * gamepad1.left_stick_y * (driveReverse ? 1.0 : -1.0);
            double strafeInput = speed * gamepad1.left_stick_x * (driveReverse ? 1.0 : -1.0);
            double headingInput = speed * -gamepad1.right_stick_x * 0.5; //limit rotation speed

            double currentHeading = follower.getPose().getHeading();
            double rotationCorrection = 0;

            if (Math.abs(headingInput) < 0.05) {
                // Update the current heading in the PIDF Controller
                headingTeleOpPIDF.updatePosition(currentHeading);
                rotationCorrection = headingTeleOpPIDF.run();
            }
            else {
                // If rotation input exists, update the target heading
                headingTeleOpPIDF.setTargetPosition(currentHeading+Math.toRadians(headingInput*100));
                headingTeleOpPIDF.updatePosition(currentHeading);
                rotationCorrection = headingTeleOpPIDF.run();;
            }

            telemetry.addData("Target Heading", headingTeleOpPIDF.getTargetPosition());
            telemetry.addData("Actual Heading", currentHeading);
            follower.setTeleOpDrive(
                   driveInput,
                    strafeInput,
                    rotationCorrection,
                    robotCentric);

            if (gamepad1.leftBumperWasPressed()) {
                robotCentric = !robotCentric;
            }

        }

        //Automated PathFollowing

        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad1.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        if (gamepad1.bWasPressed()) {
            driveReverse = !driveReverse;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}