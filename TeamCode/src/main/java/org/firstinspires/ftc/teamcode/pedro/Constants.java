package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.kiwi.Kiwi;
import org.firstinspires.ftc.teamcode.kiwi.KiwiConfig;

/**
 * The drive constants for Pedro Pathing: the drivetrain, the Pinpoint localizer and Foresight's gains.
 *
 * These are the real robot's values. The BIOBUZZ sim doesn't need its own set here: its virtual
 * Control Hub applies sim-only values from its vhub/sim-overrides.txt to these public static
 * fields by name, so keep their names (drivetrainConfig, localizerConfig, foresightConfig and
 * foresightTuned) as they are.
 *
 * Values marked TODO are placeholders until the robot is tuned.
 */
@Configurable
public class Constants {
    public static KiwiConfig drivetrainConfig = new KiwiConfig(c -> {
        c.motorAName.set("MotorA");
        c.motorBName.set("MotorB");
        c.motorCName.set("MotorC");
        c.motorADirection.set(DcMotorSimple.Direction.REVERSE);
        c.motorBDirection.set(DcMotorSimple.Direction.REVERSE);
        c.motorCDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    /**
     * The Pinpoint. Every value marked TODO is a placeholder, not a measurement: run the Pinpoint
     * tuner (Tuning.pinpointTuner) on the robot and copy its results in.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.8451992545540878);
        c.yPodOffset.set(-5.879274503452572);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /**
     * Foresight's gains: NOT YET TUNED on the robot. These came from the simulator and definitely
     * won't match the real robot. Run Tuning.foresightTuner on the robot, paste its results here,
     * then set foresightTuned to true.
     */

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.9747010794049246);
                Controller secondaryTranslationalForward = Controller.proportional(0.3601260043476096);
                Controller primaryTranslationalLateral = Controller.proportional(1.1064300871596373);
                Controller secondaryTranslationalLateral = Controller.proportional(0.40879635284906257);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.025490154218181287));
                c.brake.set(Controller.proportionalFeedforward(0.021666631085454094));

                c.headingFeedback.set(Controller.proportional(4.340307443345062));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.057785019704679225, 0.006815977953388491));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0965694466630066, 0.09762157253729346));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0031875694475989374, 0.0027674051958705196));

                c.maxAchievableForwardVelocity.set(58.35108469254358);
                c.maxAchievableStrafeVelocity.set(57.56626316979127);
                c.naturalForwardDeceleration.set(57.0944241085665);
                c.naturalStrafeDeceleration.set(55.05474076076255);
            }
    );

    /**
     * Whether foresightConfig holds gains tuned for the robot it's running on. TeleOpDrive's
     * Foresight heading lock is only used when it is: untuned gains make its brake prediction
     * wrong, so the PID heading lock is used instead. Set it to true once the robot is tuned. The
     * sim sets it to true in its sim-overrides.txt, as these gains came from the sim.
     */
    public static boolean foresightTuned = true;

    /** Builds the follower. */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Kiwi(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
