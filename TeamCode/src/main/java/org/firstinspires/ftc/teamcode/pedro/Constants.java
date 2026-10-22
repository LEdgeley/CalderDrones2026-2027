package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kiwi.Kiwi;
import org.firstinspires.ftc.teamcode.kiwi.KiwiConfig;

public class Constants {
    public static KiwiConfig drivetrainConfig = new KiwiConfig(c -> {
        c.motorAName.set("MotorA");
        c.motorBName.set("MotorB");
        c.motorCName.set("MotorC");
        c.motorADirection.set(DcMotorSimple.Direction.REVERSE);
        c.motorBDirection.set(DcMotorSimple.Direction.REVERSE);
        c.motorCDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Kiwi(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
