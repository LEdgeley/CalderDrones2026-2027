package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kiwi.KiwiConfig;

public class Constants {
    public static KiwiConfig drivetrainConfig = new KiwiConfig(c -> {
        c.motorAName.set("motorA");
        c.motorBName.set("motorB");
        c.motorCName.set("motorC");
        c.motorADirection.set(DcMotorSimple.Direction.FORWARD);
        c.motorBDirection.set(DcMotorSimple.Direction.FORWARD);
        c.motorCDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(false);
    });

    public static Follower create(HardwareMap h) {
        // return new Follower(Localizer, new Kiwi(h, drivetrainConfig), Foresight);
        return null;
    }
}
