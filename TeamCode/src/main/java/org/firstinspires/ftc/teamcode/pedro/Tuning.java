package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.KiwiTuner;

public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure kiwiTuner() {
        return new KiwiTuner();
    }
}
