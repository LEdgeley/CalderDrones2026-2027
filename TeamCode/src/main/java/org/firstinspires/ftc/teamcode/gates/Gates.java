package org.firstinspires.ftc.teamcode.gates;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Gates {
    public static double OPEN = 1.0, CLOSED = 0.0;
    public static long FEED_MS = 250;   // how long a gate stays open per ball

    private final Servo pollen, nectar;
    private final ElapsedTime pollenTimer = new ElapsedTime(), nectarTimer = new ElapsedTime();

    // Remembered states, so each servo is only written when its gate changes
    private boolean pollenOpen, nectarOpen;

    public Gates(HardwareMap hw) {
        pollen = hw.get(Servo.class, "pollenGate");
        nectar = hw.get(Servo.class, "nectarGate");
        pollen.setPosition(CLOSED);
        nectar.setPosition(CLOSED);
        pollenOpen = false;
        nectarOpen = false;
    }

    public void feedPollen() {
        if (!pollenOpen) {
            pollen.setPosition(OPEN);
            pollenOpen = true;
        }
        pollenTimer.reset();
    }

    public void feedNectar() {
        if (!nectarOpen) {
            nectar.setPosition(OPEN);
            nectarOpen = true;
        }
        nectarTimer.reset();
    }

    /** Call every loop: closes each gate again after FEED_MS. */
    public void update() {
        if (pollenOpen && pollenTimer.milliseconds() > FEED_MS) {
            pollen.setPosition(CLOSED);
            pollenOpen = false;
        }
        if (nectarOpen && nectarTimer.milliseconds() > FEED_MS) {
            nectar.setPosition(CLOSED);
            nectarOpen = false;
        }
    }

    public boolean isPollenOpen() {
        return pollenOpen;
    }

    public boolean isNectarOpen() {
        return nectarOpen;
    }
}
