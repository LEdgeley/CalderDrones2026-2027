package org.firstinspires.ftc.teamcode.gates;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The Pollen and Nectar feed gates. feedPollen() / feedNectar() open a gate for feedMs to let one
 * ball through; call update() every loop to close them again.
 */
@Configurable
public class Gates {
    public static double openPosition = 1.0;
    public static double closedPosition = 0.0;
    /** How long a gate stays open per ball. */
    public static long feedMs = 250;

    private final Servo pollen, nectar;
    private final ElapsedTime pollenTimer = new ElapsedTime(), nectarTimer = new ElapsedTime();

    // Remembered states, so each servo is only written when its gate changes
    private boolean pollenOpen = false, nectarOpen = false;

    public Gates(HardwareMap hardwareMap) {
        pollen = hardwareMap.get(Servo.class, "pollenGate");
        nectar = hardwareMap.get(Servo.class, "nectarGate");
        pollen.setPosition(closedPosition);
        nectar.setPosition(closedPosition);
    }

    public void feedPollen() {
        if (!pollenOpen) {
            pollen.setPosition(openPosition);
            pollenOpen = true;
        }
        pollenTimer.reset();
    }

    public void feedNectar() {
        if (!nectarOpen) {
            nectar.setPosition(openPosition);
            nectarOpen = true;
        }
        nectarTimer.reset();
    }

    /** Call every loop: closes each gate again after feedMs. */
    public void update() {
        if (pollenOpen && pollenTimer.milliseconds() > feedMs) {
            pollen.setPosition(closedPosition);
            pollenOpen = false;
        }
        if (nectarOpen && nectarTimer.milliseconds() > feedMs) {
            nectar.setPosition(closedPosition);
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
