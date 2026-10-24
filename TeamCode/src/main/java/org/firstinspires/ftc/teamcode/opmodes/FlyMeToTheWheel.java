
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;


/*
 * Flywheel velocity test. Holds the "Launcher" motor at targetRPM using the shared Flywheel controller
 * (PID feedback plus feedforward, rate-limited power and a filtered RPM reading).
 * It uses the Flywheel default gains; use FlywheelTuner to tune them.
 */

@TeleOp(name="Fly Me To The Wheel", group="Linear OpMode")
@Configurable
public class FlyMeToTheWheel extends LinearOpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public static String motorName = "Launcher";
    public static DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    public static double targetRPM = 5000;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Flywheel flywheel = new Flywheel(hardwareMap, motorName, direction);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        flywheel.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            flywheel.runToRPM(targetRPM);
            double error = targetRPM - flywheel.filteredRPM();


            // Show the elapsed game time and wheel power.
            panelsTelemetry.addData("targetRPM", targetRPM);
            panelsTelemetry.addData("rawRPM", flywheel.rawRPM());
            panelsTelemetry.addData("filteredRPM", flywheel.filteredRPM());
            panelsTelemetry.addData("out", flywheel.power());


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("targetRPM", "targetRPM (%.2f)", targetRPM);
            telemetry.addData("filteredRPM", "filteredRPM (%.2f)", flywheel.filteredRPM());
            telemetry.addData("error", "error (%.2f)", error);
            telemetry.addData("Power", "%.2f", flywheel.power());
            panelsTelemetry.update(telemetry);
        }
    }
}
