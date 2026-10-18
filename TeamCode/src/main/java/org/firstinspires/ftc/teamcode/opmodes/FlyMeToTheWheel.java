
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
import com.pedropathing.controllers.Controller;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * Flywheel velocity test. Holds the "Launcher" motor at targetRPM using Pedro Pathing 3 controllers:
 * PID feedback on the RPM error plus feedforward on the target RPM.
 * All gains and the target can be changed live from Panels.
 */

@TeleOp(name="Fly Me To The Wheel", group="Linear OpMode")
@Configurable
public class FlyMeToTheWheel extends LinearOpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private DcMotorEx flywheel = null;

    // Feedback gains (power per RPM of error)
    public static double Kp = 0.005;
    public static double Kd = 0.000004;
    public static double Ki = 0.000002;

    // Feedforward gains: kV is power per RPM of target, kS is the constant power to overcome friction
    public static double kV = 0.0;
    public static double kS = 0.0;

    /** Largest change in motor power allowed per second (2.0 = zero to full power in 0.5 s). */
    public static double maxPowerRate = 2.0;
    /** Longest loop time used for the slew limit, so a stalled loop can't allow a large jump. */
    private static final double MAX_LOOP_SECONDS = 0.05;
    /** Low-pass filter time constant for the measured RPM in seconds (0 = no filtering). */
    public static double rpmFilterTimeConstant = 0.03;
    public static double targetRPM = 5000;

    @Override
    public void runOpMode() {

        double lastOut = 0;

        // Suppliers let the gains be changed from Panels while running
        Controller flywheelController = Controller.sum(
                Controller.proportional(() -> Kp),
                Controller.integral(() -> Ki),
                Controller.derivative(() -> Kd),
                Controller.proportionalFeedforward(() -> kV),
                Controller.staticTargetFeedforward(() -> kS)
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        flywheel  = hardwareMap.get(DcMotorEx.class, "Launcher");

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        loopTimer.reset();
        flywheelController.reset();

        double filteredRPM = getRPM();
        double rpmRate = 0;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double dt = loopTimer.seconds();
            loopTimer.reset();

            // Low-pass filter the measured RPM, scaled by this loop's duration
            double rawRPM = getRPM();
            double alpha = rpmFilterTimeConstant > 0 ? dt / (rpmFilterTimeConstant + dt) : 1.0;
            double lastFilteredRPM = filteredRPM;
            filteredRPM += alpha * (rawRPM - filteredRPM);
            if (dt > 1e-3) {
                rpmRate = (filteredRPM - lastFilteredRPM) / dt;
            }

            double error = targetRPM - filteredRPM;

            // Passing the filtered RPM's rate of change makes the D term act on the measurement
            double out = Range.clip(flywheelController.calculate(targetRPM, error, rpmRate), 0.0, 1.0);

            // Limit how quickly the power can change, scaled by this loop's duration
            double maxStep = maxPowerRate * Math.min(dt, MAX_LOOP_SECONDS);
            out = Range.clip(out, lastOut - maxStep, lastOut + maxStep);

            // Send calculated power to the flywheel
            flywheel.setPower(out);
            lastOut = out;


            // Show the elapsed game time and wheel power.
            panelsTelemetry.addData("targetRPM", targetRPM);
            panelsTelemetry.addData("rawRPM", rawRPM);
            panelsTelemetry.addData("filteredRPM", filteredRPM);
            panelsTelemetry.addData("out", out);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("targetRPM", "targetRPM (%.2f)", targetRPM);
            telemetry.addData("filteredRPM", "filteredRPM (%.2f)", filteredRPM);
            telemetry.addData("error", "error (%.2f)", error);
            telemetry.addData("Power", "%.2f", out);
            panelsTelemetry.update(telemetry);
        }
    }

    /** Motor RPM from the 28 counts-per-revolution motor encoder. */
    private double getRPM() {
        return flywheel.getVelocity() / 28.0 * 60.0;
    }
}
