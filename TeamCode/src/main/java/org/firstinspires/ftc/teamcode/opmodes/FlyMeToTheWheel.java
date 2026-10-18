
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

package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Fly Me To The Wheel", group="Linear OpMode")
@Configurable
public class FlyMeToTheWheel extends LinearOpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private DcMotorEx flywheel = null;


    public static double Kp = 0.005;
    public static double Kd = 0.000004;
    public static double Ki = 0.000002;
    public static double targetRPM = 5000;
    @Override
    public void runOpMode() {

        // Setup a variable for each drive wheel to save power level for telemetry
        double power;


        double integralSum = 0;

        double lastError = 0;
        double lastOut = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        flywheel  = hardwareMap.get(DcMotorEx.class, "Launcher");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        power = 0.0;

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        timer.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double currentRPM = flywheel.getVelocity() / 28.0 * 60.0;
            // calculate the error
            double error = targetRPM - currentRPM;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = Range.clip((Kp * error) + (Ki * integralSum) + (Kd * derivative), 0.0, 1.0);
            if ((out - lastOut)< -0.01)
            {
                out = lastOut - 0.01;
            }

            else if ((out - lastOut) > 0.01) {
                {
                    out = lastOut + 0.01;
                }
            }

            // Send calculated power to wheels
            flywheel.setPower(out);
            lastOut = out;
            lastError = error;

            // reset the timer for next time
            timer.reset();



            // Show the elapsed game time and wheel power.
            panelsTelemetry.addData("targetRPM", targetRPM);
            panelsTelemetry.addData("currentRPM", currentRPM);
            panelsTelemetry.addData("out", out);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("targetRPM", "targetRPM (%.2f)", targetRPM);
            telemetry.addData("currentRPM", "currentRPM (%.2f)", currentRPM);
            telemetry.addData("error", "error (%.2f)", error);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", out, out);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", out, out);
            panelsTelemetry.update(telemetry);
        }
    }
}