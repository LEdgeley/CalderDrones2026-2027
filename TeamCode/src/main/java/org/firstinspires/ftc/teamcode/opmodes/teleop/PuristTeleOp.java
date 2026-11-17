package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * The simplest possible drive: gamepad sticks straight to the three Kiwi wheels, robot-centric.
 * No Pedro, no Foresight, no field-centric control, no voltage compensation, no smoothing.
 * For checking the wheels' names and directions.
 *
 * Left stick: drive (up = forward, left = strafe left). Right stick x: turn.
 * Uses the motor names and directions from Constants.drivetrainConfig, and the same wheel maths as
 * Kiwi: Motor A at the back (rolling left/right), Motor B front-left, Motor C front-right.
 */
@TeleOp(name = "Purist TeleOp", group = "Test")
public class PuristTeleOp extends OpMode {

    private static final double SIN_60 = Math.sqrt(3) / 2.0;

    private DcMotor motorA, motorB, motorC;

    @Override
    public void init() {
        motorA = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.motorAName.get());
        motorB = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.motorBName.get());
        motorC = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.motorCName.get());
        motorA.setDirection(Constants.drivetrainConfig.motorADirection.get());
        motorB.setDirection(Constants.drivetrainConfig.motorBDirection.get());
        motorC.setDirection(Constants.drivetrainConfig.motorCDirection.get());
    }

    @Override
    public void loop() {
        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        // Pedro's convention: +forward, +strafe is left, +turn is anticlockwise (seen from above).
        // The sticks read negative for up and left, hence the minus signs.
        double forward = -ly;
        double strafe = -lx;
        double turn = -rx;

        double a = -strafe + turn;
        double b = -SIN_60 * forward + 0.5 * strafe + turn;
        double c = SIN_60 * forward + 0.5 * strafe + turn;

        // Scale all three down together if any is over full power, keeping the direction of travel
        double max = Math.max(1.0, Math.max(Math.abs(a), Math.max(Math.abs(b), Math.abs(c))));
        a /= max;
        b /= max;
        c /= max;

        motorA.setPower(a);
        motorB.setPower(b);
        motorC.setPower(c);

        telemetry.addData("left_stick_x", "%.2f", lx);
        telemetry.addData("left_stick_y", "%.2f", ly);
        telemetry.addData("right_stick_x", "%.2f", rx);
        telemetry.addData("MotorA power", "%.2f", a);
        telemetry.addData("MotorB power", "%.2f", b);
        telemetry.addData("MotorC power", "%.2f", c);
    }

    @Override
    public void stop() {
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
    }
}
