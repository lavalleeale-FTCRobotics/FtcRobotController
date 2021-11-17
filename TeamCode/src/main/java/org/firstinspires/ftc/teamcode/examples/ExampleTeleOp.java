package org.firstinspires.ftc.teamcode.examples;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.internal.OptimizedController;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.internal.RobotConfig;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.finals.FreightFrenzyControllerMapping;

@TeleOp(name = "Example TeleOP")
@Disabled
public class ExampleTeleOp extends OpMode {
    DcMotor motor;
    OptimizedRobot robot;
    OptimizedController controller1, controller2;
    Telemetry.Log log;

    @Override
    public void init() {
        controller1 = new OptimizedController(gamepad1);
        controller2 = new OptimizedController(gamepad2);

        log = telemetry.log();

        robot = new OptimizedRobot(controller1, controller2, telemetry, hardwareMap, new ExampleControllerMapping());
        motor = robot.getMotor("duckSpinner");
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {
        // Duck Spinner
        if (robot.getControl("Motor")) {
            motor.setPower(RobotConfig.SPINNER_SPEED);
        } else if (robot.getControl("ReverseDuck")) {
            motor.setPower(-RobotConfig.SPINNER_SPEED);
        } else {
            motor.setPower(robot.getControlFloat("DuckSpeedReverse") - robot.getControlFloat("DuckSpeed"));
        }

        // Show current motor Position
        log.add("Duck Position: " + motor.getCurrentPosition());
        log.clear();

        // Main drive method
        robot.updateDrive(controller1, controller2, true, false, 0.7d, OptimizedRobot.RobotDirection.FRONT, OptimizedRobot.RobotDirection.FRONT, false);
    }
}