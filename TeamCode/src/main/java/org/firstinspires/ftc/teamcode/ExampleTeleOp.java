package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.internal.OptimizedController;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.examples.SampleControllerMapping;
import org.firstinspires.ftc.teamcode.examples.SampleHardwareAliasMapping;

import java.util.Arrays;

@TeleOp(name = "Example TeleOP")
public class ExampleTeleOp extends OpMode {
    DcMotor duckSpinner;
    DcMotor arm;
    OptimizedRobot robot;
    OptimizedController controller;
    OptimizedController controller2;
    Telemetry.Log log;

    @Override
    public void init() {
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());
        duckSpinner = robot.getMotor("duckSpinner");
        arm = robot.getMotor("arm");
        controller = new OptimizedController(gamepad1);
        controller2 = new OptimizedController(gamepad2);
        log = telemetry.log();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {
        if (controller.getBool(OptimizedController.Key.A)) {
            duckSpinner.setPower(-0.6);
        } else {
            duckSpinner.setPower(0);
        }
        robot.updateDrive(controller, controller2, true, false, 1d, OptimizedRobot.RobotDirection.FRONT, OptimizedRobot.RobotDirection.FRONT, false);
        arm.setPower(controller2.getFloat(OptimizedController.Key.RIGHT_STICK_Y));
    }
}
