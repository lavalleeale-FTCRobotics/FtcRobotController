package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.internal.OptimizedController;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.examples.SampleControllerMapping;
import org.firstinspires.ftc.teamcode.examples.SampleHardwareAliasMapping;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Motor Speed Test")
public class MotorSpeedTest extends OpMode {
    DcMotor DuckSpinner;
    Telemetry.Log log;
    OptimizedController controller;
    double test = 1;
    private OptimizedRobot robot;

    @Override
    public void init() {
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());
        DuckSpinner = hardwareMap.dcMotor.get("Duck spinner");
        log = telemetry.log();
//        controller = robot.setUpVirtualController(gamepad1, OptimizedController.Key.A);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {
        log.clear();
        log.add(String.valueOf(test));
        if (controller.getOnPress(OptimizedController.Key.A)) {
            test += 0.1;
        }
        if (controller.getOnPress(OptimizedController.Key.B)) {
            test -= 0.1;
        }
        DuckSpinner.setPower(test);
    }
}
