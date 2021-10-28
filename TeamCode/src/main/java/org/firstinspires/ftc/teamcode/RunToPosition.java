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

@TeleOp(name = "Run To Position")
public class RunToPosition extends OpMode {
    DcMotor DuckSpinner;
    Telemetry.Log log;
    OptimizedController controller;
    OptimizedRobot robot;

    @Override
    public void init() {
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());
        DuckSpinner = robot.getMotor("BRMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DuckSpinner.setTargetPosition(0);
        DuckSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        log = telemetry.log();
        controller = new OptimizedController(gamepad1);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {
        log.clear();
        DuckSpinner.setPower(1);
        if (controller.getOnPress(OptimizedController.Key.A)) {
            DuckSpinner.setTargetPosition(5);
        } else if (controller.getOnPress(OptimizedController.Key.B)) {
            DuckSpinner.setTargetPosition(-5);
        }
        log.add(DuckSpinner.getCurrentPosition() + ", " + DuckSpinner.getTargetPosition() + ", " + DuckSpinner.isBusy());
    }
}
