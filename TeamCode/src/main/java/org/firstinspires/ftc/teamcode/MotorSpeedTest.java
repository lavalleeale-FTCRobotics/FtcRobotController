package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.nullness.Opt;
import org.firstinspires.ftc.internal.OptimizedController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Motor Speed Test")
public class MotorSpeedTest extends OpMode {
    DcMotor DuckSpinner;
    Telemetry.Log log;
    OptimizedController controller;
    double test = 1;

    @Override
    public void init() {
        DuckSpinner = hardwareMap.dcMotor.get("Duck spinner");
        log = telemetry.log();
        controller = new OptimizedController(gamepad1);
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
