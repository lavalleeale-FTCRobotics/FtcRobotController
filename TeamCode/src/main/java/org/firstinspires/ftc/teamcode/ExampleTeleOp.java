package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    DcMotor FLMotor;
    DcMotor FRMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;

    DcMotor duckSpinner;
    DcMotor odoMotor;

    OptimizedRobot robot;
    OptimizedController controller;

    Telemetry.Log log;

    @Override
    public void init() {

        FLMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        FRMotor = hardwareMap.dcMotor.get("frontRightMotor");
        BLMotor = hardwareMap.dcMotor.get("backLeftMotor");
        BRMotor = hardwareMap.dcMotor.get("backRightMotor");
        duckSpinner = hardwareMap.dcMotor.get("duckSpinner");
        odoMotor = hardwareMap.dcMotor.get("odometry");
        log = telemetry.log();

        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);

        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());
        BRMotor = robot.getMotor("BRMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.FORWARD);
        FRMotor = robot.getMotor("FRMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.FORWARD);
        FLMotor = robot.getMotor("FLMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.REVERSE);
        BLMotor = robot.getMotor("BLMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.REVERSE);
        controller = new OptimizedController(gamepad1);
        log = telemetry.log();

        odoMotor.setPower((gamepad1.dpad_up) ? -0.2 : ((gamepad1.dpad_down) ? 0.2 : 0));
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {
//        double drive = gamepad1.left_stick_y;
//        double strafe = gamepad1.left_stick_x;
//        double twist = gamepad1.right_stick_x;
//        double[] speeds = {
//                (drive + strafe + twist),
//                (drive - strafe - twist),
//                (drive - strafe + twist),
//                (drive + strafe - twist)
//        };
//        double max = Math.abs(speeds[0]);
//        for (int i = 1; i < speeds.length; i++) {
//            max = Math.max(max, Math.abs(speeds[i]));
//        }
//        if (max > 1) {
//            for (int i = 0; i < speeds.length; i++) {
//                speeds[i] /= max;
//            }
//        }
//
//
//        FLMotor.setPower(speeds[0]);
//        FRMotor.setPower(speeds[1]);
//        BLMotor.setPower(speeds[2]);
//        BRMotor.setPower(speeds[3]);

        FRMotor.setPower(gamepad1.right_stick_y * 0.9);
        BRMotor.setPower(gamepad1.right_stick_y * 0.9);
        FLMotor.setPower(gamepad1.left_stick_y * 0.9);
        BLMotor.setPower(gamepad1.left_stick_y * 0.9);

        duckSpinner.setPower(-gamepad1.right_trigger);
        log.add(String.valueOf(-gamepad1.right_trigger));

        double drive = controller.getFloat(OptimizedController.Key.LEFT_STICK_Y);
        double strafe = -controller.getFloat(OptimizedController.Key.LEFT_STICK_X);
        double twist = -controller.getFloat(OptimizedController.Key.RIGHT_STICK_X);
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };
        double max = Math.abs(speeds[0]);
        for (int i = 1; i < speeds.length; i++) {
            max = Math.max(max, Math.abs(speeds[i]));
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] /= max;
            }
        }
        FLMotor.setPower(speeds[0]);
        FRMotor.setPower(speeds[1]);
        BLMotor.setPower(speeds[2]);
        BRMotor.setPower(speeds[3]);
    }
}
