package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.internal.OptimizedController;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.examples.SampleControllerMapping;
import org.firstinspires.ftc.teamcode.examples.SampleHardwareAliasMapping;

import java.util.Arrays;

@TeleOp(name = "Tank TeleOP")
public class TankTeleOp extends OpMode {
    DcMotor FLMotor;
    DcMotor FRMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;
    OptimizedRobot robot;
    OptimizedController controller;

    @Override
    public void init() {
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());
        BRMotor = robot.getMotor("BRMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.FORWARD);
        FRMotor = robot.getMotor("FRMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.FORWARD);
        FLMotor = robot.getMotor("FLMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.REVERSE);
        BLMotor = robot.getMotor("BLMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.REVERSE);
        controller = new OptimizedController(gamepad1);
    }

    @Override
    public void loop() {
        FLMotor.setPower(controller.getFloat(OptimizedController.Key.LEFT_STICK_Y));
        BLMotor.setPower(controller.getFloat(OptimizedController.Key.LEFT_STICK_Y));
        FRMotor.setPower(controller.getFloat(OptimizedController.Key.RIGHT_STICK_Y));
        BRMotor.setPower(controller.getFloat(OptimizedController.Key.RIGHT_STICK_Y));
    }
}
