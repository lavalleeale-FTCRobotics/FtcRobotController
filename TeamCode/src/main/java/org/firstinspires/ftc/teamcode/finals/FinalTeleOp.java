package org.firstinspires.ftc.teamcode.finals;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.internal.OptimizedController;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.internal.RobotConfig;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Final TeleOP")
public class FinalTeleOp extends OpMode {
    DcMotor duckSpinner, arm, odometry;
    OptimizedRobot robot;
    OptimizedController controller1, controller2;
    CRServo leftServo, rightServo;
    Telemetry.Log log;
    int armLowPoint = 128;
    int armMidPoint = 300;
    int armHighPoint = 900;
    int backPosition = -3000;

    @Override
    public void init() {
        controller1 = new OptimizedController(gamepad1);
        controller2 = new OptimizedController(gamepad2);

        log = telemetry.log();

        robot = new OptimizedRobot(controller1, controller2, telemetry, hardwareMap, new FreightFrenzyControllerMapping());
        duckSpinner = robot.getMotor("duckSpinner");
        arm = robot.getMotor("arm", DcMotor.RunMode.RUN_TO_POSITION);
        odometry = robot.getMotor("odometry");

        leftServo = hardwareMap.crservo.get("Left Servo");
        rightServo = hardwareMap.crservo.get("Right Servo");
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {
        // Duck Spinner
        if (robot.getControl("Duck")) {
            duckSpinner.setPower(RobotConfig.SPINNER_SPEED);
        } else if (robot.getControl("ReverseDuck")) {
            duckSpinner.setPower(-RobotConfig.SPINNER_SPEED);
        } else {
            duckSpinner.setPower(robot.getControlFloat("DuckSpeedReverse") - robot.getControlFloat("DuckSpeed"));
        }

        // Arm
        if (robot.getControl("ArmControl")) {
            // Set Power to "ArmSmooth" Float
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(Math.pow(robot.getControlFloat("ArmSmooth"), 3));
        } else {
            // Run to position with 0.7 power
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(RobotConfig.ARM_POWER);
            if (robot.getControl("ArmSwap")) {
                if (robot.getControl("ArmMid")) {
                    arm.setTargetPosition(robot.getControl("ArmSnap") ? backPosition + armMidPoint : backPosition + armLowPoint);
                } else {
                    arm.setTargetPosition(robot.getControl("ArmSnap") ? backPosition + armHighPoint : backPosition + armLowPoint);
                }
            } else {
                if (robot.getControl("ArmMid")) {
                    arm.setTargetPosition(robot.getControl("ArmSnap") ? -armMidPoint : -armLowPoint);
                } else {
                    arm.setTargetPosition(robot.getControl("ArmSnap") ? -armHighPoint : -armLowPoint);
                }
            }
        }

        // Upon claw toggle, alternate left and right claw power
        leftServo.setPower(robot.getControl("Claw") ? -0.5 : 0.5);
        rightServo.setPower(robot.getControl("Claw") ? 0.5 : -0.5);

        // Set odometry power to the left stick cubed, to add mild curving
        odometry.setPower(Math.pow(robot.getControlFloat("Odometry"), 3));
        log.add("Arm Position: " + arm.getCurrentPosition());
        log.clear();

        // Main drive method
        robot.updateDrive(controller1, controller2, true, false, 0.7d, OptimizedRobot.RobotDirection.FRONT, OptimizedRobot.RobotDirection.FRONT, false);
    }
}