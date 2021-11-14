package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.examples.SampleControllerMapping;

@TeleOp(name = "Final TeleOP")
public class FinalTeleOp extends OpMode {
    DcMotor duckSpinner, arm, odometry;
    OptimizedRobot robot;
    OptimizedController controller, controller2;
    CRServo leftServo, rightServo;
    Telemetry.Log log;
    int armLowPoint = 128;
    int armMidPoint = 300;
    int armHighPoint = 900;
    int backPosition = -3000;
    int offset = 0;

    @Override
    public void init() {
        robot = new OptimizedRobot(telemetry, hardwareMap, new SampleControllerMapping());
        duckSpinner = robot.getMotor("duckSpinner");
        arm = robot.getMotor("arm", DcMotor.RunMode.RUN_TO_POSITION);
        odometry = robot.getMotor("odometry");
        leftServo = hardwareMap.crservo.get("Left Servo");
        rightServo = hardwareMap.crservo.get("Right Servo");
        controller = new OptimizedController(gamepad1);
        controller2 = new OptimizedController(gamepad2);
        log = telemetry.log();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {
        // Duck Spinner
        if (controller2.getBool(robot.getControl("Duck"))) {
            duckSpinner.setPower(RobotConfig.SPINNER_SPEED);
        } else if (controller2.getBool(robot.getControl("ReverseDuck"))) {
            duckSpinner.setPower(-RobotConfig.SPINNER_SPEED);
        } else {
            duckSpinner.setPower(controller2.getFloat(robot.getControl("DuckSpeedReverse")) - controller2.getFloat(robot.getControl("DuckSpeed")));
        }

        // Arm
        // If manual arm control is off and the Arm Snap button is pressed swap position
        if (controller2.getToggle(robot.getControl("ArmControl"))) {
            // Set Power to "ArmSmooth" Float
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(controller2.getFloat(robot.getControl("ArmSmooth")));
        } else {
            // Run to position with maximum power
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.7);
            if (controller2.getToggle(robot.getControl("ArmSwap"))) {
                arm.setTargetPosition(controller2.getToggle(robot.getControl("ArmSnap")) ? offset + backPosition + (controller2.getToggle(robot.getControl("ArmMid")) ? armMidPoint : armHighPoint) : offset + backPosition + armLowPoint);
            } else {
                arm.setTargetPosition(controller2.getToggle(robot.getControl("ArmSnap")) ? offset - (controller2.getToggle(robot.getControl("ArmMid")) ? armMidPoint : armHighPoint): offset - armLowPoint);
            }
        }
        // Change Arm offset When Right bumper pressed
        if (controller2.getOnPress(robot.getControl("ArmReset"))) {
            offset = arm.getCurrentPosition();
        }

        // Upon claw toggle, alternate left and right claw power
        leftServo.setPower(controller.getToggle(robot.getControl("Claw")) ? -0.5: 0.5);
        rightServo.setPower(controller.getToggle(robot.getControl("Claw")) ? 0.5: -0.5);

        // Set odometry power to the left stick cubed, to add mild curving
        odometry.setPower(Math.pow(controller2.getFloat(robot.getControl("Odometry")), 3));
        log.add(String.valueOf(arm.getCurrentPosition()));
        log.clear();

        // Main drive method
        robot.updateDrive(controller, controller2, true, false, 0.7d, OptimizedRobot.RobotDirection.FRONT, OptimizedRobot.RobotDirection.FRONT, false);
    }
}