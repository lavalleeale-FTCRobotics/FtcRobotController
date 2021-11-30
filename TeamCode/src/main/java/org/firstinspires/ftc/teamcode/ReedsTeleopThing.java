package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@TeleOp(name = "Reeds Teleop Thing")
public class ReedsTeleopThing extends OpMode {
    CRServo leftServo;
    CRServo rightServo;
    DcMotor armMotor;

    Telemetry.Log log;

    boolean clawClosed = false;

    @Override
    public void init() {
        leftServo = hardwareMap.crservo.get("Left Servo");
        rightServo = hardwareMap.crservo.get("Right Servo");
        armMotor = hardwareMap.dcMotor.get("arm");

        log = telemetry.log();
    }

    @Override
    public void loop(){
        if (gamepad1.a)
            clawClosed = true;
        else if (gamepad1.b)
            clawClosed = false;
        leftServo.setPower((clawClosed) ? 0.5 : -0.5);
        rightServo.setPower((clawClosed) ? -0.5 : 0.5);
        armMotor.setPower(Math.pow(gamepad1.right_stick_y, 3));
    }
}
