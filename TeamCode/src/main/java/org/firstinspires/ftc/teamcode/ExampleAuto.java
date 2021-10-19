package org.firstinspires.ftc.teamcode;

<<<<<<< Updated upstream
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Test Auto")
public class ExampleAuto extends LinearOpMode {

    DcMotor FLMotor, FRMotor, BLMotor, BRMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        FLMotor = hardwareMap.dcMotor.get("FLMotor");
        FRMotor = hardwareMap.dcMotor.get("FRMotor");
        BLMotor = hardwareMap.dcMotor.get("BLMotor");
        BRMotor = hardwareMap.dcMotor.get("BRMotor");
        FRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        setMotorPower(0.9);

        sleep(200);

        setMotorPower(0);

        turn(90);

        setMotorPower(0.9);

        sleep(200);

        setMotorPower(0);
    }

    private void setMotorPower(double power){
        FLMotor.setPower(power);
        FRMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
    }

    private void turn(int degrees){
        if(degrees == 0)
            throw new IllegalArgumentException();
        if(degrees > 0) {
            FRMotor.setPower(0.5);
            BRMotor.setPower(0.5);
            FLMotor.setPower(-0.5);
            BLMotor.setPower(-0.5);
        } else {
            FRMotor.setPower(-0.5);
            BRMotor.setPower(-0.5);
            FLMotor.setPower(0.5);
            BLMotor.setPower(0.5);
        }
        sleep(degrees * 10);

        setMotorPower(0);
    }
}
=======
import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.nullness.Opt;
import org.firstinspires.ftc.internal.OptimizedController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.pipelines.FinalWABOTPipeline;
import org.firstinspires.ftc.teamcode.internal.OpenCVLoader;

@Autonomous(name = "Example Auto")
public class ExampleAuto extends LinearOpMode {
    DcMotor DuckSpinner;
    Telemetry.Log log;
    private OpenCVLoader loader;

    @Override
    public void runOpMode() throws InterruptedException {
        loader = new OpenCVLoader(hardwareMap, true, new FinalWABOTPipeline());
        log = telemetry.log();
        waitForStart();
        log.add(loader.pipeline.getVisionOutput());
    }
}
>>>>>>> Stashed changes
