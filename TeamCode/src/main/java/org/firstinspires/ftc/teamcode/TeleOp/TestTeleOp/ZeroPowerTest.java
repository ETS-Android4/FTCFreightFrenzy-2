package org.firstinspires.ftc.teamcode.TeleOp.TestTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;

@TeleOp(name="ZeroPowerDT")
public class ZeroPowerTest extends LinearOpMode {

    MainBase base = null;


    @Override
    public void runOpMode() {
        custom_init();
        waitForStart();
        while (opModeIsActive()) {
            custom_loop();
        }
    }

    public void custom_init() {
        base = new MainBase();
        base.init(hardwareMap);

        telemetry.addData("Initialization Complete!", "");
        telemetry.update();
    }

    public void custom_loop() {

        base.leftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        base.rightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("LeftDT Encoders: ", base.leftDT.getCurrentPosition());
        telemetry.addData("RightDT Encoders: ", base.rightDT.getCurrentPosition());
        telemetry.update();
    }
}