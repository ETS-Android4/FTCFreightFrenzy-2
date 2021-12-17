package org.firstinspires.ftc.teamcode.TeleOp.TestTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;

@TeleOp(name="ZeroPowerDT")
public class ZeroPowerTest extends LinearOpMode {

    MainBase base = null;

    public boolean GP1_RB_Held   = false;
    public boolean GP2_LB_Held   = false;
    public boolean GP2_RB_Held   = false;
    public boolean GP2_Y_Held    = false;
    public boolean SlowMode      = false;
    public boolean AUTO_LIFT     = false;
    public double  LCLAW_OPEN    = .6;
    public double  LCLAW_CLOSED  = 0; //Delux hitec 485HB
    public double  RCLAW_OPEN    = 0.25;
    public double  RCLAW_CLOSED  = 0.75;
    public double  BUCKET_OPEN   = 0.75;
    public double  BUCKET_CLOSED = 0.3;
    public double  DUCK_SPEED = -0.42;
    int level = 0;


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