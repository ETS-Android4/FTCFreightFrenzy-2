package org.firstinspires.ftc.teamcode.TeleOp.TestTeleOp.Components;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;

@Disabled
@TeleOp(name = "Intake")
public class Intake extends LinearOpMode{

    MainBase base = null;

    public boolean GP2_LB_Held   = false;
    public boolean GP2_RB_Held   = false;
    public boolean GP2_Y_Held    = false;
    public double  LCLAW_OPEN    = 0.75;
    public double  LCLAW_CLOSED  = 0.1; //Delux hitec 485HB
    public double  RCLAW_OPEN    = 0;
    public double  RCLAW_CLOSED  = 0.6;
    public double  BUCKET_OPEN   = 0.5;
    public double  BUCKET_CLOSED = 0.3;

    @Override
    public void runOpMode(){
        custom_init();
        waitForStart();
        while(opModeIsActive()){
            custom_loop();
        }
    }

    public void custom_init(){
        base = new MainBase();
        base.init(hardwareMap);

        telemetry.addData("Initialization Complete!","");
        telemetry.update();
    }

    public void custom_loop(){

        //---------------LEFT-CLAW---------------\\
        if (gamepad2.left_bumper && !GP2_LB_Held) {
            GP2_LB_Held = true;
            if (base.leftClaw.getPosition() == LCLAW_CLOSED) {
                base.leftClaw.setPosition(LCLAW_OPEN);
            } else {
                base.leftClaw.setPosition(LCLAW_CLOSED);
            }
        }
        if (!gamepad2.left_bumper) {
            GP2_LB_Held = false;
        }


        //---------------RIGHT-CLAW---------------\\
        if (gamepad2.right_bumper && !GP2_RB_Held) {
            GP2_RB_Held = true;
            if (base.rightClaw.getPosition() == RCLAW_CLOSED) {
                base.rightClaw.setPosition(RCLAW_OPEN);
            } else {
                base.rightClaw.setPosition(RCLAW_CLOSED);
            }
        }
        if (!gamepad2.right_bumper) {
            GP2_RB_Held = false;
        }


        //---------------BUCKET---------------\\
        if (gamepad2.y && !GP2_Y_Held) {
            if (base.bucket.getPosition() == BUCKET_CLOSED) {
                base.bucket.setPosition(BUCKET_OPEN);
            } else {
                base.bucket.setPosition(BUCKET_CLOSED);
            }
        }
        if (!gamepad2.y) {
            GP2_Y_Held = false;
        }

    }
}
