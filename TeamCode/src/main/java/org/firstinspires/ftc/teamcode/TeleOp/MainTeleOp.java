package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;

@TeleOp(name="MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    MainBase base = null;

    public boolean GP1_RB_Held   = false;
    public boolean GP2_LB_Held   = false;
    public boolean GP2_RB_Held   = false;
    public boolean GP2_Y_Held    = false;
    public boolean DriveChange   = false;
    public boolean GP1_A_Held    = false;
    public boolean SlowMode      = false;
    public boolean AUTO_LIFT     = false;
    public double  LCLAW_OPEN    = 1.0;
    public double  LCLAW_CLOSED  = 0; //Delux hitec 485HB
    public double  RCLAW_OPEN    = 0;
    public double  RCLAW_CLOSED  = 0.6;
    public double  BUCKET_OPEN   = 0.95;
    public double  BUCKET_CLOSED = 0.5;
    public double  DUCK_SPEED    = 0.42;
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
        base.init(hardwareMap, this);

        telemetry.addData("Initialization Complete!", "");
        telemetry.update();
    }

    public void custom_loop() {

        //--------------------DRIVE-TRAIN CONTROLS--------------------\\

        if(gamepad1.a && !GP1_A_Held){
            DriveChange = !DriveChange;
            GP1_A_Held = true;
        }
        if(!gamepad1.a){
            GP1_A_Held = false;
        }

        double leftPower  = 0;
        double rightPower = 0;
        //--------------------NORMAL--------------------\\
        if(!DriveChange){
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            leftPower  = forward - turn;
            rightPower = forward + turn;

        }

        //--------------------TANK--------------------\\
        else{
            double lForward = -gamepad1.left_stick_y;
            double rForward = -gamepad1.right_stick_y;

            leftPower  = lForward;
            rightPower = rForward;
        }

        double[] powers = {leftPower, rightPower};
        boolean needToScale = false;
        for (double power : powers) {
            if (Math.abs(power) > 1) {
                needToScale = true;
                break;
            }
        }
        if (needToScale) {
            double greatest = 0;
            for (double power : powers) {
                if (Math.abs(power) > greatest) {
                    greatest = Math.abs(power);
                }
            }
            leftPower /= greatest;
            rightPower /= greatest;
        }

        //--------------------SLOW-MODE--------------------\\
        if (gamepad1.right_bumper && !GP1_RB_Held) {
            GP1_RB_Held = true;
            SlowMode = !SlowMode;
        }
        if (!gamepad1.right_bumper) {
            GP1_RB_Held = false;
        }
        if (SlowMode) {
            base.leftDT.setPower(0.15 * leftPower);
            base.rightDT.setPower(0.15 * rightPower);
        } else {
            base.leftDT.setPower(leftPower);
            base.rightDT.setPower(rightPower);
        }


        //--------------------ROBOT CONTROLS--------------------\\

        //---------------LEFT-DUCK---------------\\
        if (gamepad2.x) {
            base.leftDuck.setPower(DUCK_SPEED);
        } else {
            base.leftDuck.setPower(0);
        }

        //---------------RIGHT-DUCK---------------\\
        if (gamepad2.b) {
            base.rightDuck.setPower(DUCK_SPEED);
        } else {
            base.rightDuck.setPower(0);
        }

        //---------------DUAL-DUCK---------------\\
        double duckSpin = gamepad2.left_stick_x / 2;
        if (duckSpin > 0.1) {
            base.rightDuck.setPower(duckSpin);
            base.leftDuck.setPower(-duckSpin);
        }
        else if (duckSpin < -0.1){
            base.rightDuck.setPower(-duckSpin);
            base.leftDuck.setPower(duckSpin);
        }
        else {
            base.rightDuck.setPower(0);
            base.leftDuck.setPower(0);
        }

        //---------------LIFT---------------\\
        double liftArm = -gamepad2.right_stick_y;
        if (Math.abs(liftArm) < 0.1) {
            if (gamepad2.dpad_up) {
                AUTO_LIFT = true;
                level = 3;
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                AUTO_LIFT = true;
                level = 2;
            } else if (gamepad2.dpad_down) {
                AUTO_LIFT = true;
                level = 1;
            } else if (gamepad2.right_bumper) {
                AUTO_LIFT = true;
                level = 4;
            }
            if (AUTO_LIFT) {
                base.lift(level, this);
            } else {
                base.lift.setPower(0);
            }
        } else {
            base.lift.setPower(liftArm);
            AUTO_LIFT = false;
        }

        telemetry.addData("Lift Encoders: ", base.lift.getCurrentPosition());

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

        //LEFT-CLAW Var Transfer Test
        /*if (gamepad2.left_bumper && !GP2_LB_Held) {
            GP2_LB_Held = true;
            if (base.leftClaw.getPosition() == LCLAW_OPEN) {
                base.leftClaw.setPosition(LCLAW_CLOSED);
            } else {
                base.leftClaw.setPosition(LCLAW_OPEN);
            }
        }*/

        //---------------RIGHT CLAW---------------\\
        /*if (gamepad2.right_bumper && !GP2_RB_Held) {
            GP2_RB_Held = true;
            if (base.rightClaw.getPosition() == RCLAW_CLOSED) {
                base.rightClaw.setPosition(RCLAW_OPEN);

                telemetry.addData("RIGHT CLAW OPEN","");
            } else {
                base.rightClaw.setPosition(RCLAW_CLOSED);

                telemetry.addData("RIGHT CLAW CLOSED","");
            }
        }

        if (!gamepad2.right_bumper) {
            GP2_RB_Held = false;
        }*/


        //---------------BUCKET---------------\\
        if (gamepad2.y && !GP2_Y_Held) {
            GP2_Y_Held = true;
            if (base.bucket.getPosition() == BUCKET_CLOSED) {
                base.bucket.setPosition(BUCKET_OPEN);
            } else {
                base.bucket.setPosition(BUCKET_CLOSED);
            }
        }
        if (!gamepad2.y) {
            GP2_Y_Held = false;
        }

        telemetry.addData("LeftDT Encoders: ", base.leftDT.getCurrentPosition());
        telemetry.addData("RightDT Encoders: ", base.rightDT.getCurrentPosition());
        telemetry.update();
    }
}