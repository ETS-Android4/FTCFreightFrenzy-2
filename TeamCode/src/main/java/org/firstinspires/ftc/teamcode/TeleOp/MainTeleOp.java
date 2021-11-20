package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Base.MainBase;

@TeleOp(name="MainTeleOp")
public class MainTeleOp extends LinearOpMode{

    MainBase base = null;

    public boolean   GP1_RB_Held = false;
    public boolean   GP2_LB_Held = false;
    public boolean   SlowMode    = false;
    public double    CLAW_OPEN   = 0.5;
    public double    CLAW_CLOSED = 0.1;
    public double    DUCK_SPEED  = -0.8;


    @Override
    public void runOpMode(){
        custom_init();
        waitForStart();
        while(opModeIsActive()){
            custom_loop();
        }
    }

    public void custom_init() {
        base = new MainBase();
        base.init(hardwareMap);

        telemetry.addData("Initialization Complete!","");
        telemetry.update();
    }

    public void custom_loop() {

        //--------------------DRIVE-TRAIN CONTROLS--------------------\\
        double forward = -gamepad1.left_stick_y;
        double right   =  gamepad1.left_stick_x;
        double turn    = gamepad1.right_stick_x;

        double leftPower = forward + right + turn;
        double rightPower = forward - right + turn;
        double[] powers = {leftPower, rightPower};

        boolean needToScale = false;
        for (double power : powers){
            if(Math.abs(power) > 1){
                needToScale = true;
                break;
            }
        }
        if (needToScale){
            double greatest = 0;
            for (double power : powers){
                if (Math.abs(power) > greatest){
                    greatest = Math.abs(power);
                }
            }
            leftPower /= greatest;
            rightPower /= greatest;
        }

        //--------------------SLOW-MODE--------------------\\
        if(gamepad1.right_bumper && !GP1_RB_Held){
            GP1_RB_Held = true;
            SlowMode    =  !SlowMode;
        }
        if(!gamepad1.right_bumper){
            GP1_RB_Held = false;
        }

        if(SlowMode) {
            base.leftDT.setPower(0.3 * leftPower);
            base.rightDT.setPower(0.3 * rightPower);
        }
        else{
            base.leftDT.setPower(leftPower);
            base.rightDT.setPower(rightPower);
        }


        //--------------------ROBOT CONTROLS--------------------\\

        //---------------LEFT-DUCK---------------\\
        if (gamepad2.x){
            base.leftDuck.setPower(DUCK_SPEED);
        }
        else{
            base.leftDuck.setPower(0);
        }

        //---------------RIGHT-DUCK---------------\\
        if (gamepad2.b){
            base.rightDuck.setPower(DUCK_SPEED);
        }
        else{
            base.rightDuck.setPower(0);
        }

        //---------------Lift-System---------------\\
        double liftArm = gamepad2.right_stick_y;
        if(Math.abs(liftArm) < 0.1){
            base.lift.setPower(0);
        }
        else{
            base.lift.setPower(liftArm);
        }

        //---------------LEFT-CLAW---------------\\
        if(gamepad2.left_bumper && !GP2_LB_Held) {
            GP2_LB_Held = true;
            if(Math.abs(base.leftClaw.getPosition() - CLAW_CLOSED) < 0.01){
                base.leftClaw.setPosition(CLAW_OPEN);
            }
            else{
                base.leftClaw.setPosition(CLAW_CLOSED);
            }
        }
        if(!gamepad2.left_bumper) {
            GP2_LB_Held = false;
        }
        if((Math.abs(base.leftClaw.getPosition() - CLAW_CLOSED) < 0.01) /* && rings == 0*/){
            base.leftClaw.setPosition(CLAW_CLOSED);
        }

        //---------------RIGHT CLAW---------------\\
        if(gamepad2.right_bumper) {
            base.rightClaw.setPower(0.7);
            }
        else{
            base.rightClaw.setPower(0);
            }

        }
    }


