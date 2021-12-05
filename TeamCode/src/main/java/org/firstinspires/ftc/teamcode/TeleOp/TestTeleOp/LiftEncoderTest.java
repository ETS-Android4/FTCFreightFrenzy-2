package org.firstinspires.ftc.teamcode.TeleOp.TestTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;

@TeleOp(name = "LiftEncoderTest")
public class LiftEncoderTest extends LinearOpMode{

    MainBase base = null;
    public boolean AUTO_LIFT = false;

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
        if(gamepad2.a){
            base.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(gamepad2.a){
            base.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        double liftArm = -gamepad2.right_stick_y;
        if(Math.abs(liftArm) < 0.1){
            int level = 3;
            if(gamepad2.dpad_up){
                AUTO_LIFT = true;
                level = 3;
            }
            else if(gamepad2.dpad_left || gamepad2.dpad_right){
                AUTO_LIFT = true;
                level = 2;
            }
            else if(gamepad2.dpad_down){
                AUTO_LIFT = true;
                level = 1;
            }
            if(AUTO_LIFT){
                base.lift(level,this);
            }
            else{
                base.lift.setPower(0);
            }
        }
        else{
            base.lift.setPower(liftArm);
            AUTO_LIFT = false;
        }

        telemetry.addData("Encoder Counts: ", base.lift.getCurrentPosition());
        telemetry.update();

    }
}
