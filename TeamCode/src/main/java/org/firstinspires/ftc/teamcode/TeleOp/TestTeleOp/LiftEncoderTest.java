package org.firstinspires.ftc.teamcode.TeleOp.TestTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;

@TeleOp(name = "LiftEncoderTest")
public class LiftEncoderTest extends LinearOpMode{

    MainBase base = null;

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

        telemetry.addData("Encoder Counts: ", base.lift.getCurrentPosition());
        telemetry.update();
    }
}
