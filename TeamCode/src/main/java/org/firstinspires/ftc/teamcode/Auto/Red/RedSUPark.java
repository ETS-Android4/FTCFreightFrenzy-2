package org.firstinspires.ftc.teamcode.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

@Disabled
@Autonomous(name= "RED-SU PARKING")
public class RedSUPark extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        base.init(hardwareMap,this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        //Red autonomous: Delivers Duck and Parks in Storage Unit

        //Resets bucket & claw to avoid lift collision
        base.bucket.setPosition(0.90);
        base.leftClaw.setPosition(1.0);

        //Scores duck at carousel
        base.encoderDrive(0.8,-19.4,-19.4,this); //Drives backwards to carousel
        base.leftDuck.setPower(0.53); //Spins duck-wheel for duck soring
        sleep(2000); //Sleeps to allow for adequate spin time
        base.leftDuck.setPower(0); //Stops duck-wheel

        //SU PARKING
        base.gyroTurn(var.DT_HALF_SPEED,-110,this); //rotate front towards SU
        base.encoderDrive(var.DT_HALF_SPEED,21.5,21.5,this); //Drive into SU
        base.gyroTurn(var.DT_HALF_SPEED,-99,this);
    }
}
