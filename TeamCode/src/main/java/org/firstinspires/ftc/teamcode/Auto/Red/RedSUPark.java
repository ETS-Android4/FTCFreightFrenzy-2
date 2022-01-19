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

        base.encoderDrive(0.7,-19.4,-19.4,this); //Drives backwards to carousel
        base.rightDuck.setPower(var.DUCK_SPEED); //Spins carousel
        sleep(2300);
        base.rightDuck.setPower(0);
        base.gyroTurn(var.DT_HALF_SPEED,-110,this); //rotate front towards SU
        base.encoderDrive(var.DT_HALF_SPEED,21.5,21.5,this); //Drive into SU
        base.gyroTurn(var.DT_HALF_SPEED,-99,this);
        telemetry.addLine("Parked in RED-SU");
        telemetry.update();

    }
}
