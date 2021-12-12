package org.firstinspires.ftc.teamcode.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;


@Autonomous(name= "RED SU Park")
public class RedSUPark extends LinearOpMode{

    MainBase base = new MainBase();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, false);

        base.init(hardwareMap);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        //Red autonomous: Delivers Duck and Parks in Storage Unit
        //23b,Duck,6f,90l,6f,90l,3f,90r,8f
        base.encoderDrive(.5,-23,-23,this);
        base.rightDuck.setPower(.42);
        sleep(2500);
        base.encoderDrive(.5,6,6,this);
        base.gyroTurn(.5,-90, this);
        base.encoderDrive(.5,6,6,this);
        base.gyroTurn(.5,-90, this);
        base.encoderDrive(.5,3,3,this);
        base.gyroTurn(.5,90, this);
        base.encoderDrive(.5,8,8,this);





    }
}
