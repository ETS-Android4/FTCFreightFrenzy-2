package org.firstinspires.ftc.teamcode.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;


@Autonomous(name= "RED SU Park")
public class RedSUPark extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, false);

        base.init(hardwareMap,this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        //Red autonomous: Delivers Duck and Parks in Storage Unit
        //23b,Duck,6f,90l,6f,90l,3f,90r,8f
        base.encoderDrive(0.5,-19,-19,this); // drive to Carousel
        base.rightDuck.setPower(.42); // spin it
        sleep(2500); // for 2.5 sec.
        base.rightDuck.setPower(0);
        base.gyroTurn(.5,110,this); //rotate front towards SU
        base.encoderDrive(.5,21.5,21.5,this);// drive into SU
        base.gyroTurn(.5,99,this);
        telemetry.addData("Parked in Red SU :)","");
        telemetry.update();

    }
}
