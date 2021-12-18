package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;


@Autonomous(name= "BLUE SU PARK")
public class BlueSUDeliver extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, false);

        base.init(hardwareMap);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        waitForStart();
        base.gyro.resetZAxisIntegrator();
        int position = 0;

        //Blue autonomous: Delivers Duck and Parks in Storage Unit
        //Position: Back facing Carousel (Back 10 degrees from wall.)
        //-13 in, 50r, 32 in, 140r, 8 in
        //base.gyroTurn(.5, -30, this);

        base.encoderDrive(0.5,-18,-18,this); // drive to Carousel
        base.leftDuck.setPower(-.42); // spin it
        sleep(2500); // for 2.5 sec.
        base.gyroTurn(.5,90,this); //rotate front towards hub
        base.encoderDrive(.5,24,24,this);// drive half-past SU
        base.gyroTurn(.5,-90,this);
        base.encoderDrive(.5,19,19,this);
        switch (position) { //hub level test result goes there <==
            case 0: //lvl. 1 and open bucket
                base.liftAuto(1, this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                break;
            case 1: //lvl. 2 and open bucket
                base.liftAuto(2, this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.5, 1.5, 1.5, this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                base.encoderDrive(.5, -1.5, -1.5, this);
                break;
            case 2: //lvl. 3 and  open bucket
                base.liftAuto(3, this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.5, 2, 2, this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                base.encoderDrive(.5, -2, -2, this);
                break;
            default: //just in case
                base.liftAuto(3, this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.5, 2, 2, this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                break;
        }
        base.encoderDrive(.5,-37,-37,this);
        base.gyroTurn(.5,-90,this);
        base.encoderDrive(.5,-11,-11,this);
        telemetry.addData("Parked in Blue SU :)","");
        telemetry.update();

    }
}
