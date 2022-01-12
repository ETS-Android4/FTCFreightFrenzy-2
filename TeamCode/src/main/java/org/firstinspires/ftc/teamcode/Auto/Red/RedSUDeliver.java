package org.firstinspires.ftc.teamcode.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;


@Autonomous(name= "RED-SU DELIVERY")
public class RedSUDeliver extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, true);

        base.init(hardwareMap, this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        base.gyro.resetZAxisIntegrator();

        ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.LEFT;

        //Blue autonomous: Delivers Duck and Parks in Storage Unit
        //Position: Back facing Carousel (Back 10 degrees from wall.)
        //-13 in, 50r, 32 in, 140r, 8 in
        //base.gyroTurn(.5, -30, this);

        base.encoderDrive(0.7,-19,-19,this); // drive to Carousel
        base.rightDuck.setPower(.42); // spin it
        sleep(2500); // for 2.5 sec.
        base.rightDuck.setPower(0);
        base.gyroTurn(.5,-40,this); //rotate front towards SU
        base.encoderDrive(0.7,33.4,33.4,this);// drive into SU
        /*base.gyroTurn(.5,99,this);
        base.encoderDrive(.5,11,11,this);// drive half-past SU
        base.gyroTurn(.5,90,this);
        base.encoderDrive(.5,19,19,this);*/
        switch (position) { //hub level test result goes there <==
            case LEFT: //lvl. 1 and open bucket
                base.liftAuto(1, this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(1000);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(3000);
                break;
            case MIDDLE: //lvl. 2 and open bucket
                base.liftAuto(2, this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.5, 1.5, 1.5, this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(3000);
                base.encoderDrive(.5, -1.5, -1.5, this);
                break;
            case RIGHT: //lvl. 3 and  open bucket
                base.liftAuto(3, this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.5, 2, 2, this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(3000);
                base.encoderDrive(.5, -2, -2, this);
                break;
            default: //just in case
                base.liftAuto(3, this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.5, 2, 2, this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(3000);
                break;
        }
        base.bucket.setPosition(var.BUCKET_OPEN);
        base.leftClaw.setPosition(var.LCLAW_OPEN); // set and close bucket
        base.liftAuto(1,this); //Bring lift down
        base.encoderDrive(.5,-5,-5,this);
        base.gyroTurn(0.5,1,this);
        base.encoderDrive(0.8,20,20,this);
        base.gyroTurn(0.7,-30,this);
        base.encoderDrive(1.0,30,30,this);
        base.gyroTurn(0.5,1,this);
        base.encoderDrive(0.7,40,40,this);
        telemetry.addData("Parked in RED WH:)","");
        telemetry.update();

    }
}