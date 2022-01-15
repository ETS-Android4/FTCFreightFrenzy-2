package org.firstinspires.ftc.teamcode.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

@Autonomous(name="RED-WH DELIVERY")
public class RedWH extends LinearOpMode{

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

        //ObjectDetector.POSITIONS position = detector.getDecision();
        ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.LEFT;

        detector.setTelemShow(false);

        base.encoderDrive(.5, var.CLEAR_WALL, var.CLEAR_WALL,this); //clear wall
        base.gyroTurn(.5,-10,this); //face hub
        base.encoderDrive(.5,7,7,this); //drive to hub
        switch (position) { //hub level test result goes there <==
            case LEFT: //lvl. 1 and open bucket
                base.liftAuto(1,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                break;
            case MIDDLE: //lvl. 2 and open bucket
                base.liftAuto(2,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.5,1.5,1.5,this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                base.encoderDrive(.5,-1.5,-1.5,this);
                break;
            case RIGHT: //lvl. 3 and  open bucket
                base.liftAuto(3,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.5,2,2,this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                base.encoderDrive(.5,-2,-2,this);
                break;
            default: //just in case
                base.liftAuto(3,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.5,2,2,this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                break;
        }
        base.encoderDrive(.5,-8,-8,this); //back up
        base.bucket.setPosition(var.BUCKET_CLOSED);
        base.leftClaw.setPosition(var.LCLAW_CLOSED); // set and close bucket
        base.liftAuto(0,this); //Bring lift down
        sleep(1500);
        base.gyroTurn(.5,100,this); // turn towards warehouse
        base.encoderDrive(.5,61,60,this); //drive into warehouse
        base.leftClaw.setPosition(var.LCLAW_CLOSED); // set and close bucket
    }
}
