package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

@Autonomous(name= "BLUE-SU DELIVER")
public class BlueSUDeliver extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, false);

        base.init(hardwareMap, this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        base.gyro.resetZAxisIntegrator();

        //ObjectDetector.POSITIONS position = detector.getDecision();
        ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.RIGHT;
        //Blue autonomous: Delivers Duck and Parks in WH/
        //Position: Back facing Carousel (Back 10 degrees from wall)

        base.bucket.setPosition(0.90);
        base.leftClaw.setPosition(1.0);
        base.encoderDrive(0.8,-19.4,-19.4,this); // drive to Carousel
        base.leftDuck.setPower(0.53); //Spin it
        sleep(2000); // SLEEP IF ALLIANCE REQUESTS
        base.leftDuck.setPower(0);
        base.gyroTurn(0.5,10,this); //Rotate to face WH
        base.encoderDrive(0.7,45,45,this); //Drives halfway to WH
        //base.gyroDrive(0.8,45,45,0,0,0,this); //To test gyroDrive
        base.gyroTurn(0.5,100,this); //Turns to face hub

        switch (position) {
            case LEFT: //SCORES IN FIRST (BOTTOM) TIER
                base.liftAuto(1, false,this);
                base.encoderDrive(0.5,11,11,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(500);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(500);
                break;
            case MIDDLE: //SCORES IN SECOND (MIDDLE) TIER
                base.liftAuto(2, false,this);
                base.encoderDrive(0.5,11.2,11.2,this);
                //base.encoderDrive(0.3,1.7,1.7,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(500);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(500);
                break;
            case RIGHT: //SCORES IN THIRD (TOP) TIER
                base.encoderDrive(0.5,13.5,13.5,this); //Approaches hub head-on
                //base.rangeDrive(0.3,40,-1,this); //To test rangeDrive
                base.liftAuto(3, false,this); //Extends lift to top-tier
                base.encoderDrive(0.5,1.7,1.7,this); //Positioning for scoring
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(500);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(500);
                break;
        }

        base.encoderDrive(0.5,-10,-10,this);
        base.bucket.setPosition(var.BUCKET_OPEN);
        base.leftClaw.setPosition(var.LCLAW_OPEN); // set and close bucket
        base.liftAuto(0,false, this); //Bring lift down
        base.gyroTurn(0.5,30,this); //Turns diagonally towards WH
        base.encoderDrive(0.9,60,60,this); //Enters WH
        base.gyroTurn(0.5,10,this); //Turns perpendicular to back wall
        base.encoderDrive(0.8,20,20,this); //Drives to top-right of WH
        telemetry.addData("Parked in Blue WH:)","");
        base.lift.setPower(0); //Remove and test

        //telemetry.update();

    }
}
