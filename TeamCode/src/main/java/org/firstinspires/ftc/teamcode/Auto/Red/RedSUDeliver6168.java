package org.firstinspires.ftc.teamcode.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

//Red autonomous: Delivers duck, delivers pre-loaded, and parks in WH (Top-Left)
//Position: Back facing Carousel (Back 10 degrees from wall).

@Autonomous(name= "RED-SU 6168")
public class RedSUDeliver6168 extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this,false,false);

        base.init(hardwareMap, this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        base.gyro.resetZAxisIntegrator();

        ObjectDetector.POSITIONS position = detector.getDecision();
        //ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.RIGHT;

        //Resets bucket & claw to avoid lift collision
        base.bucket.setPosition(0.90);
        base.leftClaw.setPosition(1.0);

        //Scores duck at carousel
        base.encoderDrive(0.8,-18.9,-18.9,this); //Drives backwards to carousel
        base.rightDuck.setPower(0.53); //Spins duck-wheel for duck soring
        sleep(2000); //Sleeps to allow for adequate spin time
        base.rightDuck.setPower(0); //Stops duck-wheel
        sleep(10200);

        //Repositioning to score pre-loaded element just before approaching hub
        base.gyroTurn(0.5,-10,this); //Rotate to face WH
        base.encoderDrive(0.7,45,45,this); //Drives halfway to WH
        base.gyroTurn(0.5,-99,this); //Turns to face shipping hub

        switch (position) {
            case LEFT: //SCORES IN FIRST (BOTTOM) TIER
                base.liftAutoRED(1,false,this);
                base.encoderDrive(0.5,8,8,this);
                sleep(400);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(400);
                base.encoderDrive(0.5,2.7,2.7,this);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(400);

                //Drives backward from shipping hub to prepare for WH parking
                base.encoderDrive(0.8,-6,-6,this);
                sleep(600);

                //Closes bucket & claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Repositions lift to ground-level position
                base.liftAuto(0,false,this);

                //Positioning before parking
                base.gyroTurn(0.5,-14,this); //Turns diagonally towards WH
                //sleep(5000); //Untested sleep for 5893

                //PARKING
                base.encoderDrive(1.0,66,66,this); //Enters WH
                break;
            case MIDDLE: //SCORES IN SECOND (MIDDLE) TIER
                base.liftAuto(2,false,this);
                base.encoderDrive(0.5,10,10,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(500);
                base.encoderDrive(0.3,3.5,3.5,this);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(450);

                //Drives backward from shipping hub to prepare for WH parking
                base.encoderDrive(0.5,-3.0,-3.0,this);

                //Closes bucket & claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Repositions lift to ground-level position
                base.liftAuto(0,false,this);

                //PARKING
                base.gyroTurn(0.5,-12,this); //Turns diagonally towards WH
                base.encoderDrive(1.0,64,64,this); //Enters WH
                break;
            case RIGHT: //SCORES IN THIRD (TOP) TIER
                base.liftAuto(3,false,this); //Extends lift to top-tier
                base.encoderDrive(0.3,15.3,15.3,this); //Approaches hub head-on
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(200);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(400);

                //Drives backward from shipping hub to prepare for WH parking
                base.encoderDrive(0.8,-5.0,-5.0,this);

                //Closes bucket & claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Repositions lift to ground-level position
                base.liftAuto(0,false,this);

                //PARKING
                base.gyroTurn(0.5,-9,this); //Turns diagonally towards WH
                base.encoderDrive(1.0,64.5,64.5,this); //Enters WH
                break;
        }
    }
}