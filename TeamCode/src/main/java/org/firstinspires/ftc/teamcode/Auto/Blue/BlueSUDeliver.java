package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

//Blue Autonomous: Delivers Duck and Parks in WH/
//Starting Position: Back facing Carousel (10 degrees from wall)

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
        ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.LEFT;


        //Resets bucket & claw to avoid lift collision
        base.bucket.setPosition(0.90);
        base.leftClaw.setPosition(1.0);

        //Scores duck at carousel
        base.encoderDrive(0.8,-19.4,-19.4,this); //Drives backwards to carousel
        base.leftDuck.setPower(0.53); //Spins duck-wheel for duck soring
        sleep(2000); //Sleeps to allow for adequate spin time
        base.leftDuck.setPower(0); //Stops duck-wheel

        //Repositioning to score pre-loaded element just before approaching hub
        base.gyroTurn(0.5,10,this); //Rotate to face WH
        base.encoderDrive(0.7,45,45,this); //Drives halfway to WH
        //base.gyroDrive(0.8,45,45,0,0,0,this); //To test gyroDrive
        base.gyroTurn(0.5,100,this); //Turns to face shipping hub

        switch (position) {
            case LEFT: //SCORES IN FIRST (BOTTOM) TIER
                base.liftAuto(1, false,this);
                base.encoderDrive(0.5,11.9,11.9,this);
                sleep(800);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(1000);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(500);
                break;
            case MIDDLE: //SCORES IN SECOND (MIDDLE) TIER
                base.liftAuto(2, false,this);
                base.encoderDrive(0.5,12.6,12.6,this);
                sleep(800);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(500);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(200);
                base.encoderDrive(0.3,1.7,1.7,this);
                break;
            case RIGHT: //SCORES IN THIRD (TOP) TIER
                base.encoderDrive(0.5,13.5,13.5,this); //Approaches hub head-on
                //base.rangeDrive(0.3,40,-1,this); //To test rangeDrive
                base.liftAuto(3, this); //Extends lift to top-tier
                base.encoderDrive(0.5,1.7,1.7,this); //Positioning for scoring
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(500);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(500);
                break;
        }

        //Drives backward from shipping hub to prepare for WH parking
        base.encoderDrive(0.5,-7,-7,this);

        //Closes bucket & claw
        base.bucket.setPosition(var.BUCKET_OPEN);
        base.leftClaw.setPosition(var.LCLAW_OPEN);

        //Repositions lift to ground-level position
        base.liftAuto(0,this);

        //PARKING
        /*base.gyroTurn(0.5,34,this); //Turns diagonally towards WH
        base.encoderDrive(0.9,66,66,this); //Enters WH
        base.gyroTurn(0.5,20,this); //Turns perpendicular to back wall
        base.encoderDrive(0.8,15,15,this); //Drives to top-right of WH [PARKED]*/

    }
}
