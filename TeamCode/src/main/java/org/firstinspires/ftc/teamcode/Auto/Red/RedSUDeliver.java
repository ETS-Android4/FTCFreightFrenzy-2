package org.firstinspires.ftc.teamcode.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

//Red autonomous: Delivers duck, delivers pre-loaded, and parks in WH
//Position: Back facing Carousel (Back 10 degrees from wall.)

@Autonomous(name= "RED-SU DELIVERY")
public class RedSUDeliver extends LinearOpMode{

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


        //Resets bucket & claw to avoid lift collision
        base.bucket.setPosition(0.90);
        base.leftClaw.setPosition(1.0);

        //Scores duck at carousel
        base.encoderDrive(0.8,-19.4,-19.4,this); //Drives backwards to carousel
        base.leftDuck.setPower(0.53); //Spins duck-wheel for duck soring
        sleep(2000); //Sleeps to allow for adequate spin time
        base.leftDuck.setPower(0); //Stops duck-wheel

        //Repositioning to score pre-loaded element just before approaching hub
        base.gyroTurn(0.5,-10,this); //Rotate to face WH
        base.encoderDrive(0.7,45,45,this); //Drives halfway to WH
        //base.gyroDrive(0.8,45,45,0,0,0,this); //To test gyroDrive
        base.gyroTurn(0.5,-100,this); //Turns to face shipping hub

        switch (position) { //hub level test result goes there <==
            case LEFT: //lvl. 1 and open bucket
                base.encoderDrive(0.5,10,10,this);
                base.liftAuto(1, this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(500);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(500);
                break;
            case MIDDLE: //lvl. 2 and open bucket
                base.encoderDrive(0.5,11.2,11.2,this);
                base.liftAuto(2, this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(500);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(500);
                break;
            case RIGHT: //lvl. 3 and  open bucket
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
        base.liftAuto(0,false, this);

        //PARKING
        base.gyroTurn(0.5,-35,this); //Turns diagonally towards WH
        base.encoderDrive(0.9,68,68,this); //Enters WH
        base.gyroTurn(0.5,-20,this); //Turns perpendicular to back wall
        base.encoderDrive(0.8,15,15,this); //Drives to top-right of WH [PARKED]
    }
}