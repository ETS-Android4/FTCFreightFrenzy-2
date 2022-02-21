package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

//Blue Autonomous: Delivers Duck and Parks in WH/
//Starting Position: Back facing Carousel (10 degrees from wall)

@Autonomous(name= "BLUE-SU 6168")
public class
BlueSUDeliver6168 extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, false,false);

        base.init(hardwareMap, this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("God Speed");
        telemetry.update();

        waitForStart();

        base.gyro.resetZAxisIntegrator();

        ObjectDetector.POSITIONS position = detector.getDecision();
        //ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.RIGHT;


        //Resets bucket & claw to avoid lift collision
        base.bucket.setPosition(0.90);
        base.leftClaw.setPosition(1.0);

        //Scores duck at carousel
        base.encoderDrive(0.8,-19.4,-19.4,this); //Drives backwards to carousel
        base.leftDuck.setPower(0.53); //Spins duck-wheel for duck soring
        sleep(2000); //Sleeps to allow for adequate spin time
        base.leftDuck.setPower(0); //Stops duck-wheel
        sleep(10100); //Perfect sleep for 6168

        //Repositioning to score pre-loaded element just before approaching hub
        base.gyroTurn(0.5,10,this); //Rotate to face WH
        base.encoderDrive(0.7,45,45,this); //Drives halfway to WH
        base.gyroTurn(0.5,100,this); //Turns to face shipping hub

        switch (position) {
            case LEFT: //SCORES IN FIRST (BOTTOM) TIER
                base.liftAuto(1, false,this);
                base.encoderDrive(0.5,10.9,10.9,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(400);
                base.encoderDrive(0.2,2.5,2.5,this); //Must test power
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(400);

                //Drives backward from shipping hub to prepare for WH parking
                base.encoderDrive(0.5,-3.2,-3.2,this);

                //Closes bucket & claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Repositions lift to ground-level position
                base.liftAuto(0,false,this);

                //Positioning before parking
                base.gyroTurn(0.5,20,this); //Turns diagonally towards WH
                //sleep(5000); //Untested sleep for 5893

                //PARKING
                base.encoderDrive(1.0,65.5,65.5,this); //Enters WH
                break;
            case MIDDLE: //SCORES IN SECOND (MIDDLE) TIER
                base.liftAuto(2, false,this);
                base.encoderDrive(0.5,12,12,this);
                sleep(800);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(400);
                base.encoderDrive(0.3,2.8,2.8,this); //2.5 check test
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(500);

                //Drives backward from shipping hub to prepare for WH parking
                base.encoderDrive(0.5,-1.9,-1.9,this);

                //Closes bucket & claw
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                base.bucket.setPosition(var.BUCKET_OPEN);

                //Repositions lift to ground-level position
                base.liftAuto(0,false,this);

                //PARKING
                base.gyroTurn(0.5,16,this); //Turns diagonally towards WH
                base.encoderDrive(1.0,65,65,this); //Enters WH
                base.bucket.setPosition(var.BUCKET_OPEN);
                break;
            case RIGHT: //SCORES IN THIRD (TOP) TIER
                base.liftAuto(3, false,this); //Extends lift to top-tier
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                base.encoderDrive(0.5,16.5,16.5,this); //Approaches hub head-on
                sleep(1600);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(500);

                //Drives backward from shipping hub to prepare for WH parking
                base.encoderDrive(0.5,-2.0,-2.0,this);

                //Closes bucket & claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Repositions lift to ground-level position
                base.liftAuto(0,false,this);

                //PARKING
                base.gyroTurn(0.5,17,this); //Turns diagonally towards WH
                base.encoderDrive(1.0,65,65,this); //Enters WH for 6168
                break;
        }
    }
}
