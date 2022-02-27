package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

//Blue Autonomous: Scores duck, delivers pre-loaded element, and parks in SU
//Starting Position: Back facing Carousel (10 degrees from wall)
//REQUIRES TESTING!!!

@Autonomous(name= "BLUE-SU Delivery SU-Park")
public class BlueSUDeliverSUParking extends LinearOpMode{

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


        waitForStart();

        base.gyro.resetZAxisIntegrator();

        ObjectDetector.POSITIONS position = detector.getDecision();
        //ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.RIGHT; //Running case RIGHT for testing purposes.


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
        base.gyroTurn(0.5,100,this); //Turns to face shipping hub

        switch (position) { //hub level test result goes there <==
            case LEFT: //lvl. 1 and open bucket
                base.liftAuto(1, false,this);
                base.encoderDrive(0.5,10.9,10.9,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(400);
                base.encoderDrive(0.2,2.5,2.5,this); //Must test power
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(400);

                //Drives backward from shipping hub to prepare for WH parking
                base.encoderDrive(0.5, -5, -5, this);

                //Closes bucket & claw
                base.bucket.setPosition(0.98);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Repositions lift to ground-level position
                base.liftAuto(0, false, this);

                //PARKING
                base.gyroTurn(0.5, 12, this); //Turns to face SU
                base.encoderDrive(1.0, -50, -50, this); //Enters SU
                base.gyroTurn(0.5, 100, this);
                base.encoderDrive(0.5, 14, 14, this);
                break;
            case MIDDLE: //lvl. 2 and open bucket
                base.liftAuto(2, false,this);
                base.encoderDrive(0.5,12,12,this);
                sleep(800);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(400);
                base.encoderDrive(0.3,2.3,2.3,this);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(500);

                //Drives backward from shipping hub to prepare for WH parking
                base.encoderDrive(0.5, -10, -10, this);

                //Closes bucket & claw
                base.bucket.setPosition(0.98);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Repositions lift to ground-level position
                base.liftAuto(0, false, this);

                //PARKING
                base.gyroTurn(0.5, 12, this); //Turns to face SU
                base.encoderDrive(1.0, -50, -50, this); //Enters SU
                base.gyroTurn(0.5, 100, this);
                base.encoderDrive(0.5, 15.5, 15.5, this);
                break;
            case RIGHT: //lvl. 3 and  open bucket
                base.liftAuto(3, false,this); //Extends lift to top-tier
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                base.encoderDrive(0.5,16.5,16.5,this); //Approaches hub head-on
                sleep(1600);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(400);

                //Drives backward from shipping hub to prepare for WH parking
                base.encoderDrive(0.5, -10, -10, this);

                //Closes bucket & claw
                base.bucket.setPosition(0.98);

                //Repositions lift to ground-level position
                base.liftAuto(0, false, this);

                //PARKING
                base.gyroTurn(0.5, 12, this); //Turns to face SU
                base.encoderDrive(1.0, -50, -50, this); //Enters SU
                base.gyroTurn(0.5, 100, this);
                base.encoderDrive(0.5, 15, 15, this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                break;
        }
    }
}
