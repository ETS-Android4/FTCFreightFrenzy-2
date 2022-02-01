package org.firstinspires.ftc.teamcode.Auto.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

//Autonomous: Delivers Pre-loaded Block, scores additional element, parks in WH (Top-Right)
//Position: Facing forward. Right tread lined against inside of floor-tile. Both treads touching back wall.

@Autonomous(name="BLUE-WH DELIVERY & SCORE")
public class BlueWHDeliverScoring extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, true,false);

        base.init(hardwareMap, this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("God Speed");
        telemetry.update();
        
        detector.setTelemShow(true);

        waitForStart();

        base.gyro.resetZAxisIntegrator();

        //ObjectDetector.POSITIONS position = detector.getDecision();
        ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.LEFT;

        //Resets bucket & claw to avoid lift collision
        base.bucket.setPosition(var.BUCKET_OPEN);
        base.leftClaw.setPosition(1.0);

        switch (position) {
            case LEFT: //SCORES IN FIRST (BOTTOM) TIER

                //Positioning prior to scoring
                base.encoderDrive(1.0, 5, 15,this); //Clears back wall & faces carousel
                base.liftAuto(1,false,this);
                base.encoderDrive(1.0,7,7,this);

                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(500);
                base.encoderDrive(0.8,8,8,this);

                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(600);

                //ANYTHING BEYOND THIS POINT IS UNTESTED CODE!!!

                //Drives backwards (away) from hub
                base.encoderDrive(1.0,-10,-20,this);

                //CLOSES bucket and claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Brings down lift while parking
                base.liftAuto(0,false,this);

                //Placement before WH PARKING
                base.gyroTurn(0.5,-87,this);
                base.encoderDrive(1.0,50,50,this); //Drives towards SHARED HUB
                base.gyroTurn(0.5,-95,this);

                //OPENS bucket and claw
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);

                base.encoderDrive(0.3,10,10,this);

                //CLOSES bucket and claw
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(300);
                base.bucket.setPosition(var.BUCKET_OPEN);

                base.gyroTurn(0.7,87,this);
                base.encoderDrive(1.0,30,30,this); //Drives towards HUB
                base.gyroTurn(0.5,30,this);

                base.liftAuto(1,false,this);
                base.encoderDrive(0.6,10,10,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(400);
                base.encoderDrive(0.5,5,5,this);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(600);

                //Drives backwards (away) from hub
                base.encoderDrive(1.0,-14,-14,this);

                //CLOSES bucket and claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Brings down lift while parking
                base.liftAuto(0,false,this);

                //Placement before WH PARKING
                base.gyroTurn(0.7,-69,this); //Turns towards SHARED HUB
                base.encoderDrive(1.0,50,50,this); //Drives towards SHARED HUB
                base.gyroTurn(0.7,-55,this);
                base.encoderDrive(0.7,10,10,this);
                base.gyroTurn(0.7,-87,this);
                break;
            case MIDDLE: //SCORES IN SECOND (MIDDLE) TIER

                //Positioning prior to scoring
                base.encoderDrive(1.0, 5, 15,this); //Clears back wall & faces carousel
                base.liftAuto(2,false,this);
                base.encoderDrive(1.0,7,7,this);

                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(500);
                base.encoderDrive(0.8,8,8,this);

                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(600);

                //ANYTHING BEYOND THIS POINT IS UNTESTED CODE!!!

                //Drives backwards (away) from hub
                base.encoderDrive(1.0,-10,-20,this);

                //CLOSES bucket and claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Brings down lift while parking
                base.liftAuto(0,false,this);

                //Placement before WH PARKING
                base.gyroTurn(0.5,-87,this);
                base.encoderDrive(1.0,50,50,this); //Drives towards SHARED HUB
                base.gyroTurn(0.5,-95,this);

                //OPENS bucket and claw
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);

                base.encoderDrive(0.3,10,10,this);

                //CLOSES bucket and claw
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(300);
                base.bucket.setPosition(var.BUCKET_OPEN);

                base.gyroTurn(0.7,87,this);
                base.encoderDrive(1.0,30,30,this); //Drives towards HUB
                base.gyroTurn(0.5,30,this);

                base.liftAuto(1,false,this);
                base.encoderDrive(0.6,10,10,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(400);
                base.encoderDrive(0.5,5,5,this);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(600);

                //Drives backwards (away) from hub
                base.encoderDrive(1.0,-14,-14,this);

                //CLOSES bucket and claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Brings down lift while parking
                base.liftAuto(0,false,this);

                //Placement before WH PARKING
                base.gyroTurn(0.7,-69,this); //Turns towards SHARED HUB
                base.encoderDrive(1.0,50,50,this); //Drives towards SHARED HUB
                base.gyroTurn(0.7,-55,this);
                base.encoderDrive(0.7,10,10,this);
                base.gyroTurn(0.7,-87,this);
                break;
            case RIGHT: //SCORES IN THIRD (TOP) TIER

                //REQUIRES REWRITING (Possible task for Immanuel or Claire)

                break;
        }
    }
}
