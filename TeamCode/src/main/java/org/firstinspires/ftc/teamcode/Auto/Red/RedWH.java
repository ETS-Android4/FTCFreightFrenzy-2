package org.firstinspires.ftc.teamcode.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

@Disabled
@Autonomous(name="RED WH")
public class RedWH extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        //ObjectDetector detector = new ObjectDetector(this, false);


        base.init(hardwareMap);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        base.gyro.resetZAxisIntegrator();

        int position = 0;

        //---------------- CASE LEFT ----------------
        /*if (position == 0){
            //Robot movements for "Case Left" position of team marker.

        }

        //---------------- CASE MIDDLE ----------------
        else if(position == 1){
            //Robot movements for "Case Middle" position of team marker.

        }

        //---------------- CASE RIGHT ----------------
        else if(position == 3){
            //Robot movements for "Case Right" position of team marker.

        }*/

        //Autonomous: Delivers Pre-loaded Block and Parks in Warehouse
        //Position: facing forward

        base.encoderDrive(.5, var.CLEAR_WALL, var.CLEAR_WALL,this); //clear wall
        base.gyroTurn(.5,-10,this); //face hub
        base.encoderDrive(.5,7,7,this); //drive to hub
        switch (position) { //hub level test result goes there <==
            case 0: //lvl. 1 and open bucket
                base.lift(1,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                break;
            case 1: //lvl. 2 and open bucket
                base.lift(2,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                break;
            case 2: //lvl. 3 and  open bucket
                base.lift(3,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                break;
            default: //just in case
                base.lift(3,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                break;
        }
        base.gyroTurn(.5,100,this); // turn towards warehouse
        base.encoderDrive(.5,60,60,this); //drive into warehouse
        base.lift(1,this); base.bucket.setPosition(var.BUCKET_CLOSED); base.leftClaw.setPosition(var.LCLAW_CLOSED); // set and close bucket
    }
}
