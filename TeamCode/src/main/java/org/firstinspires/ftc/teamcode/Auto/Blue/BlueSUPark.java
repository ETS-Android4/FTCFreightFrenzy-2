package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;


@Autonomous(name= "BLUE SU PARK")
public class BlueSUPark extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, false);

        base.init(hardwareMap);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        //Blue autonomous: Delivers Duck and Parks in Storage Unit
        //Position: Back facing Carousel
        //base.gyroTurn(.5, -30, this);
        base.encoderDrive(.5,-23,-23,this); // drive to Carousel
        base.leftDuck.setPower(-.42); // spin it
        sleep(2500); // for 2.5 sec.
        base.encoderDrive(.5,var.CLEAR_WALL, var.CLEAR_WALL, this); //clear carousel y
        base.gyroTurn(.5,-60, this); // rotate to face away from starting wall
        base.encoderDrive(.5,6,6,this); // clear carousel x
        base.gyroTurn(.5,-90, this); // rotate to face SU wall
        base.encoderDrive(.5,3,3,this); // drive towards SU wall
        base.gyroTurn(.5,90, this); // rotate to face SU
        base.encoderDrive(.5,8,8,this); // drive into SU

    }
}
