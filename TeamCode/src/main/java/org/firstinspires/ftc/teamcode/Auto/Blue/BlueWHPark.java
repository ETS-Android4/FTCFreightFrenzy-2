package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;


@Autonomous(name= "BLUE WH Park")
public class BlueWHPark extends LinearOpMode{

    MainBase base = new MainBase();

    @Override
    public void runOpMode() throws InterruptedException {

        base.init(hardwareMap, this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        //Blue autonomous: Parks in Warehouse
        //Robot Position: Robot facing WH and 40 degrees from wall
        base.encoderDrive(0.6,50,50,this); //Drives forward into WH
        base.gyroTurn(0.5,-20,this);
        base.encoderDrive(0.5,5,5,this);
    }
}
