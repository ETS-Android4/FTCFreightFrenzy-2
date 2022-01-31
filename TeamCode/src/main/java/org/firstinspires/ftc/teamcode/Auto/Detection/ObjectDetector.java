package org.firstinspires.ftc.teamcode.Auto.Detection;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class ObjectDetector {

    OpMode opMode;
    OpenCvCamera camera;

    CustomPipeline pipeline;

    private final Point FRONT_LEFT_TL   = new Point(10,160);
    private final Point FRONT_LEFT_BR   = new Point(51, 200);
    private final Point FRONT_MIDDLE_TL = new Point(125, 160);
    private final Point FRONT_MIDDLE_BR = new Point(175,  200);
    private final Point FRONT_RIGHT_TL  = new Point(270, 160);
    private final Point FRONT_RIGHT_BR  = new Point(320, 200);

    private final Point R_FRONT_LEFT_TL   = new Point(110,180);
    private final Point R_FRONT_LEFT_BR   = new Point(160, 220);
    private final Point R_FRONT_MIDDLE_TL = new Point(260, 180);
    private final Point R_FRONT_MIDDLE_BR = new Point(310,  220);
    private final Point R_FRONT_RIGHT_TL  = new Point(10, 180);
    private final Point R_FRONT_RIGHT_BR  = new Point(51, 220);

    private final Point SWITCH_LEFT_TL    = new Point(10,160);
    private final Point SWITCH_LEFT_BR    = new Point(40, 200);
    private final Point SWITCH_MIDDLE_TL  = new Point(150, 160);
    private final Point SWITCH_MIDDLE_BR  = new Point(180,  200);
    private final Point SWITCH_RIGHT_TL   = new Point(290, 160);
    private final Point SWITCH_RIGHT_BR   = new Point(320, 200);

    private Point leftTL;
    private Point leftBR;
    private Point middleTL;
    private Point middleBR;
    private Point rightTL;
    private Point rightBR;

    private RGBColor left;
    private RGBColor middle;
    private RGBColor right;
    private boolean show_value = true;


    public ObjectDetector(OpMode op, boolean isFrontCAM, boolean isRed){

        opMode = op;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "sideCAM"), cameraMonitorViewId);

        pipeline = new CustomPipeline();
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        if (!isRed) {
            leftTL   = (isFrontCAM) ? FRONT_LEFT_TL : SWITCH_LEFT_TL;
            leftBR   = (isFrontCAM) ? FRONT_LEFT_BR : SWITCH_LEFT_BR;
            middleTL = (isFrontCAM) ? FRONT_MIDDLE_TL : SWITCH_MIDDLE_TL;
            middleBR = (isFrontCAM) ? FRONT_MIDDLE_BR : SWITCH_MIDDLE_BR;
            rightTL  = (isFrontCAM) ? FRONT_RIGHT_TL : SWITCH_RIGHT_TL;
            rightBR  = (isFrontCAM) ? FRONT_RIGHT_BR : SWITCH_RIGHT_BR;
        }
        else{
            leftTL   = (isFrontCAM) ? R_FRONT_LEFT_TL : SWITCH_LEFT_TL;
            leftBR   = (isFrontCAM) ? R_FRONT_LEFT_BR : SWITCH_LEFT_BR;
            middleTL = (isFrontCAM) ? R_FRONT_MIDDLE_TL : SWITCH_MIDDLE_TL;
            middleBR = (isFrontCAM) ? R_FRONT_MIDDLE_BR : SWITCH_MIDDLE_BR;
            rightTL  = (isFrontCAM) ? R_FRONT_RIGHT_TL : SWITCH_RIGHT_TL;
            rightBR  = (isFrontCAM) ? R_FRONT_RIGHT_BR : SWITCH_RIGHT_BR;
        }
    }

    public void stopStreaming() {
        camera.stopStreaming();
    }

    public enum POSITIONS {
        LEFT, MIDDLE, RIGHT
    }

    public POSITIONS getDecision() {

        POSITIONS position = POSITIONS.RIGHT;

        int leftValue   = left.getdBlue();
        int middleValue = middle.getdBlue();
        int rightValue  = right.getBlack(); //Ensure getBlack works instead of getdBlue

        if(leftValue > middleValue && leftValue > rightValue){
            position = POSITIONS.LEFT;
        }
        else if(middleValue > leftValue && middleValue > rightValue){
            position = POSITIONS.MIDDLE;
        }

        if (show_value){

            opMode.telemetry.addData("Position: ", position);
            opMode.telemetry.addData("Left Value: ", leftValue);
            opMode.telemetry.addData("Middle Value: ", middleValue);
            opMode.telemetry.addData("Right Value: ", rightValue);
            opMode.telemetry.update();
        }

        return position;
    }

    class CustomPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input){

            left = getAverageColor(input, leftTL, leftBR);
            middle = getAverageColor(input, middleTL, middleBR);
            right = getAverageColor(input, rightTL, rightBR);

            int thickness = 3;
            Scalar leftColor = new Scalar(255, 0, 0);
            Scalar middleColor = new Scalar(255, 255, 255);
            Scalar rightColor = new Scalar(0, 0, 255);

            Imgproc.rectangle(input, leftTL, leftBR, leftColor, thickness);
            Imgproc.rectangle(input, middleTL, middleBR, middleColor, thickness);
            Imgproc.rectangle(input, rightTL, rightBR, rightColor, thickness);

            //sendTelemetry();

            return input;
        }

        private RGBColor getAverageColor(Mat mat, Point topLeft, Point bottomRight) {
            int red = 0;
            int green = 0;
            int blue = 0;
            int total = 0;

            for (int x = (int)topLeft.x; x < bottomRight.x; x++){
                for (int y = (int)topLeft.y; y < bottomRight.y; y++){
                    red += mat.get(y,x)[0];
                    green += mat.get(y,x)[1];
                    blue += mat.get(y,x)[2];
                    total++;
                }
            }

            red /= total;
            green /= total;
            blue /= total;

            return new RGBColor(red, green, blue);
        }

        private void sendTelemetry() {
            opMode.telemetry.addLine("Left :" + " R " + left.getRed() + " G " + left.getGreen() + " B " + left.getdBlue());
            opMode.telemetry.addLine("Middle :" + " R " + middle.getRed() + " G " + middle.getGreen() + " B " + middle.getdBlue());
            opMode.telemetry.addLine("Right :" + " R " + right.getRed() + " G " + right.getGreen() + " B " + right.getdBlue());
            opMode.telemetry.update();
        }

    }

    public void setTelemShow(boolean show){
        this.show_value = show;
    }
}
