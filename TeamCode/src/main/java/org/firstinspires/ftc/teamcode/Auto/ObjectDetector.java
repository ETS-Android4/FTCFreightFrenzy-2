package org.firstinspires.ftc.teamcode.Auto;

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

    private final Point BLUE_TOP_TL    = new Point(130,40);
    private final Point BLUE_TOP_BR    = new Point(185, 85);


    private final Point RED_TOP_TL     = new Point(0,135);
    private final Point RED_TOP_BR     = new Point(50, 165);


    private Point topTL;
    private Point topBR;

    private RGBColor box;
    private boolean show_value = true;

    public ObjectDetector(OpMode op, boolean isRed){

        opMode = op;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "sideCAM"), cameraMonitorViewId);

        pipeline = new CustomPipeline();
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        topTL = (isRed) ? RED_TOP_TL : BLUE_TOP_TL;
        topBR = (isRed) ? RED_TOP_BR : BLUE_TOP_BR;
    }

    public void stopStreaming(){
        camera.stopStreaming();
    }

    public int getDecision(){

        int boxValue = box.getBlue();

        if (show_value){
            opMode.telemetry.addData("Box Value: ", boxValue);
            opMode.telemetry.update();
        }

        if (boxValue < 200) {
            return 0;
        }
        else if (boxValue < 270) {
            return 1;
        }
        else return 2;
    }

    class CustomPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input){

            box = getAverageColor(input, topTL, topBR);

            int thickness = 3;

            Scalar topColor = new Scalar(255,0,0);
            int position = getDecision();
            if (position == 4){
                topColor = new Scalar(0,255,0);
            }
            else if (position == 1){
                topColor = new Scalar(0,255,0);
            }

            Imgproc.rectangle(input, topTL, topBR, topColor, thickness);

            //sendTelemetry();

            return input;
        }

        private RGBColor getAverageColor(Mat mat, Point topLeft, Point bottomRight){
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

        private void sendTelemetry(){
            opMode.telemetry.addLine("Top :" + " R " + box.getRed() + " G " + box.getGreen() + " B " + box.getBlue());
            opMode.telemetry.update();
        }

    }

    public void setTelemShow(boolean show){
        this.show_value = show;
    }
}