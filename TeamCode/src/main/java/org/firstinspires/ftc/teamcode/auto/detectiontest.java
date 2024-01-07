package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Detection Test", group="Autonomous")
public class detectiontest extends LinearOpMode {

    private PIDController movePID;
    public static double p = 0.15, i = 0.5, d = 0.00000001; //0.15, 0.5, 8 0s 8

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 3840; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 2160; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    // Define border values
    private static final double BORDER_LEFT_X = 0.1;   // Example value: 10% of the image cropped from the left
    private static final double BORDER_RIGHT_X = 0.1;  // Example value: 10% of the image cropped from the right
    private static final double BORDER_TOP_Y = 0.1;    // Example value: 10% of the image cropped from the top
    private static final double BORDER_BOTTOM_Y = 0.1; // Example value: 10% of the image cropped from the bottom

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Red Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Blue Range                                      Y      Cr     Cb
    //public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 120.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(255.0, 100.0, 255.0);


    // Yellow Range
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    private static double maxpowermove = 0.6;
    private static double maxpowerstay = 0.6;
    private static double maxpowerturn = 0.5;

    private ElapsedTime runtime = new ElapsedTime();





    @Override
    public void runOpMode() {



        //start of camera code
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(BORDER_LEFT_X, BORDER_RIGHT_X, BORDER_TOP_Y, BORDER_BOTTOM_Y));

        // Configure Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        // Configure Borders for cropping
        myPipeline.configureBorders(BORDER_LEFT_X, BORDER_RIGHT_X, BORDER_TOP_Y, BORDER_BOTTOM_Y);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("There is an error: ", errorCode);
            }
        });

        telemetry.update();


//        double rectMidpointX = myPipeline.getRectMidpointX();
//        double screenThird = CAMERA_WIDTH / 3.0;
//
//        if (rectMidpointX > 2 * screenThird) {
//            telemetry.addLine("OBJECT IS ON THE RIGHT SIDE");
//        } else if (rectMidpointX > screenThird) {
//            telemetry.addLine("OBJECT IS IN THE MIDDLE");
//        } else {
//            telemetry.addLine("OBJECT IS ON THE LEFT SIDE");
//        }

        telemetry.update();

        // Camera Stuff
        if(myPipeline.error){
            telemetry.addData("Exception: ", myPipeline.debug);
        }
        telemetry.addData("RectArea: ", myPipeline.getRectArea());
        telemetry.update();

        waitForStart();

        resetRuntime();





        //start of auto
        while(opModeIsActive()){


            // Camera Stuff
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            double rectMidpointX = myPipeline.getRectMidpointX();
            double screenThird = CAMERA_WIDTH / 3.0;

            if(rectMidpointX > 2 * screenThird){
                telemetry.addLine("OBJECT IS ON THE RIGHT SIDE");
                AUTONOMOUS_C();
            }
            else if(rectMidpointX > screenThird){
                telemetry.addLine("OBJECT IS IN THE MIDDLE");
                AUTONOMOUS_B();
            }
            else {
                telemetry.addLine("OBJECT IS ON THE LEFT SIDE");
                AUTONOMOUS_A();
            }
            telemetry.update();

            runtime.reset();

        }

    }


    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }


    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous A");
        telemetry.addLine("I AM A GOOGLE THINGY");

    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");
        telemetry.addLine("I AM HUNGRY");


    }
    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous C");
        telemetry.addLine("Kaiden is weird");


    }
}



