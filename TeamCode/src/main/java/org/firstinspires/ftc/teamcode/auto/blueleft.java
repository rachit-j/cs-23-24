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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Blue Left", group="Autonomous")
public class blueleft extends LinearOpMode {

    private PIDController movePID;
    public static double p = 0.15, i = 0.5, d = 0.00000001; //0.15, 0.5, 8 0s 8

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 2304; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 1536; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Red Range                                      Y      Cr     Cb
    // public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
    // public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Blue Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 100.0, 255.0);


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
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor leftlift = null;
    private DcMotor rightlift = null;

    Servo wrist;
    Servo leftclaw;
    Servo rightclaw;
    Servo lefthang;
    Servo righthang;

    IMU imu;
    DcMotor verticalLeft, verticalRight, horizontal;
    final double COUNTS_PER_INCH = 1860;
    odometry update;

    double clawclosevalue = 0.51;
    double clawopenvalue = 0.41;
    double linkagedownvalue = 0.53;
    double linkageupvalue = 0.08;
    double liftuppower = 0.8;
    double liftdownpower = 0.5;

    int liftupposition = 1500;
    int liftmidposition = 1000;
    int liftdownposition = 500;





    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));


        movePID = new PIDController(p, i, d);

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        leftlift = hardwareMap.get(DcMotor.class, "leftlift");
        rightlift = hardwareMap.get(DcMotor.class, "rightlift");

        wrist = hardwareMap.get(Servo.class, "wrist");
        leftclaw = hardwareMap.get(Servo.class, "leftclaw");
        rightclaw = hardwareMap.get(Servo.class, "rightclaw");
        lefthang = hardwareMap.get(Servo.class, "lefthang");
        righthang = hardwareMap.get(Servo.class, "righthang");

        righthang.setDirection(Servo.Direction.REVERSE);

        rightclaw.setDirection(Servo.Direction.REVERSE);

        RobotHardware robot = new RobotHardware(fl, fr, bl, br, leftlift, rightlift);
        robot.innitHardwareMap();


        //odometers
        verticalLeft = hardwareMap.dcMotor.get("fl");
        verticalRight = hardwareMap.dcMotor.get("br");
        horizontal = hardwareMap.dcMotor.get("fr");

        //start odometry thread
        update = new odometry(verticalLeft, verticalRight, horizontal, 10, imu);
        Thread positionThread = new Thread(update);
        positionThread.start();

        //start of camera code
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("There is an error: ", errorCode);
            }
        });

        telemetry.update();
        leftclaw.setPosition(clawclosevalue);
        rightclaw.setPosition(clawclosevalue);
        wrist.setPosition(linkagedownvalue);

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
        myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
        if(myPipeline.error){
            telemetry.addData("Exception: ", myPipeline.debug);
        }
        telemetry.addData("RectArea: ", myPipeline.getRectArea());
        telemetry.update();

        waitForStart();
        imu.resetYaw();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        resetRuntime();





        //start of auto
        while(opModeIsActive()){


            // Camera Stuff
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values
            testing(myPipeline);

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            leftlift.setTargetPosition(25);
            rightlift.setTargetPosition(25);
            leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftlift.setPower(liftuppower);
            rightlift.setPower(liftuppower);

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
            while (runtime.seconds() < 0.5) {
                rightclaw.setPosition(clawopenvalue);
            }
            moveTo(20, -19, -90, 8);
            moveTo(20, -19, 0, 3);
            wrist.setPosition(linkagedownvalue);
            leftlift.setTargetPosition(0);
            leftlift.setPower(liftdownpower);
            rightlift.setTargetPosition(0);
            rightlift.setPower(liftdownpower);
            moveTo(40, 0, 90, 0);

        }

    }

    public void moveTo(double targetX, double targetY, double targetOrientation, double error) {
        double distanceX = targetX - (update.x() / COUNTS_PER_INCH);
        double distanceY = targetY - (update.y() / COUNTS_PER_INCH);
        double distance = Math.hypot(distanceX, distanceY);
        while(opModeIsActive() && distance > error) {
            distance = Math.hypot(distanceX, distanceY);
            distanceX = targetX - (update.x() / COUNTS_PER_INCH);
            distanceY = targetY - (update.y() / COUNTS_PER_INCH);

            movePID.setPID(p, i, d);
            double currentX = update.x() / COUNTS_PER_INCH;
            double currentY = update.y() / COUNTS_PER_INCH;
            double x = movePID.calculate(currentX, targetX);
            double y = movePID.calculate(currentY, targetY);

            double turn = 0.045 * (update.h() - targetOrientation);
            double theta = Math.toRadians(update.h());
            if (Math.abs(distanceX) < 1 || Math.abs(distanceY) < 1) {
                movePID.reset();
            }
            if (x > maxpowermove) {
                x = maxpowermove;
            }
            else if (x < -maxpowermove) {
                x = -maxpowermove;
            }
            else x = x;
            if (y > maxpowermove) {
                y = maxpowermove;
            }
            else if (y < -maxpowermove) {
                y = -maxpowermove;
            }
            else y = y;
            if (turn > maxpowerturn) {
                turn = maxpowerturn;
            }
            else if (turn < -maxpowerturn) {
                turn = -maxpowerturn;
            }
            else turn = turn;
            double l = y * Math.sin(theta + (Math.PI/4)) - x * Math.sin(theta - (Math.PI/4));
            double r = y * Math.cos(theta + (Math.PI/4)) - x * Math.cos(theta - (Math.PI/4));
            fl.setPower(l + turn);
            fr.setPower(r - turn);
            bl.setPower(r + turn);
            br.setPower(l - turn);
            if(isStopRequested()) {
                update.stop();
            }
        }
    }
    public void stay(double targetX, double targetY, double targetOrientation) {
        double distanceX = targetX - (update.x() / COUNTS_PER_INCH);
        double distanceY = targetY - (update.y() / COUNTS_PER_INCH);
        double x = 0.1 * distanceX;
        double y = 0.1 * distanceY;
        double turn = 0.035 * (update.h() - targetOrientation);
        double theta = Math.toRadians(update.h());

        if (x > maxpowerstay) {
            x = maxpowerstay;
        }
        else if (x < -maxpowerstay) {
            x = -maxpowerstay;
        }
        else x = x;
        if (y > maxpowerstay) {
            y = maxpowerstay;
        }
        else if (y < -maxpowerstay) {
            y = -maxpowerstay;
        }
        else y = y;
        if (turn > maxpowerturn) {
            turn = maxpowerturn;
        }
        else if (turn < -maxpowerturn) {
            turn = -maxpowerturn;
        }
        else turn = turn;

        double l = y * Math.sin(theta + (Math.PI/4)) - x * Math.sin(theta - (Math.PI/4));
        double r = y * Math.cos(theta + (Math.PI/4)) - x * Math.cos(theta - (Math.PI/4));

        fl.setPower(l + turn);
        fr.setPower(r - turn);
        bl.setPower(r + turn);
        br.setPower(l - turn);

        if(isStopRequested()) {
            update.stop();
        }
    }
    public void testing(ContourPipeline myPipeline){
        if(lowerruntime + 0.05 < getRuntime()){
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if(upperruntime + 0.05 < getRuntime()){
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        telemetry.addData("lowerCr ", (int)CrLowerUpdate);
        telemetry.addData("lowerCb ", (int)CbLowerUpdate);
        telemetry.addData("UpperCr ", (int)CrUpperUpdate);
        telemetry.addData("UpperCb ", (int)CbUpperUpdate);
    }
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }


    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous A");
        runtime.reset();
        while (runtime.seconds() < 3) {
            stay(23, -24, 90);
        }
        runtime.reset();
        while (runtime.seconds() < 0.5) {
            leftclaw.setPosition(clawopenvalue);
        }
        moveTo(28, -24, 90, 3);
        runtime.reset();
        while (runtime.seconds() < 3) {
            stay(32, -12, -90);
            leftlift.setTargetPosition(350);
            rightlift.setTargetPosition(350);
            leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftlift.setPower(liftuppower);
            rightlift.setPower(liftuppower);
            wrist.setPosition(linkageupvalue);
        }
        runtime.reset();
        while (runtime.seconds() < 2) {
            stay(37, -12, -90);
        }

    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");
        runtime.reset();
        while (runtime.seconds() < 3) {
            stay(14, -34, 90);
        }
        runtime.reset();
        while (runtime.seconds() < 0.5) {
            leftclaw.setPosition(clawopenvalue);
        }
        moveTo(23, -32, 90, 3);
        runtime.reset();
        while (runtime.seconds() < 3) {
            stay(32, -19, -90);
            leftlift.setTargetPosition(350);
            rightlift.setTargetPosition(350);
            leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftlift.setPower(liftuppower);
            rightlift.setPower(liftuppower);
            wrist.setPosition(linkageupvalue);
        }
        runtime.reset();
        while (runtime.seconds() < 2) {
            stay(37, -19, -90);
        }


    }
    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous C");
        moveTo(6, -24, 90, 8);
        runtime.reset();
        while (runtime.seconds() < 3) {
            stay(0, -24, 90);
        }
        runtime.reset();
        while (runtime.seconds() < 0.5) {
            leftclaw.setPosition(clawopenvalue);
        }
        moveTo(1, -24, 90, 3);
        runtime.reset();
        while (runtime.seconds() < 3) {
            stay(32, -25, -90);
            leftlift.setTargetPosition(350);
            rightlift.setTargetPosition(350);
            leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftlift.setPower(liftuppower);
            rightlift.setPower(liftuppower);
            wrist.setPosition(linkageupvalue);
        }
        runtime.reset();
        while (runtime.seconds() < 2) {
            stay(37, -25, -90);
        }

    }
}



