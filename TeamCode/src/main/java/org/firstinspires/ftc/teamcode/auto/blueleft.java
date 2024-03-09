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
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Blue Left", group="Autonomous")
public class blueleft extends LinearOpMode {

    private PIDController longPID;
    private PIDController latPID;
    public static double plong = 0.05, ilong = 0, dlong = 3*Math.pow(10, -9);
    public static double plat = 0.15, ilat = 0, dlat = 3*Math.pow(10, -9);

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 2304; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 1536; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.2;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Red Range                                      Y      Cr     Cb
    //public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Blue Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 140.0);
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


    private static double maxpowermove = 0.8;
    private static double maxpowerstay = 0.6;
    private static double maxpowerturn = 0.5;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor leftlift = null;
    private DcMotor rightlift = null;
    private DcMotor backlift = null;
    private DcMotor intake = null;


    public double count = 2; // Amount of white pixels

    // Control variables
    public double depositpos = 0.8;
    public double intakepos = 0.245;
    public int boxstate = 0;
    public double flickerclose = 0.35;
    public double flickeropen = 0.8;
    public double launcherrelease = 0.5;
    public double launcherclose = 1;



    Servo launcher;
    Servo deposit;

    IMU imu;
    DcMotor verticalLeft, verticalRight, horizontal;
    final double COUNTS_PER_INCH = 336.877963;
    odometry update;



    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        longPID = new PIDController(plong, ilong, dlong);
        latPID = new PIDController(plat, ilat, dlat);

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        RobotHardware robot = new RobotHardware(fl, fr, bl, br);
        robot.innitHardwareMap();

        // other motors
        leftlift = hardwareMap.get(DcMotor.class, "leftlift");
        rightlift = hardwareMap.get(DcMotor.class, "rightlift");
        backlift = hardwareMap.get(DcMotor.class, "backlift");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // servos

        launcher = hardwareMap.get(Servo.class, "launcher");



        //odometers
        verticalLeft = hardwareMap.dcMotor.get("fl");
        verticalRight = hardwareMap.dcMotor.get("br");
        horizontal = hardwareMap.dcMotor.get("fr");

        //start odometry thread
        update = new odometry(verticalLeft, verticalRight, horizontal, 10, imu);
        Thread positionThread = new Thread(update);
        positionThread.start();

        // Configure Motors
        leftlift.setDirection(DcMotor.Direction.FORWARD);
        rightlift.setDirection(DcMotor.Direction.REVERSE);
        backlift.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

        // init lifts

        leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftlift.setTargetPosition(0);
        rightlift.setTargetPosition(0);
        backlift.setTargetPosition(0);
        leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftlift.setPower(1);
        rightlift.setPower(1);
        backlift.setPower(1);


        telemetry.addData("RectArea: ", myPipeline.getRectArea());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        imu.resetYaw();

        resetRuntime();

        //start of auto
        while(opModeIsActive()){
            // Lift & Position Telemetry
//            telemetry.addData("leftlift: ", leftlift.getCurrentPosition());
//            telemetry.addData("rightlift: ", rightlift.getCurrentPosition());
//            telemetry.addData("backlift: ", backlift.getCurrentPosition());
//            telemetry.addData("x: ", update.x() / COUNTS_PER_INCH);
//            telemetry.addData("y: ", update.y() / COUNTS_PER_INCH);
//            telemetry.addData("h: ", update.h());

            // Camera Stuff
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values
            testing(myPipeline);

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();


            double rectMidpointX = myPipeline.getRectMidpointX();
            double screenThird = CAMERA_WIDTH / 3.0;

            /*if(rectMidpointX > 2 * screenThird){
                telemetry.addLine("OBJECT IS ON THE RIGHT SIDE");
                telemetry.update();
                AUTONOMOUS_C();
            }
            else if(rectMidpointX > screenThird){
                telemetry.addLine("OBJECT IS IN THE MIDDLE");
                telemetry.update();
                AUTONOMOUS_B();
            }
            else {
                telemetry.addLine("OBJECT IS ON THE LEFT SIDE");
                telemetry.update();
                AUTONOMOUS_A();
            }*/
            AUTONOMOUS_B();

            telemetry.update();

            runtime.reset();


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

            longPID.setPID(plong, ilong, dlong);
            latPID.setPID(plat, ilat, dlat);
            double currentX = update.x() / COUNTS_PER_INCH;
            double currentY = update.y() / COUNTS_PER_INCH;
            double x = longPID.calculate(currentX, targetX);
            double y = latPID.calculate(currentY, targetY);

            double turn = 0.035 * (update.h() - targetOrientation);
            double theta = Math.toRadians(update.h());
            if (Math.abs(distanceX) < 1 || Math.abs(distanceY) < 1) {
                longPID.reset();
                latPID.reset();
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
        double x = 0.2 * distanceX;
        double y = 0.2 * distanceY;
        double turn = 0.03 * (update.h() - targetOrientation);
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
    public void STACKDEPOLOOP() {
        moveTo(-50, -3, 90, 8);
        // intake.setPower(0);
        runtime.reset();
        while (runtime.seconds() < 0.5) {
            stay(-50, -3, 90);
        }
        runtime.reset();

        moveTo(-64, -26, 112.5, 8);
        // intake.setPower(0);
        runtime.reset();
        while (runtime.seconds() < 0.5) {
            stay(-68, -26, 112.5);
        }
        runtime.reset();

        // intake 2 pixels

        moveTo(-50, -3, 90, 8);
        // intake.setPower(0);
        runtime.reset();
        while (runtime.seconds() < 0.5) {
            stay(-50, -3, 90);
        }
        runtime.reset();

        moveTo(0, -3, 90, 8);
        // intake.setPower(0);
        runtime.reset();
        while (runtime.seconds() < 0.5) {
            stay(0, -3, 90);
        }
        runtime.reset();

        moveTo(39, -32, 90, 8);
        // intake.setPower(0);
        runtime.reset();
        while (runtime.seconds() < 2) {
            stay(39, -32, 90);
        }
        runtime.reset();

        // deposit
    }


    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous A");

        // Phase 1: Dropping purple pixel
        // deposit.setPosition(1);
        runtime.reset();
        while (runtime.seconds() < 1) {
            stay(0, -3, 0);
        }
        runtime.reset();
        while (runtime.seconds() < 1) {
            stay(0, -3, 0);
            // setBoxup();
//            setFlickerclose();
        }
        moveTo(12, -30, 90, 8);
        runtime.reset();
        while (runtime.seconds() < 3) {
            stay(-2, -26, 90);
        }
        // intake.setPower(-0.5);
//        setFlickeropen();

        //moving to backdrop
        // setlift(500);
        moveTo(39, -32, 90, 8);
        // intake.setPower(0);
        runtime.reset();
        while (runtime.seconds() < 2) {
            stay(39, -32, 90);
        }
        runtime.reset();

        /* while (runtime.seconds() < 2) {
            deposit.setPosition(0.5);
        } */

        STACKDEPOLOOP();

        STACKDEPOLOOP();

        //parking
        moveTo(12, -20, 90, 8);
        // setBoxdown();
        moveTo(36, -3, 90, 0);

    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");

        // pick up purple pixel
        // deposit.setPosition(1);
        runtime.reset();
        while (runtime.seconds() < 1) {
            stay(0, -3, 0);
        }
        runtime.reset();
        while (runtime.seconds() < 1) {
            stay(0, -3, 0);
            // setBoxup();
//            setFlickerclose();
        }

        //move to line and drop purple pixel
        moveTo(12, -48, 0, 8);
        runtime.reset();
        while (runtime.seconds() < 3) {
            stay(12, -36, 90);
        }
        // intake.setPower(-0.5);
//        setFlickeropen();

        //moving to backdrop
        //setlift(500);
        moveTo(39, -26, 90, 8);
        //intake.setPower(0);
        runtime.reset();
        while (runtime.seconds() < 2) {
            stay(39, -26, 90);
        }
        runtime.reset();
        /*while (runtime.seconds() < 2) {
            deposit.setPosition(0.5);
        } */

        STACKDEPOLOOP();

        STACKDEPOLOOP();

        /*

        //parking
        setlift(0);*/
        moveTo(12, -20, 90, 8);
        //setBoxdown();
        moveTo(36, -3, 90, 0);

    }
    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous C");
        // pick up purple pixel
        //deposit.setPosition(1);
        runtime.reset();
        while (runtime.seconds() < 1) {
            stay(0, -3, 0);
        }
        runtime.reset();
        while (runtime.seconds() < 1) {
            stay(0, -3, 0);
            //setBoxup();
//            setFlickerclose();
        }

        //move to line and drop purple pixel
        moveTo(30, -20, 0, 8);
        runtime.reset();
        while (runtime.seconds() < 3) {
            stay(20, -28, 90);
        }
//        setFlickeropen();
        //intake.setPower(-0.5);

        //moving to backdrop
        //setlift(500);
        moveTo(39, -23, 90, 8);
        //intake.setPower(0);
        runtime.reset();
        while (runtime.seconds() < 2) {
            stay(39, -23, 90);
        }
        runtime.reset();
        /*while (runtime.seconds() < 2) {
            deposit.setPosition(0.5);
        }

        */

        STACKDEPOLOOP();

        STACKDEPOLOOP();

        /*

        //parking
        setlift(0);*/
        moveTo(12, -20, 90, 8);
        //setBoxdown();
        moveTo(36, -3, 90, 0);
    }


    public void setlift(int targetValue) {
        leftlift.setTargetPosition(targetValue);
        rightlift.setTargetPosition(targetValue);
        backlift.setTargetPosition(targetValue);
    }
}