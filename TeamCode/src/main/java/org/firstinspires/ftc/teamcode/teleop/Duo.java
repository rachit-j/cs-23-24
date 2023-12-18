package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry;


@TeleOp(name="Duo", group="Linear Opmode")

public class Duo extends LinearOpMode {

    // Declare OpMode members.
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

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

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

        waitForStart();
        runtime.reset();
        double modifier = 1;

        imu.resetYaw();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        //odometers
        verticalLeft = hardwareMap.dcMotor.get("fl");
        verticalRight = hardwareMap.dcMotor.get("br");
        horizontal = hardwareMap.dcMotor.get("fr");

        //start odometry thread
        update = new odometry(verticalLeft, verticalRight, horizontal, 10, imu);
        Thread positionThread = new Thread(update);
        positionThread.start();

        resetRuntime();

        double clawclosevalue = 0.51;
        double clawopenvalue = 0.40;
        double linkagedownvalue = 0.53;
        double linkageupvalue = 0.08;
        double liftuppower = 0.8;
        double liftdownpower = 0.5;

        int liftupposition = 1500;
        int liftmidposition = 1000;
        int liftdownposition = 500;

        while (opModeIsActive()) {

            telemetry.addData("x: ", update.x() / COUNTS_PER_INCH);
            telemetry.addData("y: ", update.y() / COUNTS_PER_INCH);
            telemetry.addData("h: ", update.h());
            telemetry.addData("leftlift pos: ", leftlift.getCurrentPosition());
            telemetry.addData("rightleft pos: ", rightlift.getCurrentPosition());
            telemetry.update();


            /*
            DRIVER 1 YEAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
             */

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            fl.setPower(modifier*(y + x + turn));
            fr.setPower(modifier*(y - x - turn));
            bl.setPower(modifier*(y - x + turn));
            br.setPower(modifier*(y + x - turn));

            if (gamepad1.left_trigger > 0.1){
                modifier = 0.3;
            } else if (gamepad1.right_trigger > 0.1) {
                modifier = 1;
            } else modifier = 0.6;


            /*
            DRIVER 2 YEAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
             */

            if (gamepad2.dpad_up) {
                leftlift.setTargetPosition(liftupposition);
                rightlift.setTargetPosition(liftupposition);
                leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftlift.setPower(liftuppower);
                rightlift.setPower(liftuppower);
            }
            if (gamepad2.dpad_right) {
                leftlift.setTargetPosition(liftmidposition);
                rightlift.setTargetPosition(liftmidposition);
                leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftlift.setPower(liftuppower);
                rightlift.setPower(liftuppower);
            }
            if (gamepad2.dpad_down) {
                leftlift.setTargetPosition(liftdownposition);
                rightlift.setTargetPosition(liftdownposition);
                leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftlift.setPower(liftuppower);
                rightlift.setPower(liftuppower);
            }
            if (gamepad2.x) {
                leftlift.setTargetPosition(0);
                leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftlift.setPower(0);
                rightlift.setTargetPosition(0);
                rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightlift.setPower(0);
            }


            if (gamepad2.dpad_left && gamepad1.dpad_left) {
                lefthang.setPosition(0);
                righthang.setPosition(0);
                wrist.setPosition(linkageupvalue);
            }

            if (gamepad2.y) {
                wrist.setPosition(linkagedownvalue);
            }
            if (gamepad2.a) {
                wrist.setPosition(linkageupvalue);
            }

            if (gamepad2.left_trigger > 0.1) {
                leftclaw.setPosition(clawclosevalue);
            }
            if (gamepad2.left_bumper) {
                leftclaw.setPosition(clawopenvalue);
            }
            if (gamepad2.right_trigger > 0.1) {
                rightclaw.setPosition(clawclosevalue);
            }
            if (gamepad2.right_bumper) {
                rightclaw.setPosition(clawopenvalue);
            }
            if (gamepad2.left_stick_y > 0.5) {
                leftlift.setTargetPosition(leftlift.getCurrentPosition() - 100);
                rightlift.setTargetPosition(rightlift.getCurrentPosition() - 100);
                leftlift.setPower(1);
                rightlift.setPower(1);
            }
            if (gamepad2.left_stick_y < -0.5) {
                leftlift.setTargetPosition(leftlift.getCurrentPosition() + 100);
                rightlift.setTargetPosition(rightlift.getCurrentPosition() + 100);
                leftlift.setPower(1);
                rightlift.setPower(1);
            }

            if (gamepad2.b) {
                wrist.setPosition(linkagedownvalue);
                leftlift.setTargetPosition(0);
                leftlift.setPower(liftdownpower);
                rightlift.setTargetPosition(0);
                rightlift.setPower(liftdownpower);
            }

        }
    }
}
