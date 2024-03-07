package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
    private DcMotor backlift = null;
    private DcMotor intake = null;

    Servo leftintakearm;
    Servo rightintakearm;
    Servo mainrelease;
    Servo auxrelease;
    Servo leftboxarm;
    Servo rightboxarm;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        leftlift = hardwareMap.get(DcMotor.class, "leftlift");
        rightlift = hardwareMap.get(DcMotor.class, "rightlift");
        backlift = hardwareMap.get(DcMotor.class, "backlift");
        intake = hardwareMap.get(DcMotor.class, "intake");

        RobotHardware robot = new RobotHardware(fl, fr, bl, br);
        robot.innitHardwareMap();

        leftintakearm = hardwareMap.get(Servo.class, "leftintakearm");
        rightintakearm = hardwareMap.get(Servo.class, "rightintakearm");
        mainrelease = hardwareMap.get(Servo.class, "mainrelease");
        auxrelease = hardwareMap.get(Servo.class, "auxrelease");
        leftboxarm = hardwareMap.get(Servo.class, "leftboxarm");
        rightboxarm = hardwareMap.get(Servo.class, "rightboxarm");

        leftintakearm.setDirection(Servo.Direction.REVERSE);
        leftboxarm.setDirection(Servo.Direction.REVERSE);
        mainrelease.setDirection(Servo.Direction.REVERSE);

        leftlift.setDirection(DcMotor.Direction.FORWARD);
        rightlift.setDirection(DcMotor.Direction.REVERSE);
        backlift.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

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

        double boxdepositpos = 0.75;
        double boxintakepos = 0.18;
        double intakeup = 0.73;
        double intakedown = 0.41;
        double intakehalfdown = 0.55;
        double modifier = 1;

        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("leftlift: ", leftlift.getCurrentPosition());
            telemetry.addData("rightlift: ", rightlift.getCurrentPosition());
            telemetry.addData("backlift: ", backlift.getCurrentPosition());

            telemetry.update();

            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            if (gamepad1.left_trigger > 0.5) {
                modifier = 0.3;
            } else modifier = 1;

            fl.setPower(modifier*(y + x + turn));
            fr.setPower(modifier*(y - x - turn));
            bl.setPower(modifier*(y - x + turn));
            br.setPower(modifier*(y + x - turn));


//            if (gamepad2.right_trigger > 0.8) {
//                intake.setPower(1);
//                leftintakearm.setPosition(intakedown);
//                rightintakearm.setPosition(intakedown);
//            }
//            else if (gamepad2.right_trigger > 0.2) {
//                intake.setPower(1);
//                leftintakearm.setPosition(intakehalfdown);
//                rightintakearm.setPosition(intakehalfdown);
//            }
//            else if (gamepad2.left_trigger > 0.5) {
//                intake.setPower(-1);
//            }
//            else {
//                intake.setPower(0);
//                leftintakearm.setPosition(intakeup);
//                rightintakearm.setPosition(intakeup);
//            }
            if (gamepad2.right_trigger > 0.1) {
                intake.setPower(1);
                leftintakearm.setPosition(intakeup - (gamepad2.right_trigger*0.32));
                rightintakearm.setPosition(intakeup - (gamepad2.right_trigger*0.32));
            }
            else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
                leftintakearm.setPosition(intakeup);
                rightintakearm.setPosition(intakeup);
            }

            if (gamepad2.a) {
                leftboxarm.setPosition(boxdepositpos);
                rightboxarm.setPosition(boxdepositpos);
            }
            if (gamepad2.b) {
                setlift(0);
                leftboxarm.setPosition(boxintakepos);
                rightboxarm.setPosition(boxintakepos);
                auxrelease.setPosition(0);
                mainrelease.setPosition(0.6);
            }

            if (gamepad2.dpad_up) {
                setlift(1800);
            }
            if (gamepad2.dpad_right) {
                setlift(1200);
            }
            if (gamepad2.dpad_down) {
                setlift(600);
            }
            if (gamepad2.left_stick_y > 0.5) { //stick pointing down
                setlift(leftlift.getCurrentPosition() - 200);
            }
            if (gamepad2.left_stick_y < -0.5) {
                setlift(leftlift.getCurrentPosition() + 200);
            }
            if (gamepad2.right_bumper) {
                mainrelease.setPosition(0.6);
                auxrelease.setPosition(0.8);
            }
            if (gamepad2.left_bumper) {
                mainrelease.setPosition(0.45);
                auxrelease.setPosition(0.8);
            }
            if (gamepad2.x) {
                mainrelease.setPosition(0.4);
            }
        }
    }

    public void setlift(int targetValue) {
        leftlift.setTargetPosition(targetValue);
        rightlift.setTargetPosition(targetValue);
        backlift.setTargetPosition(targetValue);
    }
}
