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

    Servo leftbox;
    Servo rightbox;
    Servo leftflicker;
    Servo rightflicker;

    Servo launcher;
    Servo deposit;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        telemetry.update();

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        RobotHardware robot = new RobotHardware(fl, fr, bl, br);
        robot.innitHardwareMap();

        leftlift = hardwareMap.get(DcMotor.class, "leftlift");
        rightlift = hardwareMap.get(DcMotor.class, "rightlift");
        backlift = hardwareMap.get(DcMotor.class, "backlift");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftbox = hardwareMap.get(Servo.class, "leftbox");
        rightbox = hardwareMap.get(Servo.class, "rightbox");
        deposit = hardwareMap.get(Servo.class, "deposit");

        launcher = hardwareMap.get(Servo.class, "launcher");
        leftflicker = hardwareMap.get(Servo.class, "leftflicker");
        rightflicker = hardwareMap.get(Servo.class, "rightflicker");

        leftbox.setDirection(Servo.Direction.REVERSE);
        rightflicker.setDirection(Servo.Direction.REVERSE);
        launcher.setDirection(Servo.Direction.REVERSE);

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

        double depositpos = 0.8;
        double intakepos = 0.245;
        int boxstate = 0;
        double flickerclose = 0.35;
        double flickeropen = 0.8;
        double launcherrelease = 0.5;
        double launcherclose = 1;
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


            if (gamepad2.right_trigger > 0.5) {
                intake.setPower(1);
            }
            else if (gamepad2.left_trigger > 0.5) {
                intake.setPower(-1);
            }
            else intake.setPower(0);

            if (gamepad2.a) {
                leftbox.setPosition(depositpos);
                rightbox.setPosition(depositpos);
            }
            if (gamepad2.b) {
                leftbox.setPosition(depositpos);
                rightbox.setPosition(depositpos);
                setlift(0);
                boxstate = 1;
            }
            if (boxstate == 1 && leftlift.getCurrentPosition() < 500) {
                leftbox.setPosition(intakepos);
                rightbox.setPosition(intakepos);
                boxstate = 0;
            }
            if (gamepad2.right_bumper) {
                deposit.setPosition(0.5);
            } else deposit.setPosition(1);

            if (gamepad2.dpad_up) {
                setlift(1500);
            }
            if (gamepad2.dpad_right) {
                setlift(1000);
            }
            if (gamepad2.dpad_down) {
                setlift(500);
            }
            if (gamepad2.left_stick_y > 0.5) { //stick pointing down
                setlift(leftlift.getCurrentPosition() - 100);
            }
            if (gamepad2.left_stick_y < -0.5) {
                setlift(leftlift.getCurrentPosition() + 100);
            }
            if (gamepad1.x) {
                leftflicker.setPosition(flickerclose);
                rightflicker.setPosition(flickerclose);
            }
            else {
                leftflicker.setPosition(flickeropen);
                rightflicker.setPosition(flickeropen);
            }
            if (gamepad1.y) {
                launcher.setPosition(launcherrelease);
            } else launcher.setPosition(launcherclose);
        }
    }

    public void setlift(int targetValue) {
        leftlift.setTargetPosition(targetValue);
        rightlift.setTargetPosition(targetValue);
        backlift.setTargetPosition(targetValue);
    }
}
