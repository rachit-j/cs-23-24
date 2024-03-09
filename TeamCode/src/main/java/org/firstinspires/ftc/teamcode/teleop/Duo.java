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
    Servo launcher;



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

        leftintakearm = hardwareMap.get(Servo.class, "leftintakearm");
        rightintakearm = hardwareMap.get(Servo.class, "rightintakearm");
        mainrelease = hardwareMap.get(Servo.class, "mainrelease");
        auxrelease = hardwareMap.get(Servo.class, "auxrelease");
        leftboxarm = hardwareMap.get(Servo.class, "leftboxarm");
        rightboxarm = hardwareMap.get(Servo.class, "rightboxarm");
        launcher = hardwareMap.get(Servo.class, "launcher");

        RobotHardware robot = new RobotHardware(fl, fr, bl, br, leftlift, rightlift, backlift, intake,
                leftintakearm, rightintakearm, mainrelease, auxrelease, leftboxarm, rightboxarm, launcher);
        robot.innitHardwareMap();

        waitForStart();


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


            if (gamepad2.right_trigger > 0.1) {
                intake.setPower(1);
                leftintakearm.setPosition(0.73 - (gamepad2.right_trigger*0.32));
                rightintakearm.setPosition(0.73 - (gamepad2.right_trigger*0.32));
            }
            else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
                robot.intakeup();
            }

            if (gamepad2.a) {
                robot.boxup();
            }

            if (gamepad2.b) {
                robot.setlift(0);
                robot.boxdown();
                robot.boxintakeready();
            }

            if (gamepad2.dpad_up) {
                robot.setlift(1800);
            }
            if (gamepad2.dpad_right) {
                robot.setlift(1200);
            }
            if (gamepad2.dpad_down) {
                robot.setlift(600);
            }
            if (gamepad2.left_stick_y > 0.5) { //stick pointing down
                robot.setlift(leftlift.getCurrentPosition() - 200);
            }
            if (gamepad2.left_stick_y < -0.5) {
                robot.setlift(leftlift.getCurrentPosition() + 200);
            }
            if (gamepad2.right_bumper) {
                robot.releasetwo();
            }
            if (gamepad2.left_bumper) {
                robot.releaseone();
            }
            if (gamepad2.x) {
                robot.boxtransferready();
            }
        }
    }

}
