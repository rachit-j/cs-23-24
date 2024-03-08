package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.io.File;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class odometry implements Runnable {
    //Odometry wheels
    private DcMotor encoderLeft, encoderRight, encoderAux;
    IMU imu;
    double globalAngle;
    double oldangle = 0;
    double currentangle = 0;
    private double getAngle() {
        currentangle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double deltaAngle = currentangle - oldangle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        oldangle = currentangle;
        return globalAngle;
    }


    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double currentRightPos = 0, currentLeftPos = 0, currentAuxPos = 0, currentIMU = 0,  orientationChange = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double oldRightPos = 0, oldLeftPos = 0, oldAuxPos = 0, oldIMU = 0;


    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public odometry(DcMotor encoderLeft, DcMotor encoderRight, DcMotor encoderAux, int threadSleepDelay, IMU imu){
        this.encoderLeft = encoderLeft;
        this.encoderRight = encoderRight;
        this.encoderAux = encoderAux;
        this.imu = imu;
        sleepTime = threadSleepDelay;
    }

    public void globalCoordinatePositionUpdate(){
        oldLeftPos = currentLeftPos;
        oldRightPos = currentRightPos;
        oldAuxPos = currentAuxPos;
        oldIMU = currentIMU;

        currentLeftPos = -encoderLeft.getCurrentPosition();
        currentRightPos = -encoderRight.getCurrentPosition();
        currentAuxPos = -encoderAux.getCurrentPosition();
        currentIMU = getAngle();

        double leftChange = currentLeftPos - oldLeftPos;
        double rightChange = currentRightPos - oldRightPos;
        double auxChange = currentAuxPos - oldAuxPos;
        double IMUChange = currentIMU - oldIMU;

//        orientationChange = (rightChange - leftChange) / 2045;
//        robotOrientationRadians = ((robotOrientationRadians + orientationChange)); //using odometry
        orientationChange = Math.toRadians(IMUChange);
        robotOrientationRadians = Math.toRadians(currentIMU); //using imu

        double horizontalChange = auxChange - (orientationChange * 1768.60931);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (n*Math.cos(robotOrientationRadians) - p*Math.sin(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (n*Math.sin(robotOrientationRadians) + p*Math.cos(robotOrientationRadians));
    }

    public double x(){ return robotGlobalXCoordinatePosition; }

    public double y(){ return robotGlobalYCoordinatePosition; }

    public double h(){ return Math.toDegrees(robotOrientationRadians); }

    public void stop(){ isRunning = false; }

    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}