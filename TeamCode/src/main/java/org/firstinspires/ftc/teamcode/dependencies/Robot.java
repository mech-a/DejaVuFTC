// Cleaned up version of HWRobot to be used for 1st Competition.
// Will be deprecated after extended classes for dcmotor are made/subassemblies/class

package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import static org.firstinspires.ftc.teamcode.dependencies.Constants.*;
import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.*;

//TODO see if throw exception clause?
public class Robot {
    //TODO based on specification, add more motor slots

    public DcMotor[] driveMotors = new DcMotor[4];
    public DcMotor[] intakeMotors = new DcMotor[4];

    public double[] driveMtrPowers = new double[4];
    public double[] intakeMtrPowers = new double[4];

    public int[] driveMtrTargets = new int[4];
    public int[] intakeMtrTargets = new int[4];

    public BNO055IMU imu;
    //public VuforiaLocalizer vuf;

    LinearOpMode caller;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    Orientation angles;


    public Robot(LinearOpMode initializer) {
        caller = initializer;
    }

    public void start(HardwareMap h, Telemetry t) {
        hardwareMap = h;
        telemetry = t;
    }

    //TODO implement enum for init
    //currently will init all hardware
    public void init() {
        driveMotorsInit();
        armMotorsInit();
        imuInit();
    }

    private void driveMotorsInit() {
        for (int i = 0; i<4; i++) {
            driveMotors[i] = hardwareMap.dcMotor.get(DRIVE_MOTOR_NAMES[i]);

            //TODO change if needed
            if(i%2 == 0)
                driveMotors[i].setDirection(DcMotor.Direction.REVERSE);
            else
                driveMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);

            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    //TODO standardized naming system (intake vs arm)
    private void armMotorsInit() {
        for (int i = 0; i<4; i++) {
            intakeMotors[i] = hardwareMap.dcMotor.get(ARM_MOTOR_NAMES[i]);

            //TODO change if needed
            if(i%2 == 0)
                intakeMotors[i].setDirection(DcMotor.Direction.REVERSE);
            else
                intakeMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);

            intakeMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void imuInit() {
        imu = hardwareMap.get(BNO055IMU.class, SENSOR_NAMES[0]);
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParameters.loggingEnabled      = true;
        gyroParameters.loggingTag          = "IMU";
        imu.initialize(gyroParameters);
        //TODO using angles while opmode is not active could cause problems
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    //TODO again enum the motors.
    //TODO make position drive compatible with arm motors
    //TODO stability? waitfullhardwarecycle? not sure about this. check LinearOpMode to see if something could work
    //Also rethink naming style, drive+whatever is getting really reptitive and long.
    public void positionDrive(int motorNum, double inches, double speed) {
        driveMtrTargets[motorNum] = (int) (inches * HD_COUNTS_PER_REV);
        driveMotors[motorNum].setTargetPosition(driveMtrTargets[motorNum]);
        driveMotors[motorNum].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotors[motorNum].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveMotors[motorNum].setPower(speed);

        while(caller.opModeIsActive() & driveMotors[motorNum].isBusy()) {
            //TODO change telemetry name to enum
            telemetry.addData(motorNum + ":", "%7d : %7d",
                    driveMotors[motorNum].getCurrentPosition(), driveMtrTargets[motorNum]);
            telemetry.update();
        }

        driveMotors[motorNum].setPower(0);
        driveMotors[motorNum].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //TODO just realized that position drive cannot be used for translate, it'll sequentially do each motor. mmmf
    public void translate(int counts, double speed) {
        //positionDrive();
    }



    public void getStatus() {
        //TODO implement a status message, possibly useful for the invoker
    }



}
