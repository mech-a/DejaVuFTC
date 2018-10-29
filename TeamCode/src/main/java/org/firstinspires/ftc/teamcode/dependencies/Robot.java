// Cleaned up version of HWRobot to be used for 1st Competition.
// Will be deprecated after extended classes for dcmotor are made/subassemblies/class

package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import static org.firstinspires.ftc.teamcode.dependencies.Constants.*;
import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.*;

//TODO see if throw exception clause?
public class Robot {
    //TODO based on specification, add more motor slots



    //FL,FR,BR,BR
    public DcMotor[] driveMotors = new DcMotor[4];
    //Raise, Telescope, Rotation, Intake
    public DcMotor[] armMotors = new DcMotor[4];

    private double[] driveMtrPowers = new double[4];
    private double[] armMtrPowers = new double[4];

    private int[] driveMtrTargets = new int[4];
    private int driveMtrTarget;
    private int[] armMtrTargets = new int[4];

    private BNO055IMU imu;
    //public VuforiaLocalizer vuf;

    private LinearOpMode caller;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Orientation angles;

    private double heading;

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

            //TODO standardize between robot and code the port numbers and i
            /*
            switch (i) {
                case 0: driveMotors[i].setDirection(DcMotor.Direction.FORWARD);
                    break;
                case 1: driveMotors[i].setDirection(DcMotor.Direction.REVERSE);
                    break;
                case 2: driveMotors[i].setDirection(DcMotor.Direction.REVERSE);
                    break;
                case 3: driveMotors[i].setDirection(DcMotor.Direction.FORWARD);
                    break;

            }*/

            if(i%3==0)
                driveMotors[i].setDirection(DcMotor.Direction.REVERSE);
            else
                driveMotors[i].setDirection(DcMotor.Direction.FORWARD);

            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void armMotorsInit() {
        for (int i = 0; i<4; i++) {
            armMotors[i] = hardwareMap.dcMotor.get(ARM_MOTOR_NAMES[i]);

            //TODO change if needed: well, i did it, but must be changed for when telescoping works
            /*
            switch (i) {
                case 0: armMotors[i].setDirection(DcMotor.Direction.FORWARD);
                        break;
                case 1: armMotors[i].setDirection(DcMotor.Direction.FORWARD);
                    break;
                case 2: armMotors[i].setDirection(DcMotor.Direction.FORWARD);
                    break;
                case 3: armMotors[i].setDirection(DcMotor.Direction.REVERSE);
                    break;
            }*/

            if(i==3)
                armMotors[i].setDirection(DcMotor.Direction.REVERSE);
            else
                armMotors[i].setDirection(DcMotor.Direction.FORWARD);

            armMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            armMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        driveMtrTargets[motorNum] = (int) (inches * HD_COUNTS_PER_INCH);
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
    //used to move forward and back

    public void translate(double inches, double speed) {

        driveMtrTarget = (int) (inches * HD_COUNTS_PER_INCH);
        for (int i = 0; i<4; i++) {
            driveMotors[i].setTargetPosition(driveMtrTarget);
            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (int i = 0; i<4; i++) {
            driveMotors[i].setPower(speed);
        }
        while(caller.opModeIsActive() & driveMotors[0].isBusy() && driveMotors[1].isBusy() && driveMotors[2].isBusy() & driveMotors[3].isBusy()) {
            //TODO change telemetry name to enum
            telemetry.addData("current position of motors", "%7d : %7d",
                    driveMotors[0].getCurrentPosition(), driveMotors[1].getCurrentPosition(),driveMotors[2].getCurrentPosition(),driveMotors[3].getCurrentPosition());
            telemetry.update();
        }
        for (int i = 0; i<4; i++) {

            driveMotors[i].setPower(0);

        }
        for (int i = 0; i<4; i++) {

            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }


    }
    public void rotate(String direction,double speed, double angle) {
        for (int i = 0; i<4; i++) {driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        heading = Math.floor(heading);
        heading = Range.clip(heading, -180.0, 180.0);


        if (direction.equals("cw") || direction.equals("clockwise")) {
            while (Math.abs(heading) > angle && caller.opModeIsActive()) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

                for (int i = 0; i<4; i++) {driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);}

                for (int i = 0; i<4; i++) {
                    if(i%3==0)
                        driveMtrPowers[i] = speed;
                    else
                        driveMtrPowers[i] = -speed;
                }

                for (int i = 0; i<4; i++) {driveMotors[i].setPower(driveMtrPowers[i]);}

            }
        } else if (direction.equals("counterclockwise") || direction.equals("ccw")) {
            while (heading < angle && caller.opModeIsActive()) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

                for (int i = 0; i<4; i++) {driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);}

                for (int i = 0; i<4; i++) {
                    if(i%3==0)
                        driveMtrPowers[i] = -speed;
                    else
                        driveMtrPowers[i] = speed;
                }

                for (int i = 0; i<4; i++) {driveMotors[i].setPower(driveMtrPowers[i]);}
            }
        }

        for (int i = 0; i<4; i++) {driveMtrPowers[i] = 0; }
        for (int i = 0; i<4; i++) {driveMotors[i].setPower(driveMtrPowers[i]);}
        for (int i = 0; i<4; i++) {driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        for (int i = 0; i<4; i++) {driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
    }



    public void getStatus() {
        //TODO implement a status message, possibly useful for the invoker
    }



}
