// Cleaned up version of HWRobot to be used for 1st Competition.
// Will be deprecated after extended classes for dcmotor are made/subassemblies/class

package org.firstinspires.ftc.teamcode.dependencies;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.DogeCVTesting.CustomGoldDetector;

import static org.firstinspires.ftc.teamcode.dependencies.Constants.*;
import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.*;
import static org.firstinspires.ftc.teamcode.dependencies.Enums.*;

/**
 * Robot
 * Robot is a dependency file that allows de-clutters
 * the code by eliminating the need to redefine motor
 * values in every
 *
 * @author Gaurav
 * @version 1.19
 * @since 2018 10 20
 */
//TODO see if throw exception clause?
public class Robot {
    //TODO based on specification, add more motor slots

    /**
     * These variables are used to define new motors
     * and their power values as arrays
     */
    //FL,FR,BR,BR
    public DcMotor[] driveMotors = new DcMotor[4];
    public Servo[] servoMotors = new Servo[2];

    //Raise, Telescope, Rotation, Intake
    public DcMotor[] armMotors = new DcMotor[4];

    private int adjustmentForangle = 3;
    private int adjustmentFortranslation = 1;


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
    private double lastangle = 0;
    private boolean ccwRotation = false;


    private CustomGoldDetector detector;

    private OpModeType callerType = OpModeType.AUTON;



    public Robot(LinearOpMode initializer) {
        caller = initializer;
    }

    public Robot(LinearOpMode initializer, OpModeType opModeType) {
        caller = initializer;
        callerType = opModeType;

    }

    public void start(HardwareMap h, Telemetry t) {
        hardwareMap = h;
        telemetry = t;
    }

    //TODO implement enum for init

    /**
     * the init() and method initializes all the hardware
     * for the robot
     */
    public void init() {
        driveMotorsInit();
        armMotorsInit();
        imuInit();
        servoMotorsInit();
        telemetry.addData("Stat", "Initialized!");
    }

    private void servoMotorsInit(){
        for(int i = 0; i<2 && !caller.isStopRequested(); i++){
            servoMotors[i] = hardwareMap.servo.get(SERVO_MOTOR_NAMES[i]);
        }

        if(callerType == OpModeType.AUTON && !caller.isStopRequested()) {
            servoMotors[0].setPosition(MARKER_HELD);
            servoMotors[1].setPosition(SERVO_LOCKED);
        }
            //no need to change servos for teleop

    }
    private void driveMotorsInit() {
        for (int i = 0; i<4 && !caller.isStopRequested(); i++) {
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
            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    private void armMotorsInit() {
        for (int i = 0; i<4 && !caller.isStopRequested(); i++) {
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

            if(!caller.isStopRequested()) {
                armMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
    private void imuInit() {
        imu = hardwareMap.get(BNO055IMU.class, SENSOR_NAMES[0]);
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParameters.loggingEnabled      = true;
        gyroParameters.loggingTag          = "IMU";
        if(!caller.isStopRequested()) {
            imu.initialize(gyroParameters);
            //TODO using angles while opmode is not active could cause problems
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }

    }

    public void detectorInit() {
        detector = new CustomGoldDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        if(!caller.isStopRequested())
            detector.enable();
    }

    //TODO again enum the motors.
    //TODO make position drive compatible with arm motors
    //TODO stability? waitfullhardwarecycle? not sure about this. check LinearOpMode to see if something could work

    /**
     * positionDrive is a method that moves the motor to a position
     * @param motorNum
     * @param counts
     * @param speed
     */
    //Also rethink naming style, drive+whatever is getting really repetitive and long.
    public void positionDrive(int motorNum, int counts, double speed) {


        armMotors[motorNum].setTargetPosition(counts);
        if(!caller.isStopRequested()) {
            armMotors[motorNum].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotors[motorNum].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        armMotors[motorNum].setPower(speed);

        while(!caller.isStopRequested() && armMotors[motorNum].isBusy()) {
            //TODO change telemetry name to enum
            telemetry.addData(motorNum + ":", "%7d : %7d",
                    armMotors[motorNum].getCurrentPosition(), counts);
            telemetry.update();
        }

        armMotors[motorNum].setPower(0);
        armMotors[motorNum].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //TODO just realized that position drive cannot be used for translate, it'll sequentially do each motor. mmmf
    //used to move forward and back

    /**
     * translate is a method that takes inches and translates them
     * into counts for the motors
     * @param inches
     * @param speed
     */
    public void translate(double inches, double speed) {
        double localizedInches;

        if(speed > 0)
            localizedInches = inches - adjustmentFortranslation;
        else
            localizedInches =  -inches + adjustmentFortranslation;

        driveMtrTarget = (int) (localizedInches * HD_COUNTS_PER_INCH);

        for (int i = 0; i<4 && !caller.isStopRequested(); i++) {
            driveMotors[i].setTargetPosition(driveMtrTarget);
            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // try this out
        // caller.sleep(750);

        for (int i = 0; i<4 && !caller.isStopRequested(); i++) {
            driveMotors[i].setPower(speed);
        }

        while(!caller.isStopRequested() &&
                ((driveMotors[0].isBusy()) && (driveMotors[1].isBusy()) && (driveMotors[2].isBusy()) && (driveMotors[3].isBusy())) ) {
            //TODO change telemetry name to enum
            telemetry.addData("0mtrFl", "%7d : %7d",
                    driveMotors[0].getCurrentPosition(), driveMtrTarget);
            telemetry.addData("1mtrFR", "%7d : %7d",
                    driveMotors[1].getCurrentPosition(), driveMtrTarget);
            telemetry.addData("2mtrBR", "%7d : %7d",
                    driveMotors[2].getCurrentPosition(), driveMtrTarget);
            telemetry.addData("3mtrBL", "%7d : %7d",
                    driveMotors[3].getCurrentPosition(), driveMtrTarget);

            telemetry.update();
        }

        for (int i = 0; i<4 && !caller.isStopRequested(); i++) {
            driveMotors[i].setPower(0);
        }
        for (int i = 0; i<4 && !caller.isStopRequested(); i++) {
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    /**
     * Rotate is a method that uses he IMU within the Rev Hubs
     * and rotates within autonomous. This method takes params
     * that allow it to determine which motors go which way
     * based on clockwise(cw) and counterclockwise(ccw). It uses
     * other methods to determine when the desired angle has been
     * reached and stop as well as a method to reset the gyroscopic
     * sensor.
     * @param direction
     * @param speed
     * @param angle
     */

    public boolean GoldinCenter() {
        return detector.getScreenPosition().x < 400 && detector.getScreenPosition().x > 200;
    }

    /**
     * polarTranslate is for mecanum wheels
     */
    public void polarTranslate(String direction, double inches, double speed, double angle) {
        double forwardsVector;
        double horizontalVector;
        double localAngle;

        if(direction == "cw") {
            localAngle = angle;
        } else if (direction == "ccw") {
            localAngle = -angle;
        }

        forwardsVector = Math.cos(localAngle) * inches;
        horizontalVector = Math.sin(localAngle) * inches;

        //set FL and BR to the difference of forwards and horizontal vectors
        //set FR and BL to the sum of the vectors
        //Assumes strafing and normal movement are at the same speed (if not add a multiplier)


    }

    //Rotate function that inputs a direction
    //Directions can be abbreviated to 'cw' or 'ccw'
    //It does not currently reset the gyro sensor
    public void rotate(String direction, double speed, double angle) {
        angle = angle - adjustmentForangle;
//        for (int i = 0; i<4; i++) {driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//
//        heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
//        heading = Math.floor(heading);
//        heading = Range.clip(heading, -180.0, 180.0);
//
//        //boolean beforeAngle = (direction.equals("cw") ? heading > angle | heading<angle);
//
//        if (direction.equals("cw") || direction.equals("clockwise")) {
//            while (Math.abs(heading) > angle && !caller.isStopRequested()) {
//
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
//
//                for (int i = 0; i<4 && !caller.isStopRequested(); i++) {driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
//
//                for (int i = 0; i<4 && !caller.isStopRequested(); i++) {
//                    if(i%3==0)
//                        driveMtrPowers[i] = speed;
//                    else
//                        driveMtrPowers[i] = -speed;
//                }
//
//                for (int i = 0; i<4 && !caller.isStopRequested(); i++) {driveMotors[i].setPower(driveMtrPowers[i]);}
//
//            }
//        } else if (direction.equals("counterclockwise") || direction.equals("ccw")) {
//            while (heading < angle && !caller.isStopRequested()) {
//
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
//
//                for (int i = 0; i<4; i++) {driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
//
//                for (int i = 0; i<4; i++) {
//                    if(i%3==0)
//                        driveMtrPowers[i] = -speed;
//                    else
//                        driveMtrPowers[i] = speed;
//                }
//
//                for (int i = 0; i<4; i++) {driveMotors[i].setPower(driveMtrPowers[i]);}
//            }
//        }
//
//        lastangle = heading;

        double powL, powR;

        if(direction.equals("cw") || direction.equals("clockwise")) {
            ccwRotation = false;
            powL = speed;
            powR = -speed;
        }
        else {
            ccwRotation = true;
            powL = -speed;
            powR = speed;
        }



        //TODO make into function
        for(int i = 0; i<4 && !caller.isStopRequested(); i++) {
            if(i%3==0)
                driveMotors[i].setPower(powL);
            else
                driveMotors[i].setPower(powR);
        }

        //telemetry.addData("Rotating:", "%7d, %7s");

        //priming
        refreshAngle();


        while(reachedAngle(angle) && !caller.isStopRequested()) {
            refreshAngle();
            telemetry.addData("heading","%7f %7f", heading, angle);
            telemetry.update();
            //telemetry.addData("Angle", "%7d : %7d", imu.getAngularOrientation(), angle);
        }

        for (int i = 0; i<4 && !caller.isStopRequested(); i++) {
            driveMotors[i].setPower(0);
        }



        //for (int i = 0; i<4; i++) {driveMtrPowers[i] = 0; }
//        for (int i = 0; i<4; i++) {driveMotors[i].setPower(driveMtrPowers[i]);}
//        for (int i = 0; i<4; i++) {driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
//        for (int i = 0; i<4; i++) {driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
    }
    private boolean reachedAngle(double angle) {
        if (ccwRotation)
            return heading < angle;
        else
            return heading > -angle;
    }
    //Currently normalizes angle as well
    public void refreshAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        heading = AngleUnit.DEGREES.normalize(heading);


//        heading = Math.floor(heading);
//        heading = Range.clip(heading, -180.0, 180.0);
    }
    public double getHeading(){
        refreshAngle();
        return heading;
    }


    public double goldPos() {
        return detector.getScreenPosition().x;
    }


    public void getStatus() {
        //TODO implement a status message, possibly useful for the invoker
    }


    public GoldPosition goldLocation() {
        double screenPos = detector.getScreenPosition().x;

        if (screenPos < RIGHT_BOUND && screenPos > LEFT_BOUND //&&
                )
            return GoldPosition.MIDDLE;

        else if (screenPos < LEFT_BOUND && screenPos > 0)
            return GoldPosition.LEFT;

        else if (screenPos > RIGHT_BOUND)
                //&& screenPos < 480
            return GoldPosition.RIGHT;

        else
            return GoldPosition.UNK;
    }
}
