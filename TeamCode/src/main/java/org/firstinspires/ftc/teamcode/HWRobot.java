package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class HWRobot
{
    /* Declare all motors, sensors, etc. */
    public DcMotor mtrFL, mtrFR, mtrBL, mtrBR;
    public Servo srvL, srvR, srvJewel;
    public ColorSensor sensorColor;
    public BNO055IMU imu;
    public VuforiaLocalizer vuforia;




    // Declare speeds and other vars
    public double powFL = 0;
    public double powFR = 0;
    public double powBL = 0;
    public double powBR = 0;

    int a = 1;
    int b = 1;
    String mvmtWay;
    public double ninetyDegreesInInches = 9*3.1415;
    public double turnAroundInInches = 18 *3.1415;
    int posFL, posFR, posBL, posBR;

    double heading,roll,pitch;

    //double localSpeed, localSpeedFL, localSpeedFR, localSpeedBL, localSpeedBR;

    boolean isGyroRotationHappening= false;

    public String vuforiaKey = "AboeStD/////AAAAGaA8AMxAS0isjCVUhD" +
            "5lHuRymY1yqEVbDu1/PRTIEg/9JzZxKpV/P" +
            "39rY/QC64WcjeCtnUDq0jj7yWEPkWZClL" +
            "RC2KVwsjQPUe/mjwl6y51KfIKgSulpN63f" +
            "EYOBdY5ZR4fNswicR46PElRn4NaKqkV6fr9cLS62V8O" +
            "a8ow88oUK3xga8OJkNYf+3VoIQ7dj/RxiKzQCBJRt" +
            "2ZbRIUimTFw4oTC5LJ/NXV2jSD+m7KnW7TCpC7n/7hRxyKR" +
            "mw+JKGoz5kJIfxhliqs1XD3MnD9KN5w6cEwEmg3uYUZ5Bx7bcuO" +
            "N/uEaqifBnmwpdI0Vjklr67kMVYb27z1NsC+OB7moGIPdjhKho6nhwLy9XyMPw";

    //TODO check if when you create an array & edit array value ex arr[0] that is defined as a variable, does editing the arr[0] value change the variable val?
    //it doesnt
    /*private int countTargetFL,countTargetFR,countTargetBL,countTargetBR;
    private int[] countTargets = {
            countTargetFL, countTargetFR, countTargetBL, countTargetBR
    };*/
    private int[] countTargets = new int[4];


    //TODO check if counts per inch actually work
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415) ;
    static final double     DISTANCE_MODIFIER       = 1.414 ;
    
    public float hsv[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;


    /* local OpMode members and objects */
    HardwareMap hwMap           =  null;
    Telemetry telemetry = null;
    public Orientation angles;
    public VuforiaTrackable relicTemplate;


    // Initialize standard Hardware interfaces.
    public void init(HardwareMap ahwMap, DcMotor.RunMode mode, Telemetry atelemetry) {

        // Save reference to Hardware map.
        hwMap = ahwMap;
        telemetry = atelemetry;


        // Define and initialize hardware
        mtrFL = ahwMap.dcMotor.get("fl_drive");
        mtrFR = ahwMap.dcMotor.get("fr_drive");
        mtrBL = ahwMap.dcMotor.get("bl_drive");
        mtrBR = ahwMap.dcMotor.get("br_drive");

        /*
        srvJewel = ahwMap.servo.get("jewel_thing");
        srvL = ahwMap.servo.get("claw_left");
        srvR = ahwMap.servo.get("claw_right");
*/
        imu = ahwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParameters.loggingEnabled      = true;
        gyroParameters.loggingTag          = "IMU";
        imu.initialize(gyroParameters);

        /*
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        double roll = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle);
        double pitch = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle);
        */

        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuforiaParameters.vuforiaLicenseKey = vuforiaKey;

        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary



        sensorColor = ahwMap.get(ColorSensor.class, "sensor_color_distance");

        // Set directions for motors.
        mtrFL.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);

        //zero power behavior
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set power for all motors.
        mtrFL.setPower(powFL);
        mtrFR.setPower(powFR);
        mtrBL.setPower(powBL);
        mtrBR.setPower(powBR);

        // Set all motors to run with given mode
        mtrFL.setMode(mode);
        mtrFR.setMode(mode);
        mtrBL.setMode(mode);
        mtrBR.setMode(mode);

    }
    
    //Sensor Functions
    public void refreshHSV() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsv);
    }

    public String getVuMark(RelicRecoveryVuMark vuMark, boolean active) {
        String type = "";
        if(active) {
            if(vuMark == RelicRecoveryVuMark.UNKNOWN) {
                type = "UNKNOWN";
            }
            else if(vuMark == RelicRecoveryVuMark.LEFT) {
                type = "LEFT";
            }
            else if(vuMark == RelicRecoveryVuMark.CENTER) {
                type = "CENTER";
            }
            else if(vuMark == RelicRecoveryVuMark.RIGHT) {
                type ="RIGHT";
            }
        }
        return type;
    }
    





   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
    //MOTOR FUNCTIONS
    
    
    //Function to translate on the field (encoders)- currently reconstructed
    //TODO make repitition into own functions

    public void translate(String dir, double speed, double inches, boolean active){
        //double inchLocal = Math.floor(inches * DISTANCE_MODIFIER);
        double inchLocal = Math.floor(inches);
        decideDirection(dir);

        if(active) {
            mtrChangeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            getNewPositions(inchLocal);

            setDirection();

            mtrSetTargetPos(posFL,posFR,posBL,posBR);

            mtrChangeMode(DcMotor.RunMode.RUN_TO_POSITION);

            mtrSetSpeed(speed);

            while(active && (mtrFL.isBusy() && mtrFR.isBusy() && mtrBL.isBusy() && mtrBR.isBusy())) {
                posOutOfFinalTelemetry(countTargets);
            }

            mtrSetSpeed(0);

            mtrChangeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //Rethink what would happen in rotation
    //Fix function; look at how counts is modified & when moving backwards
    public void translate(String dir, double speed, int counts, boolean active){
        decideDirection(dir);

        if(active) {
            mtrChangeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            getNewPositions(counts);

            setDirection();

            mtrSetTargetPos(posFL,posFR,posBL,posBR);

            mtrChangeMode(DcMotor.RunMode.RUN_TO_POSITION);

            mtrSetSpeed(speed);

            while(active && mtrFL.isBusy() && mtrFR.isBusy() && mtrBL.isBusy() && mtrBR.isBusy()) {
                posOutOfFinalTelemetry(countTargets);
            }

            mtrSetSpeed(0);

            mtrChangeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    //Function to rotate on the field using encoders
    //Currently does not add to heading therefore if you want to turn 90 degrees when you are already 90
    //You must pass in cw/ccw (dir) and then 180
    //We'll make this addition some time
    public void rotate(String direction, double speed, double angle, boolean active){
        // TODO take out redundancy of setting power using thread
        // from -180 degrees -> 180 degrees
        decideDirection(direction);
        double cwNegativeAngle = -angle;
        isGyroRotationHappening = true;
        mtrChangeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        heading = Math.floor(heading);
        heading = Range.clip(heading, -180.0, 180.0);

        if(active) {
            if (direction.equals("clockwise") || direction.equals("cw")) {
                while(heading > cwNegativeAngle) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

                    mtrChangeMode(DcMotor.RunMode.RUN_USING_ENCODER);


                    mtrSetSpeed(speed);
                    telemetry.addData("heading", heading);
                    telemetry.update();

                }

            } else if (direction.equals("counterclockwise") || direction.equals("ccw")) {
                while(heading < angle) {

                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

                    mtrChangeMode(DcMotor.RunMode.RUN_USING_ENCODER);


                    mtrSetSpeed(speed);
                    telemetry.addData("heading", heading);
                    telemetry.update();

                }

            }

        }
        isGyroRotationHappening = false;
        mtrSetSpeed(0);
        mtrChangeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrChangeMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



    public void mtrChangeMode(DcMotor.RunMode mode){
        mtrFL.setMode(mode);
        mtrFR.setMode(mode);
        mtrBL.setMode(mode);
        mtrBR.setMode(mode);
    }

    public void mtrSetSpeed(double speed){
        if(isGyroRotationHappening == false) {
            mtrFL.setPower(speed);
            mtrFR.setPower(speed);
            mtrBL.setPower(speed);
            mtrBR.setPower(speed);
        }

        else if(isGyroRotationHappening == true) {
            mtrFL.setPower(a * speed);
            mtrFR.setPower(b * speed);
            mtrBL.setPower(a * speed);
            mtrBR.setPower(b * speed);

        }

    }

    private void decideDirection(String dir) {
        if(dir.equalsIgnoreCase("cw") || dir.equalsIgnoreCase("ccw")){
            if (dir == "clockwise" || dir == "cw") {
                a = 1;
                b = -1;
            }
            else if (dir == "counterclockwise" || dir == "ccw") {
                a = -1;
                b = 1;
            }
            mvmtWay = "rotation";
        }
        else {
            if(dir == "fwd" || dir == "forward") {
                a = 1;
                b = 1;
            }
            else if(dir == "bk" || dir == "backwards") {
                a = -1;
                b = -1;
            }
            else if(dir == "left") {
                a = -1;
                b = 1;
            }
            else if (dir == "right") {
                a = 1;
                b = -1;
            }
            mvmtWay = "translation";
        }
    }

    private void setDirection(){
        if(mvmtWay == "translation") {
            posFL = a * countTargets[0];
            posFR = b * countTargets[1];
            posBL = b * countTargets[2];
            posBR = a * countTargets[3];
        }
        /*
        else if(mvmtWay == "rotation" && isGyroRotationHappening) {
            localSpeedFL = a * localSpeed;
            localSpeedFR = b * localSpeed;
            localSpeedBL = a * localSpeed;
            localSpeedBR = b * localSpeed;
        }
        */
        else if(mvmtWay == "rotation") {
            posFL = a * countTargets[0];
            posFR = b * countTargets[1];
            posBL = a * countTargets[2];
            posBR = b * countTargets[3];
        }
    }

    private void getNewPositions(int counts){
        countTargets[0] = mtrFL.getCurrentPosition() + counts;
        countTargets[1] = mtrFR.getCurrentPosition() + counts;
        countTargets[2] = mtrBL.getCurrentPosition() + counts;
        countTargets[3] = mtrBR.getCurrentPosition() + counts;
    }

    private void getNewPositions(double inches){
        countTargets[0] = mtrFL.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * DISTANCE_MODIFIER);
        countTargets[1] = mtrFR.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * DISTANCE_MODIFIER);
        countTargets[2] = mtrBL.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * DISTANCE_MODIFIER);
        countTargets[3] = mtrBR.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * DISTANCE_MODIFIER);
    }


    private void posOutOfFinalTelemetry(int[] countTargets) {
        telemetry.addData("MtrFL", "Pos / Final", mtrFL.getCurrentPosition(), "/", countTargets[0]);
        telemetry.addData("MtrFR", "Pos / Final", mtrFR.getCurrentPosition(), "/", countTargets[1]);
        telemetry.addData("MtrBL", "Pos / Final", mtrBL.getCurrentPosition(), "/", countTargets[2]);
        telemetry.addData("MtrBR", "Pos / Final", mtrBR.getCurrentPosition(), "/", countTargets[3]);
        telemetry.update();
    }

    private void mtrSetTargetPos(int posFL, int posFR, int posBL, int posBR) {
        mtrFL.setTargetPosition(posFL);
        mtrFR.setTargetPosition(posFR);
        mtrBL.setTargetPosition(posBL);
        mtrBR.setTargetPosition(posBR);
    }


    // servos
    public void jewelServoFlip(double position) {
        srvJewel.setPosition(position);
    }



    //private void determineValueOfATranslation

    /* redundant- moved to init
    HWRobot(DcMotor amtrFL, DcMotor amtrFR, DcMotor amtrBL, DcMotor amtrBR, DcMotor.RunMode mode) {
        //Saves a localization
        mtrFL = amtrFL;
        mtrFR = amtrFR;
        mtrBL = amtrBL;
        mtrBR = amtrBR;


    }
    */




}

