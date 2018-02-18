package org.firstinspires.ftc.teamcode.called;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
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

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.called.RobotValues.ARM_JEWEL_UP;
import static org.firstinspires.ftc.teamcode.called.RobotValues.BLUE_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.called.RobotValues.BLUE_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_BETWEEN_COLUMNS;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_FOR_FULL_ROTATION;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.EXTRUDER_SPEED;
import static org.firstinspires.ftc.teamcode.called.RobotValues.HITTER_JEWEL_NORTH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.HITTER_JEWEL_SOUTH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.NV60_SPEED;
import static org.firstinspires.ftc.teamcode.called.RobotValues.RED_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.called.RobotValues.RED_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SCALE_FACTOR;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SIX_INCHES_NV60;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_FOR_CONVEYORS;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_VUFORIA;

public class HWRobot
{
    //TODO: Make some claw values floats
    /* Declare all motors, sensors, etc. */
    public DcMotor mtrFL, mtrFR, mtrBL, mtrBR, mtrLinear, mtrClawL, mtrClawR;
    public Servo srvJewelHitter, srvJewelArm;
    public ColorSensor sensorColor;
    public BNO055IMU imu;
    public VuforiaLocalizer vuforia;


    // Declare speeds and other vars
    public double powFL = 0;
    public double powFR = 0;
    public double powBL = 0;
    public double powBR = 0;
    public double powLin = 0;
    public double powConL = 0;
    public double powConR = 0;

    int posFL, posFR, posBL, posBR;

    int a = 1;
    int b = 1;
    String mvmtWay;
    public String vuf = "";


    double heading,roll,pitch;

    boolean areWeActive = false;




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

    // countTargets stores the encoder targets for translation

    private int[] countTargets = new int[4];


    /*
    //TODO check if counts per inch actually work
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415) ;
    static final double     DISTANCE_MODIFIER       = 1.414 ;
    */

    // hsv stores the color sensor's hue, sturation, and value

    public float hsv[] = {0F, 0F, 0F};

    /* local OpMode members and objects */
    HardwareMap hwMap           =  null;
    Telemetry telemetry = null;
    public Orientation angles;
    public VuforiaTrackables relicTrackables;
    public VuforiaTrackable relicTemplate;

    /*
    public HWRobot(Telemetry atelemetry, HardwareMap ahwMap) {
        telemetry = atelemetry;
        hwMap = ahwMap;
}
*/

    public void getOpModeData(Telemetry atelemetry, HardwareMap ahwMap) {
        telemetry = atelemetry;
        hwMap = ahwMap;
    }

    // Initialize chosen Hardware interfaces.
    public void init(String hw) {
        switch(hw.toLowerCase()){
            case "motors": case "motor": case "mtr": case "mtrs":
                initMotors();
                break;
            case "servos": case "servo":
                initServos();
                break;
            case "imu":
                initImu();
                break;
            case "vuforia": case "vuf":
                initVuforia();
                break;
            case "sensors":case "sensor":
                initSensors();
                break;
            case "all":
                initMotors();
                //initServos();
                initImu();
                //initVuforia();
                //initSensors();
                break;
            default:
                telemetry.addData("err:", "unknown initialization");
                telemetry.update();
        }
    }

    public void makeActive(boolean setting) {
        areWeActive = setting;
    }

    private void initMotors() {
        // Define and initialize hardware, specifically the DC motors
        mtrFL = hwMap.dcMotor.get("fl_drive");
        mtrFR = hwMap.dcMotor.get("fr_drive");
        mtrBL = hwMap.dcMotor.get("bl_drive");
        mtrBR = hwMap.dcMotor.get("br_drive");

        mtrLinear = hwMap.dcMotor.get("linear_motor");
        mtrClawL = hwMap.dcMotor.get("left_claw");
        mtrClawR = hwMap.dcMotor.get("right_claw");

        // Set directions for motors.
        mtrFL.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);

        //TODO mtr linear might be giving negative encoder values or encoder not plugged in correctly
        mtrLinear.setDirection(DcMotor.Direction.REVERSE);
        mtrClawL.setDirection(DcMotor.Direction.REVERSE);
        mtrClawR.setDirection(DcMotor.Direction.FORWARD);

        //zero power behavior
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtrLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrClawL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrClawR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set power for all motors.
        mtrFL.setPower(powFL);
        mtrFR.setPower(powFR);
        mtrBL.setPower(powBL);
        mtrBR.setPower(powBR);

        mtrLinear.setPower(powLin);
        mtrClawL.setPower(powConL);
        mtrClawR.setPower(powConR);


        // Set all motors to run with given mode
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtrLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrClawL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrClawR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void initServos() {
        //srv fetch + def
        srvJewelArm = hwMap.servo.get("arm_jewel");
        srvJewelHitter = hwMap.servo.get("hitter_jewel");

        //set pos
        srvJewelArm.setPosition(ARM_JEWEL_UP);
        srvJewelHitter.setPosition(HITTER_JEWEL_SOUTH);

    }

    private void initImu() {
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParameters.loggingEnabled      = true;
        gyroParameters.loggingTag          = "IMU";
        imu.initialize(gyroParameters);
        //TODO using angles while opmode is not active could cause problems
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private void initVuforia() {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(
                cameraMonitorViewId
        );

        parameters.vuforiaLicenseKey = vuforiaKey;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    private void initSensors() {
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");
    }
    
    //Sensor Functions
    public void refreshHSV() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsv);
    }

    public String getVuMarkOld(boolean a) {
        String type = "";

        if(a) {
            relicTrackables.activate();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            for(int i = 1; i < 5; i++) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
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
                type = "RIGHT";
            }
        }
        telemetry.addData("VUF:", vuf);
        telemetry.update();
        return type;
    }

    public void getVuMark(boolean active) { // Reads the Vumark and return the cryptobox position, or "UNKNOWN" if it cannot be determined
        if(active) {
            relicTrackables.activate();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            for(int i = 1; i < 5; i++) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            if(vuMark == RelicRecoveryVuMark.UNKNOWN) {
                vuf = "UNKNOWN";
            }
            else if(vuMark == RelicRecoveryVuMark.LEFT) {
                vuf = "LEFT";
            }
            else if(vuMark == RelicRecoveryVuMark.CENTER) {
                vuf = "CENTER";
            }
            else if(vuMark == RelicRecoveryVuMark.RIGHT) {
                vuf = "RIGHT";
            }
        }
        telemetry.addData("VUF:", vuf);
        telemetry.update();
    }

    //TODO moveForCrypto to not strafe; rotate and move
    public void moveForCrypto(String vuforiaRead, boolean active) {
        if(vuforiaRead.equals("LEFT")) {
            telemetry.addData("Left read", "Going for left");
            translate("left", 0.2, COUNTS_BETWEEN_COLUMNS, active);
            //robot.translate("forward", 0.1, COUNTS_PER_INCH * );
        }
        else if(vuforiaRead.equals("CENTER")) {
            telemetry.addData("Center read", "Going for center");
            //translate("fwd", 0.1, COUNTS_PER_INCH * DISTANCE_BETWEEN_COLUMNS, active);
        }
        else if(vuforiaRead.equals("RIGHT")) {
            telemetry.addData("Right read", "Going for right");
            translate("right", 0.2, COUNTS_BETWEEN_COLUMNS, active);
        }
        else if(vuforiaRead.equals("UNKNOWN")) {
            telemetry.addData("Unknown read:", "going for center");
        }
        telemetry.update();
    }

    public void moveForCrypto(boolean active) {
        if(vuf.equals("LEFT")) {
            telemetry.addData("Left read", "Going for left");
            translate("left", 0.2, COUNTS_BETWEEN_COLUMNS, active);
            //robot.translate("forward", 0.1, COUNTS_PER_INCH * );
        }
        else if(vuf.equals("CENTER")) {
            telemetry.addData("Center read", "Going for center");
            //translate("fwd", 0.1, COUNTS_PER_INCH * DISTANCE_BETWEEN_COLUMNS, active);
        }
        else if(vuf.equals("RIGHT")) {
            telemetry.addData("Right read", "Going for right");
            translate("right", 0.2, COUNTS_BETWEEN_COLUMNS, active);
        }
        else if(vuf.equals("UNKNOWN")) {
            telemetry.addData("Unknown read:", "going for center");
        }
        telemetry.update();
    }

    public void adjustPosForCrypto(String generalDirection, String teamColor, boolean front, boolean active) {
        String normalizedGenDir = generalDirection;

        if(front == true) {
            //front
            normalizedGenDir = "fwd";
            //translate to be in between 1st and 2nd column, like how back will start out
            translate(normalizedGenDir, SPEED_TO_VUFORIA, COUNTS_BETWEEN_COLUMNS/2, active);
        }


        if (active) {
            if (teamColor.equals("red")) {
                //ik its redundant dont scream at me its for aes.
                if(vuf.equals("LEFT")) {
                    translate(normalizedGenDir,SPEED_TO_VUFORIA, 2*COUNTS_BETWEEN_COLUMNS, active);
                }
                else if (vuf.equals("CENTER")) {
                    translate(normalizedGenDir,SPEED_TO_VUFORIA, 1*COUNTS_BETWEEN_COLUMNS, active);
                }
                else if (vuf.equals("RIGHT")) {
                    //nothing, already @ position
                }
            }
            else if (teamColor.equals("blue")) {
                if(vuf.equals("RIGHT")) {
                    translate(normalizedGenDir,SPEED_TO_VUFORIA, 2*COUNTS_BETWEEN_COLUMNS, active);
                }
                else if (vuf.equals("CENTER")) {
                    translate(normalizedGenDir,SPEED_TO_VUFORIA, 1*COUNTS_BETWEEN_COLUMNS, active);
                }
                else if (vuf.equals("LEFT")) {
                    //nothing, already @ position
                }
            }
        }
    }
    public void adjustPosForCrypto(String vuforia, String generalDirection, String teamColor, boolean front, boolean active) {
        String normalizedGenDir = generalDirection;

        if(front == true) {
            //front
            normalizedGenDir = "fwd";
            //translate to be in between 1st and 2nd column, like how back will start out
            translate(normalizedGenDir, SPEED_TO_VUFORIA, COUNTS_BETWEEN_COLUMNS/2, active);
        }


        if (active) {
            if (teamColor.equals("red")) {
                //ik its redundant dont scream at me its for aes.
                if(vuforia.equals("LEFT")) {
                    translate(normalizedGenDir,SPEED_TO_VUFORIA, 2*COUNTS_BETWEEN_COLUMNS, active);
                }
                else if (vuforia.equals("CENTER")) {
                    translate(normalizedGenDir,SPEED_TO_VUFORIA, 1*COUNTS_BETWEEN_COLUMNS, active);
                }
                else if (vuforia.equals("RIGHT")) {
                    //nothing, already @ position
                }
            }
            else if (teamColor.equals("blue")) {
                if(vuforia.equals("RIGHT")) {
                    translate(normalizedGenDir,SPEED_TO_VUFORIA, 2*COUNTS_BETWEEN_COLUMNS, active);
                }
                else if (vuforia.equals("CENTER")) {
                    translate(normalizedGenDir,SPEED_TO_VUFORIA, 1*COUNTS_BETWEEN_COLUMNS, active);
                }
                else if (vuforia.equals("LEFT")) {
                    //nothing, already @ position
                }
            }
        }
    }


    public void knockOffJewel(String team, boolean active) {
        if(active) {
            float hue = hsv[0];
            if (team.equals("red")) {
                //if the ball is blue, hit it off
                //assuming the color sensor is reading the most back (southern) one
                if (hue > BLUE_LOWER_LIMIT && hue < BLUE_UPPER_LIMIT) {
                    //do action to hit off
                    telemetry.addData("Color Read:", "Blue");
                    telemetry.addData("Flipping:", "North");
                    telemetry.update();
                    srvJewelHitter.setPosition(HITTER_JEWEL_NORTH);
                }
                //if the ball is red, hit the northern (other) ball
                else if (hue > RED_UPPER_LIMIT || hue < RED_LOWER_LIMIT) {
                    telemetry.addData("Color Read:", "Red");
                    telemetry.addData("Flipping:", "South");
                    telemetry.update();
                    srvJewelHitter.setPosition(HITTER_JEWEL_SOUTH);
                } else {
                    telemetry.addData("Err", "no read");
                    telemetry.update();
                }

            } else if (team.equals("blue")) {
                //if the ball is blue, hit off the other ball
                if (hue > BLUE_LOWER_LIMIT && hue < BLUE_UPPER_LIMIT) {
                    //do action to hit off; ideally use a stick that has a servo on it that moves to hit it off; much easier
                    telemetry.addData("Color Read:", "Blue");
                    telemetry.addData("Flipping:", "South");
                    telemetry.update();
                    srvJewelHitter.setPosition(HITTER_JEWEL_SOUTH);

                }
                //if the ball is red, hit off that ball
                else if (hue > RED_UPPER_LIMIT || hue < RED_LOWER_LIMIT) {
                    telemetry.addData("Color Read:", "Red");
                    telemetry.addData("Flipping:", "North");
                    telemetry.update();
                    srvJewelHitter.setPosition(HITTER_JEWEL_NORTH);
                } else {
                    telemetry.addData("Err", "no read");
                    telemetry.update();
                }

            } else {
                telemetry.addData("unknown team", "");
                telemetry.update();
            }
        }
    }

    public void conveyorStatus(String status) { //
        if(status.toLowerCase().equals("start")) {
            mtrClawL.setPower(SPEED_FOR_CONVEYORS);
            mtrClawR.setPower(SPEED_FOR_CONVEYORS);
        }
        else if (status.toLowerCase().equals("reverse")) {
            mtrClawL.setPower(-SPEED_FOR_CONVEYORS);
            mtrClawR.setPower(-SPEED_FOR_CONVEYORS);
        }
        else {
            mtrClawL.setPower(0);
            mtrClawL.setPower(0);
        }
    }

    //TODO: remove
    public void moveSlide(String dir) {
        int finalCounts = 0;
        if (dir.toLowerCase().equals("up")) {
            finalCounts = SIX_INCHES_NV60;
        } else if(dir.toLowerCase().equals("down")) {
            finalCounts = -SIX_INCHES_NV60;
        }
        mtrLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrLinear.setPower(NV60_SPEED);
        mtrLinear.setTargetPosition(finalCounts);
        while(mtrLinear.isBusy()) {}
        mtrLinear.setPower(0);
    }

    //TODO: umm something this is a very interesting comment
    /*
    public void extrudeGlyphs(String order) {
        if(order.toLowerCase().equals("both")) {
            srvTL.setPower(EXTRUDER_SPEED);
            srvTR.setPower(EXTRUDER_SPEED);
            srvBL.setPower(EXTRUDER_SPEED);
            srvBR.setPower(EXTRUDER_SPEED);
        }
        else if(order.toLowerCase().equals("top")) {
            srvTL.setPower(EXTRUDER_SPEED);
            srvTR.setPower(EXTRUDER_SPEED);
            srvBL.setPower(0);
            srvBR.setPower(0);
        }
        else if(order.toLowerCase().equals("bottom")) {
            srvTL.setPower(0);
            srvTR.setPower(0);
            srvBL.setPower(EXTRUDER_SPEED);
            srvBR.setPower(EXTRUDER_SPEED);
        }
        else if(order.toLowerCase().equals("stop")) {
            srvTL.setPower(0);
            srvTR.setPower(0);
            srvBL.setPower(0);
            srvBR.setPower(0);
        }
    }
    */

    public void resets(String kind) { // Changes conveyor state
        String normKind = kind.toLowerCase();
        if(normKind.equals("reverse")) {
            conveyorStatus("reverse");
        }
        else if(normKind.equals("normal")) {
            conveyorStatus("start");
        }
        else if(normKind.equals("stop")) {
            conveyorStatus("else aka zero out");
        }
    }

    //TODO: remove

    public void driveMotorPolarity(String polarity) {
        if(polarity.toLowerCase().equals("normal")) {
            mtrFL.setDirection(DcMotor.Direction.REVERSE);
            mtrFR.setDirection(DcMotor.Direction.FORWARD);
            mtrBL.setDirection(DcMotor.Direction.REVERSE);
            mtrBR.setDirection(DcMotor.Direction.FORWARD);
        }
        else if (polarity.toLowerCase().equals("reverse")) {
            mtrFL.setDirection(DcMotor.Direction.FORWARD);
            mtrFR.setDirection(DcMotor.Direction.REVERSE);
            mtrBL.setDirection(DcMotor.Direction.FORWARD);
            mtrBR.setDirection(DcMotor.Direction.REVERSE);
        }
    }


   
    //MOTOR FUNCTIONS

    
    //Function to translate on the field (encoders)- currently reconstructed
    //TODO make repitition into own functions

    public void translate(String dir, double speed, double inches, boolean active){ // Directional translation
        decideDirection(dir);

        if(active) {
            mtrChangeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            getNewPositions(inches);

            setDirection();

            mtrSetTargetPos(posFL,posFR,posBL,posBR);

            mtrChangeMode(DcMotor.RunMode.RUN_TO_POSITION);

            mtrSetSpeed(speed);

            while(active && mtrFL.isBusy() && mtrFR.isBusy() && mtrBL.isBusy() && mtrBR.isBusy() && areWeActive) {
                posOutOfFinalTelemetry(countTargets);
            }

            mtrSetSpeed(0);

            mtrChangeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //TODO: Rethink what would happen in rotation
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

            if( super.getClass().)

            while(active && mtrFL.isBusy() && mtrFR.isBusy() && mtrBL.isBusy() && mtrBR.isBusy() && areWeActive) {
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

    //TODO while loop crashes; need to make it RUNWITHENCODERS until heading reaches;; OR BETTER IDEA make global boolean that is opModeIsActive and use that instead of active
    public void rotate(String direction, double speed, double angle, boolean active) {
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
                while(heading > cwNegativeAngle && active && areWeActive) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

                    mtrChangeMode(DcMotor.RunMode.RUN_USING_ENCODER);


                    mtrSetSpeed(speed);
                    telemetry.addData("heading", heading);
                    telemetry.update();

                }

            } else if (direction.equals("counterclockwise") || direction.equals("ccw")) {
                while(heading < angle && active && areWeActive) {

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


    //Rotating by counts depending on the diameter of the robot and portion of circle to turn
    //NO GYRO
    public void rotateByCounts(String direction, double speed, double angle, boolean active) {
        double portionOfCircle = angle / 360;
        int countsToMove = (int) (portionOfCircle * COUNTS_FOR_FULL_ROTATION);
        decideDirection(direction);

        if(active) {
            mtrChangeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            getNewPositions(countsToMove);

            setDirection();

            mtrSetTargetPos(posFL,posFR,posBL,posBR);

            mtrChangeMode(DcMotor.RunMode.RUN_TO_POSITION);

            mtrSetSpeed(speed);

            while(active && mtrFL.isBusy() && mtrFR.isBusy() && mtrBL.isBusy() && mtrBR.isBusy() && areWeActive) {
                posOutOfFinalTelemetry(countTargets);
            }

            mtrSetSpeed(0);

            mtrChangeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    // Changes runmode of drive train DC motors

    public void mtrChangeMode(DcMotor.RunMode mode){
        mtrFL.setMode(mode);
        mtrFR.setMode(mode);
        mtrBL.setMode(mode);
        mtrBR.setMode(mode);
    }

    // Changes DC motor speed

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

    // Decides translation function

    public void decideDirection(String dir) {
        if(dir.equalsIgnoreCase("cw") || dir.equalsIgnoreCase("ccw")){
            if (dir.equals("clockwise") || dir.equals("cw")) {
                a = 1;
                b = -1;
            }
            else if (dir.equals("counterclockwise") || dir.equals("ccw")) {
                a = -1;
                b = 1;
            }
            mvmtWay = "rotation";
        }
        else {
            if(dir.equals("fwd") || dir.equals("forward")) {
                a = 1;
                b = 1;
            }
            else if(dir.equals("bk") || dir.equals("backwards") || dir.equals("back")) {
                a = -1;
                b = -1;
            }
            else if(dir.equals("left")) {
                a = -1;
                b = 1;
            }
            else if (dir.equals("right")) {
                a = 1;
                b = -1;
            }
            mvmtWay = "translation";
        }
    }

    // Gives countTargets a modifier based on translation function

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
        countTargets[0] = mtrFL.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        countTargets[1] = mtrFR.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        countTargets[2] = mtrBL.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        countTargets[3] = mtrBR.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
    }


    private void posOutOfFinalTelemetry(int[] countTargets) {
        telemetry.addData("MtrFL", mtrFL.getCurrentPosition());
        telemetry.addData("MtrFR", mtrFR.getCurrentPosition());
        telemetry.addData("MtrBL", mtrBL.getCurrentPosition());
        telemetry.addData("MtrBR", mtrBR.getCurrentPosition());
        telemetry.update();
    }

    private void mtrSetTargetPos(int posFL, int posFR, int posBL, int posBR) {
        mtrFL.setTargetPosition(posFL);
        mtrFR.setTargetPosition(posFR);
        mtrBL.setTargetPosition(posBL);
        mtrBR.setTargetPosition(posBR);
    }


    // servos
    /*
    public void jewelServoFlip(double position) {
        srvJewel.setPosition(position);
    }
*/
    /*
    public void releaseClaw() {
        srvL.setPosition(0);
        srvR.setPosition(1);
    }*/

    // TODO: remove

    public double getHeading() {
        double heading;
        heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        return heading;
    }

    // TODO: do something with stuff under this comment i want to die

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

