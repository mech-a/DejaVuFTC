package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Robot;

import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.ARM_MOTOR_NAMES;
import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.DRIVE_MOTOR_NAMES;
import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.SENSOR_NAMES;
import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.SERVO_MOTOR_NAMES;
import static org.firstinspires.ftc.teamcode.dependencies.Constants.MARKER_HELD;
import static org.firstinspires.ftc.teamcode.dependencies.Constants.SERVO_LOCKED;
import static org.firstinspires.ftc.teamcode.dependencies.Constants.SERVO_UNLOCKED;
import static org.firstinspires.ftc.teamcode.dependencies.Constants.TELESCOPING_MAX_POSITION;


/**
 * POV Drive mode with encoder driving compatibility w/o while loops
 */

@TeleOp(name="Stem Training Holonomic", group="Competition")
//@Disabled
public class STEMTrainingHolo extends LinearOpMode {
    //slower teleop that keeps track of distances traveled
    //and turn angles to be used in the auton

    // Declare OpMode members.
    Robot r = new Robot(this, Enums.OpModeType.TELEOP);

    int count = 0;
    int LIM = -300;

    double[] g1 = new double[4];
    double[] g2 = new double[4];
    double[] g1Adjusted = new double[4];
    double[] g2Adjusted = new double[4];

    double modifier = 0.55, speedSwitchSlow = 0.25, speedSwitchFast = 1, speedSwitchPow = 1;

    double pow1 = 0, pow2 = 0, pow3 = 0, pow0 = 0;

    final double TRIGGER_DEADZONE = 0.3;

    boolean runSlow = false, runFast = false, runExponential = false;

    //code spec:
    // powLift; 1 for raise, -1 for fall
    // powIntake; 1 for intake, -1 for expel
    // powRotate; 1 for outwards, -1 for inwards
    // powTelescope; 1 for extend, -1 for retract


    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        r.start(hardwareMap, telemetry);
        r.init();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            setGamepads(modifier);

            adjustPowers();

            telemetry.addData("Rotation", r.armMotors[2].getCurrentPosition());

            setPowers();

            telemetry.update();

            sleep(100);
        }
    }

    private void adjustPowers() {
        pow0 = Range.clip(g1[1] + g1[0] + g1[2], -1, 1);
        pow1 = Range.clip(g1[1] - g1[0] - g1[2], -1, 1);
        pow2 = Range.clip(g1[1] + g1[0] - g1[2], -1, 1);
        pow3 = Range.clip(g1[1] - g1[0] + g1[2], -1, 1);
    }

    private void setPowers() {
        r.driveMotors[0].setPower(pow0);
        r.driveMotors[1].setPower(pow1);
        r.driveMotors[2].setPower(pow2);
        r.driveMotors[3].setPower(pow3);
    }

    private void setGamepads(double modifier) {
        //left joystick x, y, right joystick x, y
        g1[0] = gamepad1.left_stick_x * modifier;
        g1[1] = -gamepad1.left_stick_y * modifier;
        g1[2] = gamepad1.right_stick_x * 0.25;
        g1[3] = -gamepad1.right_stick_y * modifier;

        //TODO deadzones
    }

    private void speedSwitch() {
        if(gamepad1.left_bumper) {
            runSlow = true;
            runFast = false;
        }

        else if (gamepad1.right_bumper) {
            runSlow = false;
            runFast = true;
        }

        if(runFast) {
            speedSwitchPow = speedSwitchFast;
        }

        else if(runSlow) {
            speedSwitchPow = speedSwitchSlow;
        }
//        powL = speedSwitchPow * powL;
//        powR = speedSwitchPow * powR;

    }


}
