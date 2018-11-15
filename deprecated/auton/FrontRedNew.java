package org.firstinspires.ftc.teamcode.deprecated.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.deprecated.called.AutonHandler;
import org.firstinspires.ftc.teamcode.deprecated.called.HWRobot;

@Autonomous(name="FrontRed Iterative", group="ITA")
//@Disabled
public class FrontRedNew extends OpMode
{
    // Declare OpMode members.
    AutonHandler a = new AutonHandler();
    HWRobot r = new HWRobot();
    String team = "red";
    String area = "eff-front";


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //a.autonInit(telemetry,hardwareMap,this);
        r.makeActive(true);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        a.auton(team,area,telemetry,hardwareMap,true);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        r.makeActive(false);
    }

}