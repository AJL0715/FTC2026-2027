package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class StolenLimelight extends LinearOpMode {


    // get an instance of the "Robot" class.
    private DcMotor leftback;
    private DcMotor rightfront;
    private DcMotor leftfront;
    private DcMotor rightback;
    ElapsedTime runtime;

    Pose2D pos;
    Pose2D START_POS;
    int oldLeftPos;
    int leftencoder;
    int oldRightPos;
    int rightencoder;
    int oldAuxPos;
    int rightfrontpos;
    int auxencoder;
    int leftfrontpos;
    int rightbackpos;
    int state;
    String action;







    double rangeError;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double drivepower = 1;
    double xpow;
    double ypow;
    double turnpow;
    double movetimer= 0;
    double maxpower = 1;
    double pwrmulx = 1;
    double pwrmuly = 1;
    double pwrmulh = 1;
    double ultpower =1;

    double rangeErrorx;
    double rangeErrory;
    double rangeErrorh;



    double throwposx = 100;
    double throwposy = 0;
    double throwposh = 0;
    double inonex = 108;
    double inoney = 27;
    double intwox = 144.79;
    double intwoy = 83;
    double inthrx = 172.02;
    double inthry = 136.66;
    double errorspeed = 15;

    double pidlooptimer;

    double pidlooplastx;

    double pidlooplasty;

    double pidlooplasth;

    double pidxd;

    double pidxi;

    double pidxKp =0.014;

    double pidxKi = 0.0000003; //Affects how quickly the robot stops.

    double pidxKd = 0.000087;

    double pidyd;

    double pidyi;

    double pidyKp = 0.01975;

    double pidyKi = 0.00000058;

    double pidyKd = 0.000082;

    double pidhd;

    double pidhi;

    double pidhKp = 1.4;

    double pidhKi = 0.00005;

    double pidhKd = 0.00005;

    double loopcount = 0;





    double h;
    double x;
    double y;
    int dn1 = leftencoder - oldLeftPos;
    int dn2 = rightencoder - oldRightPos;

    final static double L = 8.405;  // distance between forward encoders
    final static double B = 6.4;  //distance between the center of encoder 1 and 2 and encoder 3
    final static double R = 1.6;  // wheel radius in CM
    final static double N = 2000; // encoder ticks per revolution
    final static double cm_per_tick = 2.0 * Math.PI * R / N;









    private void MECANUM_DRIVE() {




        // Rotate the movement direction counter to the bot's rotation
        double rotX = ypow * Math.cos(-pos.getHeading(AngleUnit.RADIANS)) - xpow * Math.sin(-pos.getHeading(AngleUnit.RADIANS));
        double rotY = ypow * Math.sin(-pos.getHeading(AngleUnit.RADIANS)) + xpow * Math.cos(-pos.getHeading(AngleUnit.RADIANS));

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnpow), 1);
        double leftFrontPower = (rotY + rotX + turnpow) / denominator;
        double leftBackPower = (rotY - rotX + turnpow) / denominator;
        double rightFrontPower = (rotY - rotX - turnpow) / denominator;
        double rightBackPower = (rotY + rotX - turnpow) / denominator;



        // Setting Motor Power
        leftfront.setPower(-leftFrontPower * maxpower*ultpower);
        rightfront.setPower(-rightFrontPower * maxpower*ultpower);
        leftback.setPower(leftBackPower * maxpower*ultpower);
        rightback.setPower(-rightBackPower * maxpower*ultpower);
    }

    private Pose2D odometry() {
        // int dn1;
        // int dn2;
        int dn3;
        double dtheta;
        double dx;
        double dy;


        oldLeftPos = leftencoder;
        oldRightPos = rightencoder;
        oldAuxPos = auxencoder;
        leftencoder = -leftfront.getCurrentPosition();
        rightencoder = -rightback.getCurrentPosition();
        auxencoder = rightfront.getCurrentPosition();
        dn1 = leftencoder - oldLeftPos;
        dn2 = rightencoder - oldRightPos;
        dn3 = auxencoder - oldAuxPos;
        dtheta = cm_per_tick * ((dn1 - dn2) / L);
        dx = cm_per_tick * ((dn1 + dn2) / 2);
        dy = cm_per_tick * (dn3 + ((dn1 - dn2) / 2));
        h += dtheta / 2;
        x += dx * Math.cos(h) - dy * Math.sin(h);
        y += dx * Math.sin(h) + dy * Math.cos(h);




        pos = new Pose2D(DistanceUnit.CM, x, y, AngleUnit.RADIANS, h);

        return pos;
    }

    @Override
    public void runOpMode() {
        //in the initial part of the code we need to set these to 0, but this should not be in the while loop (as that would reset it each time)

        state = 0;

        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        rightback = hardwareMap.get(DcMotor.class, "rightback");




        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        START_POS = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.RADIANS, 0);
        pos = START_POS;
        oldLeftPos = 0;
        oldRightPos = 0;
        oldAuxPos = 0;
        leftencoder = 0;
        rightencoder = 0;
        auxencoder = 0;
        h = 0;
        pidxi = 0;
        pidlooplastx = 0;
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            odometry();
            MECANUM_DRIVE();


            telemetry.addData("X Position", pos.getX(DistanceUnit.CM));
            telemetry.addData("Y Position", pos.getY(DistanceUnit.CM));
            telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Range X", rangeErrorx);
            telemetry.addData("Range Y", rangeErrory);
            telemetry.addData("power x", xpow);
            telemetry.addData("power y", ypow);
            telemetry.addData("power h", turnpow);
            telemetry.addData("Range Heading", rangeErrorh);
            telemetry.addData("Current State", state);
            telemetry.addData("State Action", action);
            telemetry.addData("currentruntime", runtime.milliseconds());
            telemetry.update();

            if(state == 0){

                action = "Moving to throw position";

                rangeErrorx = x-throwposx;

                rangeErrory = y-throwposy;

                rangeErrorh = pos.getHeading(AngleUnit.RADIANS)-Math.toRadians(throwposh);



                if(rangeErrorh > Math.PI){

                    rangeErrorh -= 2 * Math.PI;

                }

                if(rangeErrorh < -Math.PI){

                    rangeErrorh += 2 * Math.PI;

                }





                //PID Loop for each power setting



                //X Direction PID Loop



                //Error is known from above - no changes are needed for the error portion of the code



                //rate of change of the error

                pidxd = (rangeErrorx - pidlooplastx) / (runtime.milliseconds() - pidlooptimer);



                //sum of all error over time

                pidxi = pidxi + (rangeErrorx * (runtime.milliseconds() - pidlooptimer));


                //set the X control power to the motors - value between -1 and 1
                xpow = -(Range.clip((pidxKp * rangeErrorx) + (pidxKi * pidxi) + (pidxKd * pidxd), -1, 1));








                //Y Direction PID Loop

                //Error is known from above - no changes are needed for the error portion of the code

                //rate of change of the error

                pidyd = (rangeErrory - pidlooplasty) / (runtime.milliseconds() - pidlooptimer);

                //sum of all error over time

                pidyi = pidyi + (rangeErrory * (runtime.milliseconds() - pidlooptimer));

                ypow = Range.clip((pidyKp * rangeErrory) + (pidyKi * pidyi) + (pidyKd * pidyd), -1, 1);
                //ypow  = -ypow;

                pidlooplasty = rangeErrory;

                //Heading PID Loop

                pidhd = (rangeErrorh - pidlooplasth) / (runtime.milliseconds() - pidlooptimer);

                //sum of all error over time

                pidhi = pidhi + (rangeErrorh * (runtime.milliseconds() - pidlooptimer));

                turnpow = -(Range.clip((pidhKp * rangeErrorh) + (pidhKi * pidhi) + (pidhKd * pidhd), -1, 1));


                pidlooplasth = rangeErrorh;
                pidlooplastx = rangeErrorx;
                pidlooptimer = runtime.milliseconds();

                if(Math.abs(rangeErrorx) <= 2 && Math.abs(rangeErrory) <= 2 && Math.abs(rangeErrorh) <= Math.toRadians(2)){

                    loopcount = loopcount + 1;

                }

                if(loopcount >= 25){

                    //do the things here to go on to next statee - essentially replacing the timer with a loopcount of loops where it was in the target distances.

                }



            }

        }}
    // todo: write your code here
}