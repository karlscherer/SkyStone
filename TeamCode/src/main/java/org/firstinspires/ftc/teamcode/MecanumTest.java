package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class MecanumTest extends LinearOpMode
{
    private DcMotor fr;
    private DcMotor br;
    private DcMotor fl;
    private DcMotor bl;
    @Override
    public void runOpMode(){
        fl=hardwareMap.dcMotor.get("frontleft");
        fr=hardwareMap.dcMotor.get("frontright");
        bl=hardwareMap.dcMotor.get("backleft");
        br=hardwareMap.dcMotor.get("backright");
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        double halfspeed=1;
        double reverse=1;
        boolean apress=false;
        boolean bpress=false;
        stat("HardwareMap Complete");
        waitForStart();
        while(!gamepad1.dpad_up&&opModeIsActive()){
            if(gamepad1.dpad_down){motortest();}

            if(gamepad1.a){
                apress=true;
            }
            else if(apress){
                apress=false;
                if(halfspeed==0.5){
                    halfspeed=1;
                }
                else{
                    halfspeed=0.5;
                }
            }

            if(gamepad1.b){
                bpress=true;
            }
            else if(bpress){
                bpress=false;
                if(reverse==-1){
                    reverse=1;
                }
                else{
                    reverse=-1;
                }
            }
            stat(new String[]{"Halfspeed (A): "+halfspeed,"Running TeleOp","Reverse (B): "+reverse,"DPAD-Up To Exit OpMode","DPAD-Down to run Motor Test"});
            bl.setPower(halfspeed*(reverse*(gamepad1.right_stick_y+gamepad1.left_stick_x)-gamepad1.right_stick_x));
            br.setPower(halfspeed*(reverse*(gamepad1.right_stick_y-gamepad1.left_stick_x)+gamepad1.right_stick_x));
            fl.setPower(halfspeed*(reverse*(-gamepad1.right_stick_y+gamepad1.left_stick_x)+gamepad1.right_stick_x));
            fr.setPower(halfspeed*(reverse*(-gamepad1.right_stick_y-gamepad1.left_stick_x)-gamepad1.right_stick_x));
        }
    }

    //Motor power is set here
    
    public void motortest(){
        int count=0;
        motorstop();
        for(DcMotor M : new DcMotor[]{fl,bl,fr,br}){
            stat(new String[] {"Running Motor Test","Running Motor "+count});
            M.setDirection(DcMotor.Direction.FORWARD);
            M.setPower(1);
            sleep(1000);
            M.setPower(-1);
            sleep(1000);
            M.setPower(0);
            sleep(1000);
            count++;
        }
    }

    public void stat(String[] in){
        for(String a : in){
            telemetry.addData("Status",a);
        }
        telemetry.update();
    }

    public void stat(String in){
        telemetry.addData("Status",in);
        telemetry.update();
    }
    public void motorstop(){
        for(DcMotor M : new DcMotor[]{fl,bl,fr,br}){
            M.setPower(0);
        }
    }
}

