package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")

public class Eva extends OpMode
{
    
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armOne = null;
    private DcMotor armTwo = null;
    private int servoPositionBox = 1;
    private boolean lastValueOfLB = false;
    private Servo clawBox = null;
    private DigitalChannel limitSensor = null;
    
    @Override
    public void init() {
        
        initializeArm();
            
        initializeDriver();
        
        initializeSensors();
        
        initializeClaws();
        
        initializeEncoders();
        
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        
        
        setArcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);

        //ARM PANEL
     
    
        if(gamepad1.a){
            armOne.setTargetPosition(0);
            armTwo.setTargetPosition(0);
            armOne.setPower(1);
            armTwo.setPower(-1);
        }
        else if(gamepad1.y){
            armOne.setTargetPosition(1020);
            armTwo.setTargetPosition(-1020);
            armOne.setPower(1);
            armTwo.setPower(-1);
        
        } else if(gamepad1.b){
            armOne.setTargetPosition(250);
            armTwo.setTargetPosition(-250);
            armOne.setPower(1);
            armTwo.setPower(-1);
        } else if(getLimitSensor()){
            armOne.setPower(0);
            armTwo.setPower(0);
        }
        // CLAW PANEL
        //CLAW BOX
        if(!lastValueOfLB && gamepad1.left_bumper){
            if(servoPositionBox == 0)
                servoPositionBox = 1;
            else
                servoPositionBox = 0;
        }
        lastValueOfLB = gamepad1.left_bumper;
        clawBox.setPosition(servoPositionBox);
        
        if(gamepad1.right_bumper){
          armOne.setPower(-1);
          armTwo.setPower(1);  
          armOne.setTargetPosition(-250);
          armTwo.setTargetPosition(250);
        } 
            
        // RESET ENCODERS
        if(getLimitSensor())
            initializeEncoders();
        
        //DASHBOARD
        dashboard();
        
        if(gamepad1.dpad_up){
            armOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armOne.setPower(0.5);
            armTwo.setPower(-0.5);
        } 
    }

    @Override
    public void stop() {
    }
    
    public void initializeEncoders(){
        armOne.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armTwo.setMode(DcMotor.RunMode.RESET_ENCODERS);
            
        armOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void initializeClaws(){
        clawBox     = hardwareMap.get(Servo.class, "clawBox");    
               
        clawBox.scaleRange(0.2, 1);
    }
    
    public void initializeDriver(){
        leftDrive   = hardwareMap.get(DcMotor.class, "driveLeft");
        rightDrive  = hardwareMap.get(DcMotor.class, "driveRight");
        
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    
    public void initializeSensors(){
        limitSensor = hardwareMap.get(DigitalChannel.class, "limitSensor");
    }
    
    public void initializeArm(){
        armOne      = hardwareMap.get(DcMotor.class, "armTwo");
        armTwo      = hardwareMap.get(DcMotor.class, "armOne");
    }
    
    public void setArcadeDrive(double sp, double r){
        double leftPower;
        double rightPower;

        double drive = sp;
        double turn  = -r * 0.8;
        leftPower    = Range.clip(drive - turn, -1.0, 1.0);
        rightPower   = Range.clip(drive + turn, -1.0, 1.0);
        
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
    
    public void dashboard(){
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("armOne current position: ", armOne.getCurrentPosition());
        telemetry.addData("armTwo current position: ", armTwo.getCurrentPosition());
        telemetry.addData("armOne current POWER: ", armOne.getPower());
        telemetry.addData("armTwo current POWER: ", armTwo.getPower());
        telemetry.addData("Limit sensor: ", getLimitSensor());
        telemetry.addData("Encoder driver right", rightDrive.getCurrentPosition());
        telemetry.addData("Encoder drive left", leftDrive.getCurrentPosition());
        telemetry.addData("Left Joystick Y-axis:",gamepad1.left_stick_y);
        telemetry.addData("Right Joystick X-axis:",gamepad1.right_stick_x);
        telemetry.addData("LB button:",gamepad1.left_bumper);
        telemetry.addData("A button:", gamepad1.a);
        telemetry.addData("Y button:", gamepad1.y);
        telemetry.addData("B button:",gamepad1.b);
        telemetry.addData("RB button:",gamepad1.right_bumper);
        
    }
    
    public boolean getLimitSensor(){
        return !limitSensor.getState();
    }
    
}
