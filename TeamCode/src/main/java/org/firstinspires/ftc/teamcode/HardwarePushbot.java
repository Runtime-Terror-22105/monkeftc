package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Encoder;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwarePushbot {
    /* Publicly Accessible variables */

    // DriveTrain
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;

    //Intake
    public DcMotor intake = null;
    public Servo intakeControl = null;


    // Slide
    public DcMotor slideLeft =null;
    public DcMotor slideRight =null;
    // Outtake
    public Servo depositLeft = null;
    public Servo depositRight = null;

    public CRServo wheel=null;

    //plane launcher
    public Servo plane = null;

    public Encoder slidesEncoder = null;
    public WebcamName camera = null;

    // local class variables
    private HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    public void init(HardwareMap _hwMap) {
        // save ref to hardware map
        hwMap = _hwMap;

        //initialize the drivetrain motors
        motorFrontLeft  = hwMap.get(DcMotor.class, "motorFrontLeft"); //1
        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight"); //2
        motorBackRight  = hwMap.get(DcMotor.class, "motorBackRight"); //0
        motorBackLeft   = hwMap.get(DcMotor.class, "motorBackLeft"); //3 exp

        // initialize the other motors
        intake          = hwMap.get(DcMotor.class, "intake");

        // initialize the LMEC servo stuff
        // TBD, cad hasn't cadded this yet

        // initialize the other servos
        intakeControl = hwMap.get(Servo.class, "intakeControl");

        //plane
        plane = hwMap.get(Servo.class, "plane");
        // outatake position

        wheel = hwMap.get(CRServo.class, "Wheel"); // servo in outtake
        depositLeft = hwMap.get(Servo.class, "depositLeft"); // servo for rotating the outtake thing
        depositRight = hwMap.get(Servo.class, "depositRight"); // servo for rotating the outtake thing

//
//
//        // slides
        slideLeft = hwMap.get(DcMotor.class, "LeftSlide");
        slideRight = hwMap.get(DcMotor.class, "RightSlide");


        // Camera
        camera = hwMap.get(WebcamName.class, "Webcam 1");

        // Encoders
        slidesEncoder = new Encoder(hwMap.get(DcMotorEx.class, "motorFrontLeft"));
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset the encoder
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // set the motor back to normal
    }
}
