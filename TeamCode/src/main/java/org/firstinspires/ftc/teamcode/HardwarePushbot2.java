package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwarePushbot2 {
    /* Publicly Accessible variables */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor intake = null;
    public Servo intakeControl = null;
    public DcMotor hang = null;
    public DcMotor slide = null;
    // public DcMotor vertRightSlide = null;
//    public DcMotor intake=null;
    // public Servo servoFrontLeft = null;
    // public Servo servoFrontRight = null;
    // public Servo servoBackLeft = null;
    // public Servo servoBackRight = null;
    public CRServo tapeServo = null;
    // public Servo extendLeftSlide = null;
    // public Servo extendRightSlide = null;
    public Servo depositLeft = null;
    public Servo depositRight = null;
    public Servo plane = null;

    // public DistanceSensor distSensorLeft   = null;
    // public Rev2mDistanceSensor distSensorCenter = null;
    // public Rev2mDistanceSensor distSensorRight  = null;

    public WebcamName camera = null;

    // local class variables
    private HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    public void init(HardwareMap _hwMap) {
        // save ref to hardware map
        hwMap = _hwMap;

        // initialize the drivetrain motors
//        motorFrontLeft  = hwMap.get(DcMotor.class, "motorFrontLeft");
//        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
//        motorBackRight  = hwMap.get(DcMotor.class, "motorBackRight");
//        motorBackLeft   = hwMap.get(DcMotor.class, "motorBackLeft");

        // initialize the other motors
//        intake          = hwMap.get(DcMotor.class, "intake");
//        hang            = hwMap.get(DcMotor.class, "hang");
//        slide   = hwMap.get(DcMotor.class, "slide");
//         vertRightSlide  = hwMap.get(DcMotor.class, "vertRightSlide");

        // initialize the LMEC servo stuff
        // TBD, cad hasn't cadded this yet

        // initialize the other servos
//        intakeControl = hwMap.get(Servo.class, "intakeControl");
//        plane            = hwMap.get(Servo.class, "plane");
//        tapeServo        = hwMap.crservo.get("tapeServo");
        // tapeServo = hwMap.crservo.get("tapeServo");
        // plane            = hwMap.get(Servo.class, "thing");
//        depositLeft      = hwMap.get(Servo.class, "depositLeft");
//        depositRight     = hwMap.get(Servo.class, "depositRight");
        // extendLeftSlide  = hwMap.get(Servo.class, "extendLeftSlide");
        // extendRightSlide = hwMap.get(Servo.class, "extendRightSlid

        // initialize the distance sensors
        // distSensorLeft   = hwMap.get(DistanceSensor.class, "distSensorLeft");
        // distSensorCenter = hwMap.get(Rev2mDistanceSensor.class, "distSensorCenter");
        // distSensorRight  = hwMap.get(Rev2mDistanceSensor.class, "distSensorRight");
        intake=hwMap.get(DcMotor.class, "intake");
        // Camera
//        camera = hwMap.get(WebcamName.class, "Webcam 1");
    }
}
