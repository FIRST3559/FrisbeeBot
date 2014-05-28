/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*1.Kyle Fischer
 *2.Griffin Park
 *3.Andy B 
 *4.Michelle Young
 *5.Ryan Bosler
 */
package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick; 
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot; 
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.KinectStick;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Accelerometer;
import edu.wpi.first.wpilibj.Relay;
import java.util.TimerTask;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport; 
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.networktables.NetworkTableKeyNotDefined;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.TableKeyNotDefinedException;

//import edu.wpi.first.wpilibj.*;//Imports all classes for edu.wpi.first.wpilibj (makes code neater)
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Team3559 extends SimpleRobot {
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    RobotDrive drive = new RobotDrive (1,2); //Drive  
    Joystick LeftStick = new Joystick(1);//left drive stick
    Joystick RightStick = new Joystick (2);//right drive stick
    Joystick ThirdStick = new Joystick (3);//arm joystick
    double top = .5;
   // DigitalInput tiltSwitch = new DigitalInput(1);
    double bottom = .5;
    int Greset = 0;
    KinectStick leftHand = new KinectStick(1);
    KinectStick rightHand = new KinectStick(2);
    KinectStick leftKnee = new KinectStick(3);
   // AnalogChannel rangefinder = new AnalogChannel(8);
//    Relay myRelay = new Relay(1); // relay 1
    Gyro Gmain = new Gyro(1);
//    int SpikeState = 0;
    DigitalInput ETLS = new DigitalInput(3);                    // Digital Input for the max height limit switch
    DigitalInput EMLS = new DigitalInput(2);                    // Digital Input for the min height limit switch            
    DigitalInput EBLS = new DigitalInput(1);                   // Digital Input for the min second limit switch
    Victor Eve1 = new Victor(3);                  // Digital Input for the min height limit switch            
    DigitalInput TBLS = new DigitalInput(4);      //Tilting Back (Ball loading side) Limit Switch
    DigitalInput TFLS = new DigitalInput(5);                    // Tilting Front (Ball Shooting side) Limit Switch            
    Victor Eve2 = new Victor(4);
//    boolean EVE1STAT;
//    boolean EVE2STAT;
    
    Talon Arm = new Talon(7);
//    Gyro Gmain = new Gyro(3);
    DigitalInput ABLS = new DigitalInput(6);      //
    DigitalInput AFLS = new DigitalInput(7); 
    int Setup = 0;
    AnalogChannel sonar = new AnalogChannel(2);

    Servo ArmServo = new Servo(6);
//    Servo Cam1 = new Servo(7);
 //    int GBalance;
      Relay Tilt = new Relay(1);
    int DriveMode = 0;
    Jaguar Launcher = new Jaguar (5);
      // Command autonomousCommand;
      //SendableChooser autoChooser;
     
    
    
    
    /*  
    private Drivetrain drivetrain;
    private Sensors sensors;
    private CameraFHS camera;
    
    private Dashboard dashboard;
    private Watchdog watchdog;*/

//  private ImageAnalysis analysis;
    //private AxisCamera axis;
    

//    public void robotInit() {
//        autoChooser = new SendableChooser();
//        autoChooser.addDefault("Testing",new KinectAuto() );
//        autoChooser.addObject("Get 2 Balls", new twoBalls());
//        autoChooser.addObject("Auto Shoot", new AutoShoot());
//        SmartDashboard.putData("Autonomous mode chooser", autoChooser);
//        
//
//    }


//    public void centerOnFirstTarget() throws AxisCameraException, NIVisionException
//    {
//        analysis.updateImage();
//        ParticleAnalysisReport[] report = analysis.findRectangles();
//        double xNormal;
//        if(report.length >= 1)
//        {
//            xNormal = report[0].center_mass_x_normalized;
//            System.out.println("centerOnFirstTarget" + xNormal);
//            //drivetrain.frontLeftSet(xNormal*0.3);
//            //drivetrain.frontRightSet(xNormal*0.3); 
////            drivetrain.rearLeftSet(xNormal*0.3); 
////            drivetrain.rearRightSet(xNormal*0.3);  
//        }
//        else
//        {
//            System.out.println("No Valid Targets Found!");
//            //drivetrain.frontLeftSet(0.0); 
//            //drivetrain.frontRightSet(0.0);
////            drivetrain.rearLeftSet(0.0);  
////            drivetrain.rearRightSet(0.0);   
//        }
//    }

    

    

    public void UpdateDash(){
            SmartDashboard.putBoolean("Precision Mode", ((LeftStick.getRawButton(1) == true || RightStick.getRawButton(1) == true )));
            SmartDashboard.putBoolean("Lower", EBLS.get());
            SmartDashboard.putBoolean("Middle", EMLS.get());
            SmartDashboard.putBoolean("Top", ETLS.get());
            SmartDashboard.putDouble("Shooter", (((-ThirdStick.getZ()+1))/2));
            SmartDashboard.putBoolean("Button", ThirdStick.getRawButton(2));
            SmartDashboard.putBoolean("Back of Arm", ABLS.get());
            SmartDashboard.putBoolean("Front of Arm", AFLS.get());
            
     //       SmartDashboard.putBoolean("Tilt", tiltSwitch.get());
//            autoChooser = new SendableChooser();
//            autoChooser.addDefault("Kinect", new DoSomeAutonomousStuff());
//            autoChooser.addObject("Get 2 Balls", Elevator());
//            autoChooser.addObject("Auto Shoot");
//            autoChooser.addObject("Defense", );
//            autoChooser.
//            SmartDashboard.putData("Autonomous mode chooser", autoChooser);
//            SmartDashboard.getDouble("Input # of seconds for driving backwards in Autonomous", drive.());
    

//            SmartDashboard.putBoolean(" Elevator 1", EVE1STAT );
//            SmartDashboard.putBoolean(" Elevator 2", EVE2STAT );
           // SmartDashboard.putDouble("ArmServo Angle", ArmServo.getAngle());
    }
public void CamServo(){
//   double UD = Cam1.getAngle();
////   double LR = Cam2.getAngle();
//    if (LeftStick.getRawButton(2) == true){
//        Cam1.set(UD - 1);
//    }
//    if (LeftStick.getRawButton(3) == true){
//        Cam1.set(UD + 1);
//    }
//    if ((LeftStick.getRawButton(2) == false) && (LeftStick.getRawButton(3) == false)){
//        Cam1.set(UD);
//    }

}
        public void Driving(){
//           double Left = (LeftStick.getY(Hand.kLeft)* (0.5));
//            double Right = (RightStick.getY(Hand.kLeft)* (0.5));
//            double RightNorm = (RightStick.getY(Hand.kLeft));
//            double LeftNorm = (LeftStick.getY(Hand.kLeft));
//            drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);//Makes robot left motor move foward when joystick is pushed foward
//            drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);//Makes robot right motor move foward when joystick is pushed foward
//            if ((LeftStick.getRawButton(1) == true) || (RightStick.getRawButton(1) == true)){
//                drive.tankDrive(Left, Right);
//            }else {
//                
//            if ((LeftStick.getRawButton(3)== true)){
//                if (DriveMode == 1){
//                    DriveMode = 0;
//                }else{
//                    DriveMode = 1;
//                }
//            }
//            if (DriveMode == 1){
//                if ((LeftStick.getButton(Joystick.ButtonType.kTrigger) == true)&& (RightStick.getButton(Joystick.ButtonType.kTrigger) == true)){
//                      drive.tankDrive(-Left, -Right);
//                }else{
//                      drive.tankDrive(-LeftNorm, -RightNorm);
//                }
//            }
//            if (DriveMode == 0){
//                if ((LeftStick.getButton(Joystick.ButtonType.kTrigger) == true)&& (RightStick.getButton(Joystick.ButtonType.kTrigger) == true)){
//                  drive.tankDrive(Left, Right);
//                }else{
//                  drive.tankDrive(LeftStick, RightStick);
//                }
//            }
//            
//               
//            
//                
//            }
//        
//    }
//    public void Elevator(){
//        if (Eve1.get()>= 0.0){
//            EVE1STAT = true;
//        }else{
//            EVE1STAT = false;
//        }
//        if (Eve2.get()>= 0.0){
//            EVE2STAT = true;
//        }else{
//            EVE2STAT = false;
//        }
//        if((ThirdStick.getTrigger(Hand.kLeft) == true)/* && (EMLS.get() == true)*/){
//                 Eve1.set(1);
//                 Eve2.set(1);
//              
//                 
//             }else {
//             if((EBLS.get() == false) && (EMLS.get() == false) && (ETLS.get() == false)){
//                 Eve1.set(1);
//                   
//                 }
//                 if((EBLS.get() == true) && (EMLS.get() == false) && (ETLS.get() == false)){
//                     Eve1.set(1.0);
//                     Eve2.set(1.0);
//                 }
//                 if((EBLS.get() == false) && (EMLS.get() == true) && (ETLS.get()  == false)){   
//                 Eve2.set(0.0);
//                 Eve1.set(1.0);
//                 }
//                 if(((EBLS.get()  == true) && (EMLS.get()  == true) && (ETLS.get()  == false)) || (ThirdStick.getTrigger(Hand.kLeft) == true))  {
//                     Eve1.set(1.0);
//                     Eve2.set(1.0);
//                 }
//                 if((EBLS.get() == false) && (EMLS.get() == true) && (ETLS.get() == true)){
//                     Eve2.set(0.0);
//                 Eve1.set(1.0);
//                 }
//                 if((EBLS.get() == true) && (EMLS.get() == true) && (ETLS.get() == true)){
//                     Eve2.set(0.0);
//                 Eve1.set(0.0);
//                }
//         }
//    }
//    public void Limiting(){
//        if ((TBLS.get() == true)){
//    Tilt.set(Relay.Value.kOff);
//}
//if ((TFLS.get() == true)){
//    Tilt.set(Relay.Value.kOff);
//}
//    if ((ThirdStick.getRawButton(2) == true ) && (TBLS.get() == false)){
//        Tilt.setDirection(Relay.Direction.kReverse);
//        Tilt.set(Relay.Value.kOn);
//    }
//    if ((ThirdStick.getRawButton(2) == true ) && (TFLS.get() == false)){
//        Tilt.setDirection(Relay.Direction.kForward);
//        Tilt.set(Relay.Value.kOn);
//    }
//
//if ((ABLS.get() == true) && ((ThirdStick.getX(Hand.kLeft)) <= 0) == true){
//    Arm.set(0.0);
//}
//if ((AFLS.get() == true) && ((ThirdStick.getX(Hand.kLeft)) >= 0) == true){
//    Arm.set(0.0);
//}
//if ((ABLS.get() == false) && (AFLS.get() == false)){
//    Arm.set(ThirdStick.getX());
//}
//    }
//    public void centerOnFirstTarget() throws AxisCameraException, NIVisionException{
//        analysis.updateImage();
//        ParticleAnalysisReport[] report = analysis.findRectangles();
//        double xNormal;
//        if(report.length >= 1)
//        {
//            xNormal = report[0].center_mass_x_normalized;
//            System.out.println("centerOnFirstTarget" + xNormal);
//            //drivetrain.frontLeftSet(xNormal*0.3);
//            //drivetrain.frontRightSet(xNormal*0.3); 
////            drivetrain.rearLeftSet(xNormal*0.3); 
////            drivetrain.rearRightSet(xNormal*0.3);  
//        }
//        else
//        {
//            System.out.println("No Valid Targets Found!");
//            //drivetrain.frontLeftSet(0.0); 
//            //drivetrain.frontRightSet(0.0);
////            drivetrain.rearLeftSet(0.0);  
////            drivetrain.rearRightSet(0.0);   
//        }
    }

//    public void ImageTracking () throws NIVisionException, AxisCameraException{
//      axis.getImage();
//      System.out.println(axis.getColorLevel());
//      
//    }



    public void Gyro(){
        if (LeftStick.getRawButton(10) == true ){
        Gmain.reset();   
        }
        Gmain.setSensitivity(.007);
        
        
        SmartDashboard.putDouble("Gyro", Gmain.getAngle());
//        SmartDashboard.putDouble("Sonar Voltage", ((sonar.getVoltage())/0.0097)/12);
    }
    public void autonomous() {
        Gmain.reset();
        
        // THIS IS WHERE THE STUFF FROM THE CAGE MATCH IS
//        drive.tankDrive(1, 1);
        Timer.delay (2.5);
        Launcher.set(-((-ThirdStick.getZ()+1))/2);
//        drive.tankDrive(.5,.5);
//        Timer.delay(4.00);
        drive.tankDrive(0.0, 0.0);
        Eve1.set(1.0);
        Timer.delay(8.0);
        Eve1.set(0.0);
        Launcher.set(0.0);
//        int DFTS = 0;
//        int Preaim = 0;
//        int Kin = 0;
//        int pass = 0;
//        System.out.println("I'm in autonomous mode now!");
//        int tat = 0;
//        Tilt.setDirection(Relay.Direction.kForward);
//        double LaunchSpeed = (((-ThirdStick.getZ()+1))/2); // Launcher control by trottle
//        
//        while (isEnabled()) {
//            
//            UpdateDash();
//            
//            Elevator();
//            
//           
////                drive.tankDrive(1, 1);
////                Timer.delay(4.0);
////                drive.tankDrive(0, 0);
////                Arm.set(1);
////                Eve1.set(1.0);
////                Eve2.set(1.0);
////           
////                Arm.set(rightHand.getTwist());
////                drive.tankDrive(leftHand, rightHand);
////                Launcher.set(leftKnee.getY());
//           
//        
//            while (DFTS == 0){
//                
//            drive.tankDrive(1, 1);
//            Timer.delay(2.5);
//            drive.tankDrive(0.0, 0.0);
//            Eve1.set(1.0);
//            Eve2.set(1.0);
//            Launcher.set(LaunchSpeed);
//            tat = 1;
//            DFTS = 1;
//            }
//            while (Preaim == 0){
//        THIS IS WHERE THE OLD STUFF FOR THE BOILERMAKER REGIONAL IS
//        Launcher.set(-((-ThirdStick.getZ()+1))/2);
//        UpdateDash();
//        Timer.delay(4.5);
//              Eve1.set(1.0);
//            Eve2.set(-1.0);
//            
//            Timer.delay(10.0);
//            Eve1.set(0.0);
//            Eve2.set(0.0);
//            Launcher.set(0.0);
//            }
//            
//            while (Kin == 0){
//                drive.tankDrive(leftHand.getY(Hand.kLeft), rightHand.getY(Hand.kRight));
//                Launcher.set(leftHand.getZ(Hand.kLeft));
//                Eve1.set(rightHand.getZ(Hand.kLeft));
//                Eve2.set(rightHand.getZ(Hand.kLeft));
//                if (leftHand.getRawButton(1) == true){
//                Tilt.set(Relay.Value.kOn);    
//            }else{
//                  Tilt.set(Relay.Value.kOff);  
//                }
//            }
//            while (tat == 1){
//            drive.tankDrive(1, -1);
//            Timer.delay(1.5);
//            }
//            while (tat == 2){
//            drive.tankDrive(.75,.75);
//            Tilt.set(Relay.Value.kOn);
//            }
//            
//            while (pass == 0){
//                Eve1.set(-1);
//                Eve2.set(-1);
//                Timer.delay(9.0);
//                tat = 1;
//            }
            

//            drive.tankDrive(leftHand, rightHand);
//            Launcher.set(leftKnee.getY());

            
//        try
//        {
//            centerOnFirstTarget();
//        }
//        catch (AxisCameraException ex)
//        {
//            System.out.println("Error running AxisCameraException");
//            ex.printStackTrace();
//        }
//        catch (NIVisionException ex)
//        {
//            System.out.println("Error running NIVisionException");
//            ex.printStackTrace();
//        }
        
        /*for (int i = 0; i < 4; i++) { 
            drive.drive(0.5, 0.0); // drive 50% fwd 0% turn 
            Timer.delay(2.0); // wait 2 seconds 
            drive.drive(0.0, 0.75); // drive 0% fwd, 75% turn 
        } 
        drive.drive(0.0, 0.0); // drive 0% forward, 0% turn} 
    }*/


    /**
     * This function is called once each time the robot enters operator control.
     */
            
            
//        }
    }//needs to be fixed
    public void operatorControl() {
//        GMain.setSensitivity(.007);
//        Accel.setSensitivity(.01);
        
        Gmain.reset();        
        System.out.println("I'm in TeleOp Mode now!");
        
        while (true && isOperatorControl() && isEnabled()) { // loop until change 
            UpdateDash();
            Driving();
            Gyro();
            drive.tankDrive(-(LeftStick.getY(Hand.kLeft)), -(RightStick.getY(Hand.kLeft)));
            if (RightStick.getRawButton(3) == true){
                Tilt.setDirection(Relay.Direction.kReverse);
                Tilt.set(Relay.Value.kOn);
                
            }
            if (RightStick.getRawButton(2) == true){
                Tilt.setDirection(Relay.Direction.kForward);
                Tilt.set(Relay.Value.kOn);
                
            }
            if ((RightStick.getRawButton(3) == false) && (RightStick.getRawButton(2) == false)){
                
                Tilt.set(Relay.Value.kOff);
                
            }
            if (ThirdStick.getRawButton(3) == true){
                Eve1.set(1.0);
            }
            if (ThirdStick.getRawButton(2) == true){
                Eve1.set(-1.0);
            }
            if (ThirdStick.getRawButton(4) == true){
                Eve1.set(0);
            }
            
            
//            if (ThirdStick.getRawButton(5) == true){
//                Eve2.set(0.0);
//            }
//            if (ThirdStick.getRawButton(1) == true){
//                Launcher.set(-((-ThirdStick.getZ()+1))/2);
//            }else{
//                Launcher.set(0.0);
//            }
            if (ThirdStick.getRawButton(1) == true){
                Launcher.set(-((-ThirdStick.getZ()+1))/2);
            }else{
                Launcher.set(0.0);
            }
            
           // Gyro();
//            try {
//                ImageTracking();
//            } catch (NIVisionException ex) {
//                ex.printStackTrace();
//            } catch (AxisCameraException ex) {
//                ex.printStackTrace();
//            }
             // Launcher control by trottle
            //System.out.println("Volts from the sonar are " + ((((sonar.getVoltage())*5000)/1024)*10 /98));
            
            
//            Launcher.set(((-ThirdStick.getZ()+1))/2); // Launcher control by trottle
//            System.out.println("Volts from the sonar are " + ((((sonar.getVoltage())*5000)/1024)*10 /98));
            
            
//            if ((ThirdStick.getRawButton(9)== true)) {
//                ArmServo.setAngle(120);
//            }
//            
//            if ((ThirdStick.getRawButton(8)== true)) {
//                ArmServo.setAngle(0);
//            }
//            
	NetworkTable server = NetworkTable.getTable("SmartDashboard");
	try
	{
            System.out.println(server.getNumber("Display", 0.0));
	}
	catch (TableKeyNotDefinedException exp)
	{
	}       

            
            Timer.delay(0.005);
        }  // end of while loop from Enable/Disable
    }
           
            
            
}
    




 