/*
 Our 2019 Robot code. Written by Max Morningstar and [Other contributors here] with guidance from Jeff Gibbons
 */


#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>
#include <sstream>

using std::cout;
using std::endl;
#define DRIVEMETHODJOYSTICK
class Robot : public frc::TimedRobot {
 private:

  static void VisionThread() {
     // This function executes as a separate thread, to take 640x480 pixel
     // video frames from the USB video camera, change to grayscale, and
     // send to the DriverStation. It is documented here:
     // https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/
     //         669166-using-the-cameraserver-on-the-roborio
     cs::UsbCamera camera =
                 frc::CameraServer::GetInstance()->StartAutomaticCapture();
     // camera.SetResolution(640, 480);
     camera.SetResolution(160, 120);
     cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
     cs::CvSource outputStreamStd =
              frc::CameraServer::GetInstance()->PutVideo("Gray", 160, 120);
              frc::CameraServer::GetInstance()->GetVideo();
     cv::Mat source;
     cv::Mat output;
     while(true) {
        usleep(1000);
        if ( cvSink.GrabFrame(source) )
        {
           cvtColor(source, output, cv::COLOR_BGR2GRAY);
           outputStreamStd.PutFrame(output);
        }
     }
    }
    
        // GetAllVariables() retrieves all variable values from sensors,
        // encoders, the limelight, etc.  It should be called at the beginning
        // of every 20-millisecond tick.
 /* void GetAllVariables()  {
    limev = limenttable->GetNumber("tv",0.0);
    limex = limenttable->GetNumber("tx",0.0);
    limea = limenttable->GetNumber("ta",0.0);
    limey = limenttable->GetNumber("ty",0.0);
    limes = limenttable->GetNumber("ts",0.0);
    }
        // DriveToTarget() drives autonomously towards a vision target.
        // It returns true if the limelight data is valid, false otherwise.
  bool DriveToTarget()  {
    bool returnVal = true;
    static int  iCallCount = 0;

    iCallCount++;

    
    if ( 1 == limev )  {                         // if limelight data is valid
      double autoDriveSpeed;
               // limea is the area of the target seen by the limelight camera
               // and is in percent (between 0 and 100) of the whole screen area.
      if ( 40 < limea )  {         // if we're really close...
          autoDriveSpeed = 0.20;   //   go slow
      } else if ( 25 < limea ) {   // if we're a little farther...
          autoDriveSpeed = 0.25;   //   go a little faster
      } else if ( 10 < limea ) {   // if we're farther still...
          autoDriveSpeed = 0.30;   //   go a little faster still
      } else {                     // else we must be really far...
          autoDriveSpeed = 0.35;   //   go as fast as we dare
      }
                             // LATER: May want to modify autoDriveSpeed depending
                             // on the distance from the target determined
                             // by sonar transducers.

    // May have to add/subtract a constant from limex here, to account
    // for the offset of the camera away from the centerline of the robot.
      if ( endShift ) {
          m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );
      } else if ( 0 <= limex )  {
                             // if target to the right, turn towards the right
        m_drive.CurvatureDrive( -autoDriveSpeed, -(limex/30.0), 1 );
      } else if ( limex < 0 ) {
                               // if target to the left, turn towards the left
        m_drive.CurvatureDrive( -autoDriveSpeed, -(limex/30.0), 1 );
      } else {
        m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );
      }
      
    } else {                      // else limelight data is not valid any more
      // should we continue forward here?
      returnVal = false;
    }
    if ( 0 == iCallCount%100 )  {
        cout << "lime: " << limev << ":" << limex << "/" << limey << ", " << limea << "." << endl;
    }
    return returnVal;
  }
//*/
 public:
  void RobotInit() {
                                // start a thread processing USB camera images
      std::thread visionThread(VisionThread);
      visionThread.detach();
      m_motorLSSlave1.Follow(m_motorLSMaster);
      m_motorRSSlave1.Follow(m_motorRSMaster);

      dumpValve.SetAngle(0);

// motion magic elevator motor
          /* Configure Sensor Source for Pirmary PID */
   m_motorLSPTO.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

    /**
     * Configure Talon SRX Output and Sesnor direction accordingly
     * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
     * Phase sensor to have positive increment when driving Talon Forward (Green LED)
     */
    m_motorLSPTO.SetSensorPhase(false);
    m_motorLSPTO.SetInverted(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    m_motorLSPTO.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_motorLSPTO.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    m_motorLSPTO.ConfigNominalOutputForward(0, 10);
    m_motorLSPTO.ConfigNominalOutputReverse(0, 10);
    m_motorLSPTO.ConfigPeakOutputForward(1, 10);
    m_motorLSPTO.ConfigPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    m_motorLSPTO.SelectProfileSlot(0, 0);
    m_motorLSPTO.Config_kF(0, 0.3, 10);
    m_motorLSPTO.Config_kP(0, 5, 10);
    m_motorLSPTO.Config_kI(0, 0.0, 10);
    m_motorLSPTO.Config_kD(0, 0.0, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    m_motorLSPTO.ConfigMotionCruiseVelocity(1500, 10);
    m_motorLSPTO.ConfigMotionAcceleration(1500, 10);

//motion magic arm motor
          /* Configure Sensor Source for Pirmary PID */
    //m_armMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

    /**
     * Configure Talon SRX Output and Sesnor direction accordingly
     * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
     * Phase sensor to have positive increment when driving Talon Forward (Green LED)
     */
    //m_armMotor.SetSensorPhase(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    //m_armMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    //m_armMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    //m_armMotor.ConfigNominalOutputForward(0, 10);
    //m_armMotor.ConfigNominalOutputReverse(0, 10);
    //m_armMotor.ConfigPeakOutputForward(1, 10);
    //m_armMotor.ConfigPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    //m_armMotor.SelectProfileSlot(0, 0);
    //m_armMotor.Config_kF(0, 0.3, 10);
    //m_armMotor.Config_kP(0, 0.1, 10);
    //m_armMotor.Config_kI(0, 0.0, 10);
    //m_armMotor.Config_kD(0, 0.0, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    //m_armMotor.ConfigMotionCruiseVelocity(1500, 10);
    //m_armMotor.ConfigMotionAcceleration(1500, 10);
  }
  void AutonomousInit() override {
    m_motorLSSlave1.Follow(m_motorLSMaster);
    m_motorRSSlave1.Follow(m_motorRSMaster);

  }
  void AutonomousPeriodic() override {
   /* GetAllVariables();
    if ( ( m_stick.GetRawButton(3) ) &&         // If driver is pressing the
         ( 1  == limev )                )  {    // "drivetotarget" button and
                                                // the limelight has a target,
        DriveToTarget();         // then autonomously drive towards the target
    } else { */
        if ( m_stick.GetTrigger() ) {
            m_shiftingSolenoid.Set(true); // hi gear
        } else {
            m_shiftingSolenoid.Set(false); // lo gear
        }
        if ( endShift ) {
            m_drive.CurvatureDrive( m_stick.GetY(), 0, 0 );
        }
    //} 
    if ( hatchOpen ) {
        if ( !wantHatchOpen ) {
            hatchOpen = false;
            m_hatchSolenoid.Set(false); // hatch close
        }
    } else {
        if ( wantHatchOpen ) {
            hatchOpen = true;
            m_hatchSolenoid.Set(true); // hatch open
        }
    }
    if ( brakeEngaged ) {
        if ( !wantBrakeEngaged ) {
            brakeEngaged = false;
            m_brakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // Brake on
        }
    } else {
        if ( wantBrakeEngaged ) {
            brakeEngaged = true;
            m_brakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // Brake off
        }
    }
    if ( m_console.GetRawButton(6) ) {
            wantBrakeEngaged = false;
    } else if ( m_console.GetRawButton(7) ) {
            wantBrakeEngaged = true;
    } 
    m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetZ() );
  }
  void TeleopInit() override {
    m_motorLSSlave1.Follow(m_motorLSMaster);
    m_motorRSSlave1.Follow(m_motorRSMaster);

  }
  void TeleopPeriodic() override {

    //GetAllVariables();

    /*switch( elevarmPosition ) { //at the end of each position move, set wantBrakeEngage = true;
        default: wantBrakeEngaged = false; //same as starting configuration (case 0)
                elevatorTarget = 0.0; //???
                armTarget = 0.0; //???
                if ( elevatorEncoder > 10  ) {
                    motionMagicElevator = true; //NOTE THIS
                } else {
                    motionMagicElevator = false;
                    wantBrakeEngaged = true;
                }
                if ( armEncoder > targetPosArm + (targetPosArm*.05) || armEncoder < targetPosArm - (targetPosArm*.05) ) {
                    motionMagicArm = true; 
                } else {
                    motionMagicArm = false;
                }
                break;
        case 0: wantBrakeEngaged = false; //starting configuration
                elevatorTarget = 0.0;
                armTarget = 0.0;
                if ( elevatorEncoder > 10 ) {
                    motionMagicElevator = true;
                    //m_armMotor.Set(ControlMode::PercentOutput, 1.0 ); 
                } else {
                    motionMagicElevator = false;
                    wantBrakeEngaged = true;
                    //m_armMotor.Set(ControlMode::PercentOutput, -1.0 ); 
                }
                if ( armEncoder > targetPosArm + (targetPosArm*.05) || armEncoder < targetPosArm - (targetPosArm*.05) ) {
                    motionMagicArm = true; 
                } else {
                    motionMagicArm = false;
                }
                break;
        case 1: wantBrakeEngaged = false; //Low hatch
                elevatorTarget = 1.0 ;
                armTarget = 0.0;
                if ( elevatorEncoder > 10 ) {
                    motionMagicElevator = true; 
                } else {
                    motionMagicElevator = false;
                    wantBrakeEngaged = true;
                }
                if ( armEncoder > targetPosArm + (targetPosArm*.05) || armEncoder < targetPosArm - (targetPosArm*.05) ) {
                    motionMagicArm = true; 
                } else {
                    motionMagicArm = false;
                }               
                break;
        case 2: wantBrakeEngaged = false; //Middle Hatch
                elevatorTarget = 0.0;
                armTarget = 0.0;
                if ( elevatorEncoder > targetPosElev + (targetPosElev*.05) || elevatorEncoder < targetPosElev - (targetPosElev*.05) ) {
                    motionMagicElevator = true; 
                } else {
                    motionMagicElevator = false;
                    wantBrakeEngaged = true;
                }
                if ( armEncoder > targetPosArm + (targetPosArm*.05) || armEncoder < targetPosArm - (targetPosArm*.05) ) {
                    motionMagicArm = true; 
                } else {
                    motionMagicArm = false;
                }                
                break;
        case 3: wantBrakeEngaged = false; //Preclimb
                elevatorTarget = 0.0;
                armTarget = 0.0;
                if ( elevatorEncoder > targetPosElev + (targetPosElev*.05) || elevatorEncoder < targetPosElev - (targetPosElev*.05) ) {
                    motionMagicElevator = true; 
                } else {
                    motionMagicElevator = false;
                    wantBrakeEngaged = true;
                }
                if ( armEncoder > targetPosArm + (targetPosArm*.05) || armEncoder < targetPosArm - (targetPosArm*.05) ) {
                    motionMagicArm = true; 
                } else {
                    motionMagicArm = false;
                }                
                break;
        case 4: wantBrakeEngaged = false; //The climb
                elevatorTarget = 0.0;
                armTarget = 0.0;
                if ( elevatorEncoder > targetPosElev + (targetPosElev*.05) || elevatorEncoder < targetPosElev - (targetPosElev*.05) ) {
                    motionMagicElevator = true; 
                } else {
                    motionMagicElevator = false;
                    wantBrakeEngaged = true;
                }
                if ( armEncoder > targetPosArm + (targetPosArm*.05) || armEncoder < targetPosArm - (targetPosArm*.05) ) {
                    motionMagicArm = true; 
                } else {
                    motionMagicArm = false;
                }                
                break;
                
    }*/
    
    if ( elevatorBotPosition ) { //zeroes the elevator encoder if lower limit switch is pressed
        m_motorLSPTO.SetSelectedSensorPosition(0, 0, 10);
    }
    if ( elevatorLowLimit.Get() ) {
        elevatorBotPosition = false;
    } else {
        elevatorBotPosition = true;
    }
    
//Peform Motion Magic when motionMagicElevator/Arm = true, else run Percent Output for elevator control and arm position hold
    /*if ( motionMagicElevator ) { //4096 ticks/rev
        m_motorLSPTO.Set(ControlMode::MotionMagic, targetPosElev);
    } else {
        m_motorLSPTO.Set( ControlMode::PercentOutput, m_console.GetY() );
    }
    if ( motionMagicArm ) { //4096 ticks/rev
        m_armMotor.Set(ControlMode::MotionMagic, targetPosElev);
    } else {
        m_armMotor.Set( ControlMode::PercentOutput, m_console.GetX() );
    }*/
    if ( m_console.GetRawButton(5) ) {
        /*4096 ticks/rev * 10 Rotations in either direction */
        double targetPos = 0 * 4096;
        m_motorLSPTO.Set(ControlMode::MotionMagic, targetPos);
    } else if ( m_console.GetRawButton(4) ) {
        /*4096 ticks/rev * 10 Rotations in either direction */
        double targetPos = -1 * 4096;
        m_motorLSPTO.Set(ControlMode::MotionMagic, targetPos);
    } else if ( m_console.GetRawButton(3) ) {
        /*4096 ticks/rev * 10 Rotations in either direction */
        double targetPos = -4 * 4096;
        m_motorLSPTO.Set(ControlMode::MotionMagic, targetPos);
    } else {
        /* Percent Output */
        m_motorLSPTO.Set(ControlMode::PercentOutput, m_console.GetY() * .6 );
    }







//all of this code prevents a missle switch from doing something if the previous switch hasn't been flipped
    if ( m_console.GetRawButton(9) ) { 
        missleSwitchOne = true;
    } else {
        missleSwitchOne = false;
    }
    if ( m_console.GetRawButton(10) && missleSwitchOne ) {
        missleSwitchTwo = true;
    } else {
        missleSwitchTwo = false;
    }
    if ( m_console.GetRawButton(11) && missleSwitchTwo) {
        missleSwitchThree = true;
    } else {
        missleSwitchThree = false;
    }
// Endgame code
    if ( missleSwitchOne ) { //first missle 
        if ( !missleSwitchTwo ) { //prevents the elevator from getting ripped from position 3 to 4 every time the if cycles
            elevarmPosition = 3;
        }
    }
    if ( missleSwitchTwo ) { //Second missle switch: ptowingDrop, vacuum pump on, dump when limit switch pressed
        vacMotorOn = true;
        wantEndShift = true;
    } else {
        wantEndShift = false;
        vacMotorOn = false;
    }
    if ( missleSwitchThree ) {
        elevarmPosition = 4;
     //   if (vacLimit.Get()) {
      //      habContact = true;
     //   }
    }
//hatch, brakecode
    if ( m_console.GetRawButtonPressed(1) ) {
        wantHatchOpen = true;
    } else if ( m_console.GetRawButtonPressed(2) ) {
        wantHatchOpen = false;
    }
    if ( hatchOpen ) {
        if ( !wantHatchOpen ) {
            hatchOpen = false;
            m_hatchSolenoid.Set(false); // hatch close
        }
    } else {
        if ( wantHatchOpen ) {
            hatchOpen = true;
            m_hatchSolenoid.Set(true); // hatch open
        }
    }
    //servo
    /*if ( m_console.GetRawButtonPressed(3) ) {
        wantServoReset = true;
    } else ( m_console.GetRawButtonPressed(4) ); {
        wantServoReset = false;
    }
    if ( servoReset ) {
        if ( !wantServoReset ) {
            servoReset = false;
            dumpValve.Set(1);
        }
    } else {
        if ( wantServoReset ) {
            servoReset = true;
            dumpValve.Set(0);
        }
    } */
    if ( brakeEngaged ) {
        if ( !wantBrakeEngaged ) {
            brakeEngaged = false;
            m_brakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // Brake off
        }
    } else {
        if ( wantBrakeEngaged ) {
            brakeEngaged = true;
            m_brakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // Brake on
        }
    }
    if ( m_console.GetRawButton(6) ) {
            wantBrakeEngaged = false;
    } else if ( m_console.GetRawButton(7) ) {
            wantBrakeEngaged = true;
    }
    //fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff
    if ( m_console.GetRawButton(3) ) {
        elevarmPosition = 1;
    } else if ( m_console.GetRawButton(4) ) {
        elevarmPosition = 2;
    } else if ( m_console.GetRawButton(5) ) {
        elevarmPosition = 0;
    }
//endgame shift/servo code
    if ( endShift ) {
        if ( !wantEndShift) {
            endShift = false;
            m_ptoSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // Reset Wing piston, arm piston, disengage PTO
        }
    } else {
        if ( wantEndShift ) {
            endShift = true;
            m_ptoSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // Drop Wings, arms, engage PTO
        }
    }
    //if ( habContact ) {
     //   dumpValve.Set(1.0);     
  //  }
    m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetZ() );
//vacuum motor code
    if ( vacMotorOn )  {
        m_motorVacuum.Set(ControlMode::PercentOutput, 0.5);
    } else {
        m_motorVacuum.Set(ControlMode::PercentOutput, 0.0);
    }
#ifdef DRIVEMETHODJOYSTICK
   /* if ( ( m_stick.GetRawButton(3) ) &&         // If driver is pressing the
         ( 1  == limev )                )  {    // "drivetotarget" button and
                                                // the limelight has a target,
        DriveToTarget();         // then autonomously drive towards the target
    } else { */
        if ( m_stick.GetTrigger() ) {
            m_shiftingSolenoid.Set(true); // hi gear
        } else {
            m_shiftingSolenoid.Set(false); // lo gear
        }
        if ( endShift ) {
            m_drive.CurvatureDrive( m_stick.GetY(), 0, 0 );
        }

/*          else if ( m_stick.GetZ() < 0 ) {
            m_drive.CurvatureDrive( m_stick.GetY(),
            powl( fabs(m_stick.GetZ()), m_stick.GetThrottle()+2.0 ),
            m_stick.GetTop() );
        } else {
            m_drive.CurvatureDrive( m_stick.GetY(),
            -powl( fabs(m_stick.GetZ()), m_stick.GetThrottle()+2.0 ),
            m_stick.GetTop() );
        }
        
    }  
    */
#else  
    if ( m_xbox.GetBumper(frc::GenericHID::kLeftHand) ) {
        m_shiftingSolenoid.Set(true); // hi gear
    } else {
        m_shiftingSolenoid.Set(false); // lo gear
    } 
    m_drive.CurvatureDrive( GetSimY(),
                            m_xbox.GetX(frc::GenericHID::kLeftHand),
                            m_xbox.GetBumperPressed(frc::GenericHID::kRightHand) );
#endif    
  }
 private:
//Need to add:  gyro,
    WPI_TalonSRX m_motorRSMaster{0}; // Right side drive motor
    WPI_TalonSRX m_motorLSMaster{15}; // Left  side drive motor   
    WPI_TalonSRX m_motorLSPTO{2}; // Left  side PTO (elevator motor)
    WPI_TalonSRX m_armMotor{9}; // 4-Bar arm motor
    WPI_VictorSPX m_motorRSSlave1{1};
    WPI_VictorSPX m_motorLSSlave1{14};
    WPI_VictorSPX m_motorVacuum{8};
    int iAutoCount;
    int elevarmPosition = 0; //Elevator and arm position. 0 is start config, 1 is hatch low, 2 is hatch medium, 3 is preclimb, 4 is hab contact, 5 is final climb
    int armEncoder = m_armMotor.GetSelectedSensorPosition();
    frc::Joystick m_stick{0};
    frc::Joystick m_console{1};
    frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
    frc::Compressor m_compressor{0};
    frc::PowerDistributionPanel pdp{0};
    //frc::DoubleSolenoid m_shiftingSolenoid{0,1}; //shifters
    //frc::DoubleSolenoid m_hatchSolenoid{3,2}; //grab hatch in/out
    //frc::DoubleSolenoid m_brakeSolenoid{4,5}; //brake on/off
    //frc::DoubleSolenoid m_ptodropSolenoid{6,7}; //drops the vac arms AND engages PTO
    frc::DoubleSolenoid m_ptoSolenoid{0,1};
    frc::DoubleSolenoid m_brakeSolenoid{2,3};
    frc::Solenoid m_shiftingSolenoid{4};
    frc::Solenoid m_dropSolenoid{5};
    frc::Solenoid m_hatchSolenoid{6};
    frc::Solenoid m_wingSolenoid{7};
    frc::AnalogInput DistSensor1{0};
    frc::DigitalInput vacLimit{1};
    frc::DigitalInput elevatorLowLimit{2};
    frc::DigitalInput armHighLimit{3};
    frc::Servo dumpValve{0};
    std::shared_ptr<NetworkTable> limenttable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    bool wantHatchOpen = true;
    bool hatchOpen = true;
    bool wantEndShift = false;
    bool endShift = false;
    bool wantBrakeEngaged = true;
    bool servoReset = true;
    bool wantServoReset = true;
    bool brakeEngaged = true;
    bool missleSwitchOne = false;
    bool missleSwitchTwo = false;
    bool missleSwitchThree = false;
    bool vacMotorOn = false;
    bool habContact = false;
    bool armTopPosition = false;
    bool elevatorBotPosition = false;
    bool motionMagicElevator = false;
    bool motionMagicArm = false;
        // limelight variables: x offset from centerline,
        //                      y offset from centerline,
        //                      area of target (0-100),
        //                      whether the data is valid
    double limex, limey, limea, limev, limes;
    double motorOutput = m_motorLSPTO.GetMotorOutputPercent();
    /*double elevatorTarget = 2.0; //defaulted to start position
    double armTarget = 0.0; //defaulted to start position         ???????
    double targetPosElev = elevatorTarget * 4096;
    double targetPosArm = armTarget * 4096;
    double elevatorEncoder = m_motorLSPTO.GetSelectedSensorPosition();
    */
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif