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
              frc::CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
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
  void GetAllVariables()  {
    limev = limenttable->GetNumber("tv",0.0);
    limex = limenttable->GetNumber("tx",0.0);
    limea = limenttable->GetNumber("ta",0.0);
    limey = limenttable->GetNumber("ty",0.0);
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
      if ( PTOShift ) {
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

 public:
  void RobotInit() {
                                // start a thread processing USB camera images
      std::thread visionThread(VisionThread);
      visionThread.detach();

      m_ptoSolenoid.ClearAllPCMStickyFaults();
      m_motorLSSlave1.Follow(m_motorLSMaster);
      m_motorLSSlave2.Follow(m_motorLSMaster);
      m_motorRSSlave1.Follow(m_motorRSMaster);
      m_motorRSSlave2.Follow(m_motorRSMaster);

      m_shiftingSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
      m_hatchSolenoid.Set(frc::DoubleSolenoid::Value::kForward);

      m_motorLSMaster.ConfigClosedloopRamp(1.0); // safety ramps on all motors
      m_motorRSMaster.ConfigClosedloopRamp(1.0);
      m_armMotor.ConfigClosedloopRamp(2.0);
      m_motorLSPTO.ConfigClosedloopRamp(4.0);
      m_motorRSPTO.ConfigClosedloopRamp(4.0);
      
      m_motorLSMaster.ConfigOpenloopRamp(1.0);
      m_motorRSMaster.ConfigOpenloopRamp(1.0);
      m_armMotor.ConfigOpenloopRamp(2.0);
      m_motorLSPTO.ConfigOpenloopRamp(4.0);
      m_motorRSPTO.ConfigOpenloopRamp(4.0);
                  // set all current limits to 40 Amps, after 200 milliseconds
      m_motorLSMaster.ConfigContinuousCurrentLimit( 40, 10 );
      m_motorRSMaster.ConfigContinuousCurrentLimit( 40, 10 );
      m_armMotor.ConfigContinuousCurrentLimit( 40, 10 );
      m_motorLSPTO.ConfigContinuousCurrentLimit( 40, 10 );
      m_motorRSPTO.ConfigContinuousCurrentLimit( 40, 10 );
              // set all peak current limits to 50 Amps up to 200 milliseconds
      m_motorLSMaster.ConfigPeakCurrentLimit( 50, 10 );
      m_motorRSMaster.ConfigPeakCurrentLimit( 50, 10 );
      m_armMotor.ConfigPeakCurrentLimit( 50, 10 );
      m_motorLSPTO.ConfigPeakCurrentLimit( 50, 10 );
      m_motorRSPTO.ConfigPeakCurrentLimit( 50, 10 );

      m_motorLSMaster.ConfigPeakCurrentDuration( 200, 10 );
      m_motorRSMaster.ConfigPeakCurrentDuration( 200, 10 );
      m_armMotor.ConfigPeakCurrentDuration( 200, 10 );
      m_motorLSPTO.ConfigPeakCurrentDuration( 200, 10 );
      m_motorRSPTO.ConfigPeakCurrentDuration( 200, 10 );

      m_motorLSMaster.EnableCurrentLimit( true );
      m_motorRSMaster.EnableCurrentLimit( true );
      m_armMotor.EnableCurrentLimit( true );
      m_motorLSPTO.EnableCurrentLimit( true );
      m_motorRSPTO.EnableCurrentLimit( true );
      
      m_motorLSMaster.ConfigNominalOutputForward( 0, 10 );
      m_motorRSMaster.ConfigNominalOutputForward( 0, 10 );
      m_armMotor.ConfigNominalOutputForward( 0, 10 );
      m_motorLSPTO.ConfigNominalOutputForward( 0, 10 );
      m_motorRSPTO.ConfigNominalOutputForward( 0, 10 );

      m_motorLSMaster.ConfigNominalOutputReverse( 0, 10 );
      m_motorRSMaster.ConfigNominalOutputReverse( 0, 10 );
      m_armMotor.ConfigNominalOutputReverse( 0, 10 );
      m_motorLSPTO.ConfigNominalOutputReverse( 0, 10 );
      m_motorRSPTO.ConfigNominalOutputReverse( 0, 10 );

      m_motorLSMaster.ConfigPeakOutputForward( 1, 10 );
      m_motorRSMaster.ConfigPeakOutputForward( 1, 10 );
      m_armMotor.ConfigPeakOutputForward( 1, 10 );
      m_motorLSPTO.ConfigPeakOutputForward( 1, 10 );
      m_motorRSPTO.ConfigPeakOutputForward( 1, 10 );

      m_motorLSMaster.ConfigPeakOutputReverse( -1, 10 );
      m_motorRSMaster.ConfigPeakOutputReverse( -1, 10 );
      m_armMotor.ConfigPeakOutputReverse( -1, 10 );
      m_motorLSPTO.ConfigPeakOutputReverse( -1, 10 );
      m_motorRSPTO.ConfigPeakOutputReverse( -1, 10 );
  }
  void AutonomousInit() override {
  }
  void AutonomousPeriodic() override {
    GetAllVariables();
  }
  void TeleopInit() override {
    m_motorLSSlave1.Follow(m_motorLSMaster);
    m_motorLSSlave2.Follow(m_motorLSMaster);
    m_motorRSSlave1.Follow(m_motorRSMaster);
    m_motorRSSlave2.Follow(m_motorRSMaster);
    m_shiftingSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    m_hatchSolenoid.Set(frc::DoubleSolenoid::Value::kForward);


  }
  void TeleopPeriodic() override {
    static time_t tStartChooChooTime = 0;

    GetAllVariables();

    if ( m_console.GetRawButtonPressed(2) ) {
        wantHatchOpen = !wantHatchOpen;
    }
    if ( hatchOpen ) {
        if ( !wantHatchOpen ) {
            hatchOpen = false;
            m_hatchSolenoid.Set(true); // hatch close
        }
    } else {
        if ( wantHatchOpen ) {
            hatchOpen = true;
            m_hatchSolenoid.Set(false); // hatch open
        }
    }
     m_elevator.ArcadeDrive(m_console.GetY(), m_console.GetZ()); // Elevator drive code. Gets nonexistant Z-axis instead of X to prevent carraige getting messed up
    if ( m_console.GetRawButtonPressed(1) ) {
        wantPTOShift = !wantPTOShift;
    }
    if ( PTOShift ) {
        if ( !wantPTOShift) {
            PTOShift = false;
            m_ptoSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // PTO Disengage
        }
    } else {
        if ( wantPTOShift ) {
            PTOShift = true;
            m_ptoSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // PTO Engage
        }
    }
    if ( m_console.GetRawButton(7) ) {
        m_armMotor.Set(ControlMode::PercentOutput, 0.4 );
    } else if ( m_console.GetRawButton(8) ) {
        m_armMotor.Set(ControlMode::PercentOutput, -0.4 );
    } else {
        m_armMotor.Set(ControlMode::PercentOutput, 0.0 ); 
    }
    if ( m_console.GetRawButton(3) ) {
        wantArmDrop = true;
    } else {
        wantArmDrop = false;
    }
    if ( wantArmDrop ) {
        m_armSolenoid.Set(true); // Drop Vacuum Arms
    } else {
        m_armSolenoid.Set(false); // Reset Vacuum Arms holder
    }
    if ( m_console.GetRawButton(5) )  {
        m_motorVacuum.Set(ControlMode::PercentOutput, 1.0);
        tStartChooChooTime = std::time(NULL);
    } else if ( std::time(NULL) < (tStartChooChooTime+30) ) {
        m_motorVacuum.Set(ControlMode::PercentOutput, 1.0);
    } else {
        m_motorVacuum.Set(ControlMode::PercentOutput, 0.0);
    }
#ifdef DRIVEMETHODJOYSTICK
    if ( ( m_stick.GetRawButton(3) ) &&         // If driver is pressing the
         ( 1  == limev )                )  {    // "drivetotarget" button and
                                                // the limelight has a target,
        DriveToTarget();         // then autonomously drive towards the target
    } else {
        if ( m_stick.GetTrigger() ) {
            m_shiftingSolenoid.Set(true); // hi gear
        } else {
            m_shiftingSolenoid.Set(false); // lo gear
        }
        if ( PTOShift ) {
            m_drive.CurvatureDrive( m_stick.GetY(), 0, 0 );
        } else if ( m_stick.GetZ() < 0 ) {
            m_drive.CurvatureDrive( m_stick.GetY(),
            powl( fabs(m_stick.GetZ()), m_stick.GetThrottle()+2.0 ),
            m_stick.GetTop() );
        } else {
            m_drive.CurvatureDrive( m_stick.GetY(),
            -powl( fabs(m_stick.GetZ()), m_stick.GetThrottle()+2.0 ),
            m_stick.GetTop() );
        }
    }  
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
    frc::Joystick m_stick{0};
    frc::Joystick m_console{1};
    WPI_TalonSRX m_motorRSMaster{2}; // Right side drive motor
    WPI_TalonSRX m_motorLSMaster{13}; // Left  side drive motor   
    WPI_TalonSRX m_motorRSPTO{0}; // Right side PTO
    WPI_TalonSRX m_motorLSPTO{15}; // Left  side PTO
    WPI_TalonSRX m_armMotor{6}; // 4-Bar arm motor
    WPI_VictorSPX m_motorRSSlave1{1};
    WPI_VictorSPX m_motorLSSlave1{14};
    WPI_VictorSPX m_motorLSSlave2{12};
    WPI_VictorSPX m_motorRSSlave2{3};
    WPI_VictorSPX m_motorVacuum{7};
    int iAutoCount;
    frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
    frc::DifferentialDrive m_elevator{ m_motorLSPTO, m_motorRSPTO };
    frc::Compressor m_compressor{0};
    frc::Solenoid m_shiftingSolenoid{0}; //shifters
    frc::Solenoid m_hatchSolenoid{4}; //grab hatch in/out
    frc::Solenoid m_armSolenoid{1};
    frc::DoubleSolenoid m_ptoSolenoid{6,7};
    frc::AnalogInput DistSensor1{0};
    std::shared_ptr<NetworkTable> limenttable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    bool wantHatchOpen = true;
    bool hatchOpen = true;
    bool wantPTOShift = true;
    bool PTOShift = true;
    bool wantArmDrop = false;
        // limelight variables: x offset from centerline,
        //                      y offset from centerline,
        //                      area of target (0-100),
        //                      whether the data is valid
    double limex, limey, limea, limev;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif