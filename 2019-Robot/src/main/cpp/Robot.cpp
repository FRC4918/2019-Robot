/*
 Our 2019 Robot code. Written by Max Morningstar and [Other contributors here] with guidance from Jeff Gibbons
 */


#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

using std::cout;
using std::endl;
#define DRIVEMETHODJOYSTICK
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() {
      m_ptoSolenoid.ClearAllPCMStickyFaults();
  }
  void AutonomousInit() override {
  }
  void AutonomousPeriodic() override {
  }
  void TeleopInit() override {
    m_motorLSSlave1.Follow(m_motorLSMaster);
    m_motorLSSlave2.Follow(m_motorLSMaster);
    m_motorRSSlave1.Follow(m_motorRSMaster);
    m_motorRSSlave2.Follow(m_motorRSMaster);
    m_shiftingSolenoid.Set(true);
    m_hatchSolenoid.Set(true);


  }
  void TeleopPeriodic() override {
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
#ifdef DRIVEMETHODJOYSTICK
    if ( m_stick.GetTrigger() ) {
        m_shiftingSolenoid.Set(true); // hi gear
    } else {
        m_shiftingSolenoid.Set(false); // lo gear
    }
    if ( m_stick.GetZ() < 0 ) {
        m_drive.CurvatureDrive( m_stick.GetY(),
        powl( fabs(m_stick.GetZ()), m_stick.GetThrottle()+2.0 ),
        m_stick.GetTop() );
    } else {
         m_drive.CurvatureDrive( m_stick.GetY(),
            -powl( fabs(m_stick.GetZ()), m_stick.GetThrottle()+2.0 ),
            m_stick.GetTop() );
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
    WPI_VictorSPX m_motorRSSlave1{1};
    WPI_VictorSPX m_motorLSSlave1{14};
    WPI_VictorSPX m_motorLSSlave2{12};
    WPI_VictorSPX m_motorRSSlave2{3};
    int iAutoCount;
    frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
    frc::DifferentialDrive m_elevator{ m_motorLSPTO, m_motorRSPTO };
    frc::Compressor m_compressor{0};
    frc::Solenoid m_shiftingSolenoid{0}; //shifters
    frc::Solenoid m_hatchSolenoid{1}; //grab hatch in/out
    frc::DoubleSolenoid m_ptoSolenoid{6,7
    };
    frc::AnalogInput DistSensor1{0};
    std::shared_ptr<NetworkTable> limenttable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    bool wantHatchOpen = true;
    bool hatchOpen = true;
    bool wantPTOShift = true;
    bool PTOShift = true;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif