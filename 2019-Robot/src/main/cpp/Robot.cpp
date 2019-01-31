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
#undef DRIVEMETHODJOYSTICK
class Robot : public frc::TimedRobot {
 public:
  double GetSimY() {
      double TriggerAxisLeft =  m_xbox.GetTriggerAxis (frc::GenericHID::kLeftHand);
      double TriggerAxisRight = m_xbox.GetTriggerAxis (frc::GenericHID::kRightHand);
      double returnValue = 0.0;
    if ( TriggerAxisLeft > 0.1) {
        returnValue = -TriggerAxisLeft;
    }
    else  {
        returnValue = TriggerAxisRight;
    }
    return returnValue;
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
    m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }
  void TeleopPeriodic() override { 
#ifdef DRIVEMETHODJOYSTICK
    if ( m_stick.GetTrigger() ) {
        m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // hi gear
    } else {
        m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward); // lo gear
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
        m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // hi gear
    } else {
        m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward); // lo gear
    } 
    m_drive.CurvatureDrive( GetSimY(),
                            m_xbox.GetX(frc::GenericHID::kLeftHand),
                            m_xbox.GetBumperPressed(frc::GenericHID::kRightHand) );
#endif                          
  }
 private:
    frc::Joystick m_stick{0};
    frc::XboxController m_xbox{1};
    WPI_TalonSRX m_motorRSMaster{1}; // Right side drive motor
    WPI_TalonSRX m_motorLSMaster{2}; // Left  side drive motor      
    WPI_TalonSRX m_motorArmMaster{7}; // Arm motor
    WPI_VictorSPX m_motorRSSlave1{3};
    WPI_VictorSPX m_motorLSSlave1{4};
    WPI_VictorSPX m_motorLSSlave2{5};
    WPI_VictorSPX m_motorRSSlave2{6};
    int iAutoCount;
    frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
    frc::Compressor m_compressor{0};
    frc::DoubleSolenoid m_doublesolenoid{0,1};
    frc::AnalogInput DistSensor1{0};
    frc::Spark IntakeMotors{0};
    std::shared_ptr<NetworkTable> limenttable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif