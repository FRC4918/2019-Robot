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

class Robot : public frc::TimedRobot {
 public:
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
    if ( m_stick.GetTrigger() ) {
        m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // hi gear
    } else {
        m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward); // lo gear
    }
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
