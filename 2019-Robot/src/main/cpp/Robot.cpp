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
    m_shiftingSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    m_hatchSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    m_cargoSolenoid.Set(frc::DoubleSolenoid::Value::kForward);

  }
  void TeleopPeriodic() override {
    if ( m_xbox.GetYButtonPressed() ) {
        wantHatchOpen = !wantHatchOpen;
    }
    if ( hatchOpen ) {
        if ( !wantHatchOpen ) {
            hatchOpen = false;
            m_hatchSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // hatch close
        }
    } else {
        if ( wantHatchOpen ) {
            hatchOpen = true;
            m_hatchSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // hatch open
        }
    }
    if ( m_xbox.GetBButtonPressed() ) {
        wantCargoOpen = !wantCargoOpen;
    }
    if ( cargoOpen ) {
        if ( !wantCargoOpen ) {
            cargoOpen = false;
            m_cargoSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // cargo arms close
        }
    } else {
        if ( wantCargoOpen ) {
            cargoOpen = true;
            m_cargoSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // cargo arms open
        }
    }
    if ( m_xbox.GetAButton() ) { 
        m_motorIntake.Set(ControlMode::PercentOutput, 0.2);
    }
    else if ( m_xbox.GetXButton() ) { 
        m_motorIntake.Set(ControlMode::PercentOutput, -0.2);
    }
    else {
        m_motorIntake.Set(ControlMode::PercentOutput, 0.0);
    }
#ifdef DRIVEMETHODJOYSTICK
    if ( m_stick.GetTrigger() ) {
        m_shiftingSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // hi gear
    } else {
        m_shiftingSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // lo gear
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
        m_shiftingSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // hi gear
    } else {
        m_shiftingSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // lo gear
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
    WPI_TalonSRX m_motorIntake{7}; // intake motor
    WPI_VictorSPX m_motorRSSlave1{3};
    WPI_VictorSPX m_motorLSSlave1{4};
    WPI_VictorSPX m_motorLSSlave2{5};
    WPI_VictorSPX m_motorRSSlave2{6};
    int iAutoCount;
    frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
    frc::Compressor m_compressor{0};
    frc::DoubleSolenoid m_shiftingSolenoid{0,1}; //shifters
    frc::DoubleSolenoid m_hatchSolenoid{2,3}; //grab hatch in/out
    frc::DoubleSolenoid m_cargoSolenoid{4,5}; //grab cargo open/close
    frc::AnalogInput DistSensor1{0};
    frc::Spark IntakeMotors{0};
    std::shared_ptr<NetworkTable> limenttable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    bool wantHatchOpen = true;
    bool hatchOpen = true;
    bool wantCargoOpen = true;
    bool cargoOpen = true;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif