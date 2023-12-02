
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

class Sensor : public frc2::SubsystemBase {
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Set the current angle of the arm (0 - 180 degrees).
   *
   * @param angleDeg the commanded angle
   */
  void m_sensor(double angleDeg);

 private:
  frc::DigitalInput m_sensor(int);
};