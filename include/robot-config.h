using namespace vex;

extern brain Brain;

// VEXcode devices
extern smartdrive Drivetrain;
extern motor Belt;
extern motor Intake;
extern digital_out Piston;
extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );