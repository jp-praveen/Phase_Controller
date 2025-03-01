## Author: Praveen Jawaharlal Ayyanathan
## Algorithm details can be found in https://arc.aiaa.org/doi/10.2514/6.2024-4290


#include <iq_module_communication.hpp>
#include <generic_interface.hpp>
#include <brushless_drive_client.hpp>

GenericInterface com;

// Module communication for two motors
IqSerial ser1(Serial1);
IqSerial ser2(Serial2);

// Clients for control and monitoring for Motor 1
PropellerMotorControlClient prop_control1(0);
BrushlessDriveClient brushless_drive1(0);

// Clients for control and monitoring for Motor 2
PropellerMotorControlClient prop_control2(0);
BrushlessDriveClient brushless_drive2(0);

const float PI_VAL = 3.14159265358979323846;

// Setpoints (Speed and Phase)
const float sp = 39.873 * PI_VAL;  // Speed setpoint in rad/s
const float asp = 5;  // Angular setpoint in degrees
const float ang_time = asp / (360 * 5 / 60);  // Time required for phase change
const float ramp_time = 5000.0; // Ramp duration in milliseconds

unsigned long start_time;
unsigned long index = 0;
bool maneuver_active = false;

// Data storage (pre-allocated arrays)
const int num_samples = 500;
float speed_arr[num_samples];
float angle_arr[num_samples];

// === Function to Update PID Settings (Manually tuned) ===
void updatePIDSettings() {
    ser1.set(prop_control1.velocity_ki_, 0.009f);
    ser1.set(prop_control1.velocity_kp_, 0.03f);
    ser1.set(prop_control1.velocity_kd_, 0.009f);

    ser2.set(prop_control2.velocity_ki_, 0.009f);
    ser2.set(prop_control2.velocity_kp_, 0.03f);
    ser2.set(prop_control2.velocity_kd_, 0.009f);
}

// === Function to Log Speed and Angle Data ===
void logData() {
    if (index < num_samples) {
        float angle1, angle2, speed1, speed2;

        ser1.get(brushless_drive1.obs_angle_, angle1);
        ser2.get(brushless_drive2.obs_angle_, angle2);

        ser1.get(brushless_drive1.obs_velocity_, speed1);
        ser2.get(brushless_drive2.obs_velocity_, speed2);

        // Convert speed from rad/s to RPM
        speed1 = speed1 * 60 / (2 * PI_VAL);
        speed2 = speed2 * 60 / (2 * PI_VAL);

        // Convert angles from radians to degrees
        angle1 = angle1 * 180 / PI_VAL + 180;
        angle2 = angle2 * 180 / PI_VAL + 180;

        // Store in arrays
        speed_arr[index] = speed1;
        angle_arr[index] = angle1;

        index++;
    }
}

// === Velocity Ramp===
void rampPhase() {
    unsigned long current_time = millis();
    float ramp_progress = (float)(current_time - start_time) / ramp_time;

    float interpolated_speed = ramp_progress * sp; // Gradual speed increase
    ser1.set(prop_control1.ctrl_velocity_, interpolated_speed);
    ser2.set(prop_control2.ctrl_velocity_, interpolated_speed);

    logData();  // Log values
}

// === Phase Change Maneuver ===
void maneuverPhase() {
    Serial.println("Phase maneuver started!");

    unsigned long start_time_ang = millis();
    while ((millis() - start_time_ang) < ang_time * 1000) {
        ser1.set(prop_control1.ctrl_velocity_, sp + 5 * 2 * PI_VAL / 60);
        ser2.set(prop_control2.ctrl_velocity_, sp + 5 * 2 * PI_VAL / 60);

        logData();
    }

    Serial.println("Phase maneuver complete!");
}

// === Setup Function ===
void setup() {
    Serial.begin(115200);
    ser1.begin(115200);
    ser2.begin(115200);
    updatePIDSettings();
    start_time = millis();
}

// === Main Loop  ===
void loop() {
    unsigned long current_time = millis();

    // Phase-Start: Gradual Ramp-Up 
    if (current_time - start_time < ramp_time) {
        rampPhase();
    } 
    // Phase Change Maneuver
    else {
        ser1.set(prop_control1.ctrl_velocity_, sp);
        ser2.set(prop_control2.ctrl_velocity_, sp);

        logData();  // Log data in every loop iteration

        if (Serial.available()) {
            char command = Serial.read();
            if (command == 'a') {
                maneuverPhase();  // Execute maneuver when 'a' is pressed
            } else if (command == 'q') {
                Serial.println("Exiting loop");
                while (true);  // Stop execution
            }
        }
    }
}
