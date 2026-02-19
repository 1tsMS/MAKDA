#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// --- Robot Dimensions (You MUST measure these in mm) ---
#define L1 45.0f  // Coxa Length (Hip to Thigh Pivot)
#define L2 80.0f  // Femur Length (Thigh to Knee Pivot)
#define L3 120.0f // Tibia Length (Knee to Foot Tip)

// --- Servo Driver ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// --- Servo Configuration ---
#define SERVO_MIN 100
#define SERVO_MAX 700
#define SERVO_FREQ 50
#define ANGLE_MIN 0
#define ANGLE_MAX 180
#define LOGICAL_JOINT_COUNT 12
#define SERVO_COUNT 16

// --- Calibration Data (Copied from esp32_receiver.ino) ---
static const int8_t CALIB_DIR[SERVO_COUNT] = {
  -1, +1, -1, -1, -1, +1, +1, +1,
  -1, +1, -1, +1, +1, +1, +1, +1,
};

static const int8_t CALIB_OFFSET[SERVO_COUNT] = {
  +3, -3, -2, -6, +0, -4, +1, +0,
  -1, +4, +0, +6, +0, +0, +0, +0,
};

static const uint8_t CALIB_MIN[SERVO_COUNT] = {
  28, 22, 41, 57, 24, 44, 51, 17,
  49, 4, 26, 44, 0, 0, 0, 0,
};

static const uint8_t CALIB_MAX[SERVO_COUNT] = {
  129, 153, 180, 174, 180, 158, 149, 158,
  180, 131, 180, 150, 180, 180, 180, 180,
};

// Logical to Physical Pin Mapping
static const uint8_t CALIB_INDEX[SERVO_COUNT] = {
  0, 1, 2, 3, 4, 5, 11, 10,
  9, 15, 14, 13, 6, 7, 8, 12, // Remaining are unused
};

// --- Inverse Kinematics Function ---
// Calculates servo angles (theta1, theta2, theta3) for a leg to reach (x, y, z)
// x, y, z: Coordinates in mm relative to the coxa pivot (shoulder)
// theta1: Coxa angle (Hip Horizontal)
// theta2: Femur angle (Hip Vertical/Thigh)
// theta3: Tibia angle (Knee)
// Returns true if reachable, false if out of range
bool inverseKinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3, bool isRightSide) {
    // 1. Calculate Coxa Angle (Theta 1) in X-Y plane
    theta1 = atan2(y, x) * (180.0 / PI); // Basic angle

    // 2. Calculate distance from Coxa pivot to Foot in X-Y plane
    float L_ground = sqrt(x*x + y*y);

    // 3. Subtract L1 (Coxa length) to get distance from Femur pivot to Foot
    // The femur pivot is L1 distance away along the Theta1 vector
    float d = L_ground - L1;

    // 4. Calculate distance from Femur pivot to Foot (Hypotenuse in Z-d plane)
    // d is horizontal, z is vertical
    float h = sqrt(d*d + z*z);

    // Check reachability
    if (h > (L2 + L3)) return false; // Too far

    // 5. Calculate Femur Angle (Theta 2) using Law of Cosines
    // Alpha1 is angle of (d, z) vector above horizontal
    float alpha1 = atan2(z, d);
    // Alpha2 is internal angle of the triangle at Femur pivot
    float cos_val_2 = (L2*L2 + h*h - L3*L3) / (2 * L2 * h);
    if (cos_val_2 < -1.0) cos_val_2 = -1.0;
    if (cos_val_2 > 1.0) cos_val_2 = 1.0;
    float alpha2 = acos(cos_val_2);
    
    // Resulting Femur Angle
    // Note: Servo orientation varies. Usually 0 is horizontal or vertical.
    // Assuming 0 is horizontal forward, 90 is vertical up.
    // Let's output deviations from horizontal for now.
    theta2 = (alpha1 + alpha2) * (180.0 / PI);

    // 6. Calculate Tibia Angle (Theta 3) using Law of Cosines
    // Internal angle opposite to h
    float cos_val_3 = (L2*L2 + L3*L3 - h*h) / (2 * L2 * L3);
    if (cos_val_3 < -1.0) cos_val_3 = -1.0;
    if (cos_val_3 > 1.0) cos_val_3 = 1.0;
    float alpha3 = acos(cos_val_3);
    
    // Resulting Tibia Angle
    // Usually defined relative to Femur or vertical.
    // Let's assume 180 is straight leg, 90 is bent.
    theta3 = alpha3 * (180.0 / PI);

    // --- Robot Specific Adjustments ---
    // Adjust these based on how your servos are mounted!
    // Often: 
    //   Theta1 (Hip): 90 is straight out. 
    //   Theta2 (Thigh): 90 is horizontal? or vertical?
    //   Theta3 (Knee): 90 is bent 90 degrees?

    // Common standard for simple quadrupeds:
    theta1 += 90; // Center at 90
    
    // Invert angles for right side if needed (usually handled by calibration direction, but sometimes math needs it)
    // Since we handle direction in computePulse, we export logical geometric angles here.
    
    return true; 
}


// --- Servo Output Helper ---
void setServo(uint8_t logicalId, float angle) {
    if (logicalId >= LOGICAL_JOINT_COUNT) return;

    // Apply Calibration Logic (from your existing firmware)
    int corrected = constrain((int)angle, (int)ANGLE_MIN, (int)ANGLE_MAX);
    
    // Direction
    if (CALIB_DIR[logicalId] < 0) {
        corrected = 180 - corrected;
    }

    // Offset (Note: Your firmware logic adds offset AFTER handling dir for corrected, 
    // but the array is static const. Let's replicate exact firmware logic)
    int offset = (int)CALIB_OFFSET[logicalId];
    if (CALIB_DIR[logicalId] < 0) {
        offset = -offset;
    }
    corrected += offset; // Apply offset

    // Min/Max Limits
    corrected = constrain(corrected, (int)CALIB_MIN[logicalId], (int)CALIB_MAX[logicalId]);

    // Map Angle to Pulse
    int pulse = (int)SERVO_MIN + (corrected * ((int)SERVO_MAX - (int)SERVO_MIN) + 90) / 180;
    pulse = constrain(pulse, SERVO_MIN, SERVO_MAX);

    // Write to PWM
    uint8_t physicalPin = CALIB_INDEX[logicalId];
    pwm.setPWM(physicalPin, 0, pulse);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);
    
    Serial.println("IK Test Initialized");
}

void loop() {
    // --- Test Animation: Move Leg 0 (Front-Left) Up and Down ---
    
    // Dimensions
    // x = forward/backward
    // y = left/right (distance from center)
    // z = height (negative is down)
    
    // Static Position for other legs
    // (You should set them to a stable stand pose)

    static float phase = 0;
    phase += 0.05; // Speed
    
    // Circular Path for Leg 0
    float x_target = 0 + 30 * cos(phase); // Oscillate forward/back 30mm
    float z_target = -80 + 20 * sin(phase); // Lift up/down (center at -80mm height)
    float y_target = 60; // Fixed out to the side

    float t1, t2, t3;
    
    // Calculate IK for Leg 0
    if (inverseKinematics(x_target, y_target, z_target, t1, t2, t3, false)) {
        // Adjust angles to match your specific servo mounting zeros
        // These offsets depend heavily on your assembly!
        // Assuming:
        //   t1 (Hip): 90 is Neutral
        //   t2 (Thigh): 135 is roughly usually calibrated (tilted up?)
        //   t3 (Knee): 60 is bent?
        // You will likely need to tune 't2 + offset' and 't3 + offset' here
        
        setServo(0, t1);
        setServo(1, t2 + 45); // Trial offset! 
        setServo(2, 180 - t3); // Trial offset!
        
        Serial.print("IK Valid: ");
        Serial.print(t1); Serial.print(", ");
        Serial.print(t2); Serial.print(", ");
        Serial.println(t3);
    } else {
        Serial.println("IK Unreachable!");
    }
    
    delay(20);
}
