#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;

// Linear Position Control lib
// Servo lib 

/* WiFi network name and password */
const char * ssid = "Puddles-Mesh";
const char * pwd = "Whatever1";

// Parts list
// ----------
// 1. A very strong Servo to rotate the entire platform up to 360 degrees
// 2. A surround system with 5.1 channels
// 3. A PC capable of playing F1 2023 and streaming telemetry to my ESP32 device over UDP
// 4. A Thrustmaster force feedback steering wheel and pedals with an F1 rim 
// 5. A strip of ws2832 addressable lights for the spark simulator when the car bottoms out on the track and sparks
// 6. 4x linear actuators for lifting individual corners of the motion platform base for the servo to rest on
// 7. Control software to run on the ESP32.
// 8. 22 MOSFETs to amplify the ESP32 output voltage and current to be able to power the Actuators and servos. 
// 9. 4x additional suspension Linear actuators to sit on top of the motion platform to attach the individual car wheel suspension to.
// 10. Car Suspension sprung hydraulic shock absorbers to buffer the motion of the car.
// 11. Steering rack and servo to turn the front wheels.
// 12. 4x 12v electric motor to spin / lock up the front wheels for extra realism.
// 13. 6x 12v fans to simulate wind / breeze from motion in different directions.
// 14. 1x Simulated monocoque chassis unit (built from wooden compartments, filled and sanded with foam, then painted).
// 15. 1x front wing assembly ??? 
// 16. 1x DRS-enabled rear wing (opens via a small servo) ???
// 17. Suspension braces / shock absorber springs to attach the actual monocoque to the wheels and steering
// 18. 1x 12v smoke machine to simulate tyre smoke when spinning out
// 19. 1x 12v water pump to simulate water spray when driving in the rain 
// 20. 1x ButtKicker to simulate the rumble of the engine / haptic feedback
// 21. 1x 12v LED strip to simulate the rev lights on the steering wheel
// 22. 1x tricolour LED to simulate the DRS activation light on the steering wheel
// 23. 1x 12v RED WHITE LED Panel to simulate the brake / ERS harvesting lights on the car
// 24. 4x 18" low profile wheels and tyres to attach to the actuators
// 25. 1X Pit board display to show the current lap time and position
// 26. 4x haptic vibrator piezo under the seat to show rumble and ground effects 
// 27. 3x 37" curved monitors to display the game mounted as the cockpit surround
// 28. Note: Car length must be under 5.5m and less than 2m wide as per regs and less than 95cm high. 
// 39. The car's ground plate (ride height) must be 45mm off the floor before we pitch down
// So we need to have at least 50mm of clearance for the actuators to lift the car up and down on top of the platform

// Sensor readings needed to calculate and drive these
// ==========================================
// 1. Platform Yaw Servo input
// 2. ws2832 addressable light inputs for spark simulator
// 3. Linear actuator inputs for FL, FR, RL platform base elevation
// 4. Linear actuator inputs for FL, FR, RL, RR suspension elevation
// 5. Steering rack servo input
// 6. 4x Electric motor inputs for wheel spin / lockup
// 7. 3x 12v fans to simulate wind / breeze from motion FL, FR, Up
// 8. DRS-enabled rear wing servo
// 9. Smoke machine for lockups and spins
// 10. Water pump for rain mist
// 11. 11x Rev Lights LED strip
// 12. 1x tricolour LED to simulate the DRS activation light on the steering wheel (Off / Ready / Open)
// 13. 3x Red LED Panel rear lights on 2 control circuits (1x grid of LEDs 40cm from floor, 1x circuit to the RED LEDS on the 2x rear wing edges) 
//     (raining - all flash, Off, ERS Harvesting - 40cm rapid flash)
// 14. 4x mini ws2812 lights for tyre band colours (FL, FR, RL, RR) (White Hard, Yellow Medium, Red Soft, Blue Wet, Green Intermediate)
// 15. 1x Pit board display to show the current lap time and position in view of the driver on the right side of the cockpit
// 16. 4x Haptic vibrator piezo pad under the seat to show rumble and ground effects FL, FR, RL, RR of the wheel contact with the ground - 
//     should be Surface specific for effects, if the car skids or catches air, the haptic effect should stop on that corner to show the loss of grip 
// 17. 1x tricolour ERS Available, ERS Harvesting, ERS Deployed LED indicators 
//     (Red = all deployed, Green = Available, Blue = Harvesting, purple = Overtake)
// 18. Gear display for the wheel (1-8 + R) + Suggested Gear for the wheel + Delta to the actual gear 
// 19. Speedometer for the wheel (0-230 mph)

void setup()
{
    Serial.begin(9600);01299 848438
    WiFi.begin(ssid, password);
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED){
        delay(500); Serial.print(F("."));
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Listening for UDP packets...");
    Serial.printf("UDP server : %s:%i \n", WiFi.localIP().toString().c_str(), localPort);
}

void loop()
{
    while(1)
    {
        // Init these so we can set the reference when we receive the first packets

        PacketSessionData *sessionData = nullptr;
        PacketLapData *packetlapData = nullptr;
        PacketCarStatusData *statusData = nullptr;
        PacketMotionExData *motionExData = nullptr; 
        CarTelemetryData *carTelemetryData = nullptr;
        CarMotionData *carMotionData = nullptr;
        LapData *carlapData = nullptr;

        // processing incoming packet, must be called before reading the buffer
        int packetSize = udp.parsePacket();       
        if (packetSize) {
            char *buffer = (char *)malloc(packetSize + 1); // Allocate memory dynamically
            if (buffer == NULL) {
                Serial.println("Failed to allocate memory!");
                return;
            }
            // receive response from server
            int len = udp.read(buffer, packetSize);
            if (len > 0) {
                buffer[len] = '\0'; // Null-terminate the string
                Serial.print("Server to client: ");
                Serial.println(buffer);
            }
            free(buffer); // Free dynamically allocated memory
        }

        // Now decode the packet and see if we are interested in the data inside it. 

        // according to the F12023 doc every single packet has the same header PacketHeader
        // so we can use this to determine what type of packet we are dealing with
        // and then we can decode the rest of the packet accordingly.
        // The packet header is struct PacketHeader
        
        // So let's grabit from the buffer
        PacketHeader *header = (PacketHeader *)buffer;
        
        // Now we can check the packet id to see what type of packet we are dealing with
        switch(header->m_packetId)
        {
            // we care about : 
            // the CarMotionData packet (just the line for this car-> contains pitch and roll data for the wheel sim actuators and yaw servo motor)
            // the PacketMotionExData packet (extended motion data) (only for this car -> contains wheel_slip_Ratio and angle data for each wheel, front wheel angle data for the steering servo motor, and the height of the center of gravity above ground for the linear actuator, and local velocity data on all axis to drive the PWM fans for the breeze effect)
            // and the CarTelemetryData packet (again just the line for this car -> speed, steering, DRS servo motor, and the rev lights for the LED strip, also the driving surface type for the rumbling haptic feedback actuators)
            case PacketID::Motion:
            {
                CarMotionData *carMotionData = (PacketMotionData *)buffer;
                carMotionData = &motionData->m_carMotionData[header->m_playerCarIndex];
                break;
            }
            case PacketID::MotionEx:
            {
                motionExData = (PacketMotionExData *)buffer;
                break;
            }
            case PacketID::CarTelemetry :
            {
                PacketCarTelemetryData *telemetryData = (PacketCarTelemetryData *)buffer;
                carTelemetryData = &telemetryData->m_carTelemetryData[header->m_playerCarIndex];
                break;
            }
            case PacketID::PacketSessionData:
            {
                sessionData = (PacketSessionData *)buffer;
                break;
            }
            case PacketID::PacketCarStatusData:
            {
                statusData = (PacketCarStatusData *)buffer;
                break;
            }
            case PacketID::PacketLapData:
            {
                packetlapData = (PacketLapData *)buffer;
                // split out the correct lap data for my car
                lapData = &lapData->m_lapData[header->m_playerCarIndex];
                break;
            }
        }

        // At this point we don't know if we have enough data to process as that packet might not have been received yet so lets play safe and check if we have the data before we try to use it.
        if (carTelemetryData != nullptr) 
        {
            Serial.printLn("We have car telemetry data");
            // IS DRS open? 
            if (carTelemetryData->m_drs == 1) {
                Serial.printLn("DRS is open");
            }
            // What surface are we driving on? 
            switch(carTelemetryData->m_surfaceType[0])
            {
                case SurfaceType::Tarmac:
                    Serial.printLn("Driving on Tarmac");
                    break;
                case SurfaceType::Gravel:
                    Serial.printLn("Driving on Gravel");
                    break;
                case SurfaceType::Sand:
                    Serial.printLn("Driving on Sand");
                    break;
                case SurfaceType::Grass:
                    Serial.printLn("Driving on Grass");
                    break;
                case SurfaceType::RumbleStrip:
                    Serial.printLn("Driving on Rumble Strip");
                    break;
                case SurfaceType::Water:
                    Serial.printLn("Driving on Water");
                    break;
                case SurfaceType::Mud:
                    Serial.printLn("Driving on Mud");
                    break;
                case SurfaceType::Rock:
                    Serial.printLn("Driving on Rock");
                    break;
                case SurfaceType::Concrete:
                    Serial.printLn("Driving on Concrete");
                    break;
                case SurfaceType::Cobblestone:
                    Serial.printLn("Driving on Cobblestone");
                    break;
                case SurfaceType::Metal:
                    Serial.printLn("Driving on Metal");
                    break;
                case SurfaceType::Ridged:
                    Serial.printLn("Driving on Ridged");
                    break;
            }
            // integer RPM Level for the display on the wheel
            Serial.printLn("RPM Level: " + carTelemetryData->m_engineRPM);

            // bitwise leds for the rev light display 
            Serial.printLn("Rev Lights: " + carTelemetryData->m_revLightsBitValue);

            // Speed for the speedometer
            Serial.printLn("Speed: " + carTelemetryData->m_speed);

            // Throttle for the haptic feedback
            Serial.printLn("Throttle: " + carTelemetryData->m_throttle);

            // Brake for the haptic feedback
            Serial.printLn("Brake: " + carTelemetryData->m_brake);

            // Gear for the gear display
            Serial.printLn("Gear: " + carTelemetryData->m_gear);

            // suggested gear for the gear display
            Serial.printLn("Suggested Gear: " + telemetryData->m_suggestedGear);

            // Lateral g-force should tilt the car slightly in cornering compared to the platform rolling 
            // (i.e. the car should roll slightly more than the platform to simulate the g-force)
            Serial.printLn("Lateral-G: " + carMotionData->m_gForceLateral);

            // Longtitudinal g-force should rock the car back slightly in acceleration or pitch it forward slightly in braking
            // (i.e. the car should roll slightly more than the platform to simulate the g-force)
            Serial.printLn("Longitudinal-G: " + carMotionData->m_gForceLongitudinal);
            
            // Vertical g-force should lift the car slightly in bumps or compress it in dips
            // (This should simulate slightly more than the car's suspension moving up and down, giving you that 'wheeee' moment of catching air in a jump)
            Serial.printLn("VertICAL-G: " + carMotionData->m_gForceVertical);

        }

        if (carMotionData != nullptr) 
        {
            Serial.printLn("Car_Motion_Data:");
            Serial.printLn("Yaw: " + carMotionData->m_yaw);                         // we can use the yaw data to drive the servo motor for the rotation platform
            Serial.printLn("Pitch: " + carMotionData->m_pitch);                     // we can use the pitch
            Serial.printLn("Roll: " + carMotionData->m_roll);                       // we can use the roll
            Serial.printLn("Forward Breeze:" + carMotionData->m_worldVelocityX);
            Serial.printLn("Lateral Breeze:" + carMotionData->m_worldVelocityY);
            Serial.printLn("Vertical Breeze:" + carMotionData->m_worldVelocityZ);

            // Now push back for gforces, long, lat and vert to enhance the pitch and roll positioning of the actuators
            Serial.printLn("Lateral G-Force: " + carMotionData->m_gForceLateral);
            Serial.printLn("Longitudinal G-Force: " + carMotionData->m_gForceLongitudinal);
            Serial.printLn("Vertical G-Force: " + carMotionData->m_gForceVertical);
            
        }
        if (motionExData != nullptr) {

            // we can use the wheel_slip_Ratio and angle data for each wheel to drive the haptic feedback actuators
            Serial.printLn("Motion_Extended_Data:")
            
            // we can use the front wheel angle data to also drive the servo motor for the steering on the wheels 
            Serial.printLn("Front Wheel Angle:" + motionExData->m_frontWheelsAngle);
            
            // Check the wheel slip ratio values for each wheel.
            Serial.printLn("Wheel Slip Ratio Front Left:" + motionExData->m_wheelSlipRatio[0]);
            Serial.printLn("Wheel Slip Ratio Front Right:" + motionExData->m_wheelSlipRatio[1]);
            Serial.printLn("Wheel Slip Ratio Rear Left:" + motionExData->m_wheelSlipRatio[2]);
            Serial.printLn("Wheel Slip Ratio Rear Right:" + motionExData->m_wheelSlipRatio[3]);

            // Compare the rate of change in the car's yaw angle (m_yaw) to the steering input.

            int understeerindicators = 0;
            // Understeer Detection:
            // increased slip ratios for the front wheels + 1 
            if (motionExData->m_wheelSlipRatio[0] > 0.5) {
                understeerindicators++;
            }
            if (motionExData->m_wheelSlipRatio[1] > 0.5) {
                understeerindicators++;
            }

            // Understeer occurs when the front wheels lose grip and the car doesn't turn as sharply as intended. 
            // We can pick this up by insufficient lateral force at the front wheels compared to the back 
            // Monitor the lateral forces for each front wheel (m_wheelLatForce[0] and m_wheelLatForce[1]) and compare to the rear wheels.
            // Additionally, compare the lateral forces to the steering angle and the car's yaw rate to confirm understeer conditions.

            if(motionExData->m_wheelLatForce[0] < motionExData->m_wheelLatForce[2])
                understeerindicators++;
            
            if(motionExData->m_wheelLatForce[1] < motionExData->m_wheelLatForce[3])
                understeerindicators++;
    
            if (motionExData->m_yaw < 0 && motionExData->m_frontWheelsAngle > 0) {
                understeerindicators++;
            }


            // lastly if the rotational speeds are zero for any of the front wheels, whilst the rear is spinning, then we are locked up and understeering
            if(motionExData->m_wheelSpeed[0] == 0 || motionExData->m_wheelSpeed[1] == 0 && motionExData->m_wheelSpeed[2] > 0 || motionExData->m_wheelSpeed[3] > 0)
                understeerindicators++;

            Serial.printLn("Understeer Likeliness: " + motionExData->m_gForceLateral + "6");


            // Oversteer often manifests as increased slip ratios for the rear wheels, indicating a loss of traction and the potential for the rear of the car to slide out.
            // Oversteer Detection:
            // Oversteer occurs when the rear wheels lose grip and the rear of the car rotates more than intended. This typically results from excessive lateral force at the rear wheels.
            // Monitor the lateral forces for each rear wheel (m_wheelLatForce[2] and m_wheelLatForce[3]).
            // If the lateral forces at the rear wheels are significantly higher than those at the front wheels, it could indicate oversteer.
            // Additionally, compare the lateral forces to the car's yaw rate and the steering angle to confirm oversteer conditions.
            // If the yaw rate exceeds the rate expected based on the steering input, it could indicate oversteer.

            if(motionExData->m_wheelSlipRatio[2] > 0.5)
                oversteerindicators++;
            if(motionExData->m_wheelSlipRatio[3] > 0.5)
                oversteerindicators++;

            if(motionExData->m_wheelLatForce[2] > motionExData->m_wheelLatForce[0])
                oversteerindicators++;
            if(motionExData->m_wheelLatForce[3] > motionExData->m_wheelLatForce[1])
                oversteerindicators++;

            if (motionExData->m_yaw > 0 && motionExData->m_frontWheelsAngle < 0) {
                oversteerindicators++;
            }

            // If one of the rear wheels is spinning sigificantly faster than the other, then we are probably oversteering 
            // or maybe spinning out into a doughnut (trigger the smoke machine ;))

            if(motionExData->m_wheelSpeed[2] > motionExData->m_wheelSpeed[3] * 1.5 || motionExData->m_wheelSpeed[3] > motionExData->m_wheelSpeed[2] * 1.5)
                oversteerindicators++;

            Serial.printLn("Oversteer Likeliness: " + motionExData->m_gForceLateral + "/5");
            
            // We can use the height of the center of gravity above ground to drive the lighting for the spark simulator
            Serial.printLn("Center of Gravity Height: " + motionExData->m_centerOfGravityHeight);

            // Longitudinal Force Analysis:

            // The longitudinal forces (m_wheelLongForce) can also provide insights into the car's dynamics, particularly during acceleration and braking.
            // Analyze the longitudinal forces at each wheel to detect wheelspin 

            // Wheelspin first (longitudinal force WITHOUT corresponding forward motion)
            Serial.printLn("Rear left spinning: " + motionExData->m_wheelLongForce[2] > 1);
            Serial.printLn("Rear right spinning: " + motionExData->m_wheelLongForce[3] > 1);
           
            // Lockup (braking force without corresponding wheel rotation).
            Serial.printLn("Front left locked up: " + motionExData->m_wheelLongForce[0] < 0 && motionExData->m_wheelSpeed[0] == 0);
            Serial.printLn("Front right locked up: " + motionExData->m_wheelLongForce[1] < 0 && motionExData->m_wheelSpeed[1] == 0);

            // Braking force distribution (compare the front and rear wheels).
            Serial.printLn("Front brake force: " + motionExData->m_wheelLongForce[0] + motionExData->m_wheelLongForce[1]);
            Serial.printLn("Rear brake force: " + motionExData->m_wheelLongForce[2] + motionExData->m_wheelLongForce[3]);

            // Weather Data as text description
            switch(sessionData->m_weather)
            {
                case 0:
                    Serial.printLn("Weather: Clear");
                    break;
                case 1:
                    Serial.printLn("Weather: Light Clouds");
                    break;
                case 2:
                    Serial.printLn("Weather: Overcast");
                    break;
                case 3:
                    Serial.printLn("Weather: Light Rain");
                    break;
                case 4:
                    Serial.printLn("Weather: Heavy Rain");
                    break;
                case 5:
                    Serial.printLn("Weather: Storm");
                    break;
                case 6:
                    Serial.printLn("Weather: Fog");
                    break;
                case 7:
                    Serial.printLn("Weather: Hail");
                    break;
                case 8:
                    Serial.printLn("Weather: Snow");
                    break;
            }

            // DRS Available (PacketCarStatusData=>m_drsAllowed)
            // DRS Active (PacketCarTelemetryData=>m_drs)

            Serial.printLn("DRS Allowed: " + statusData->m_drsAllowed);
            Serial.printLn("DRS Active: " + carTelemetryData->m_drs);

        }

        // So now we confirmed we have the needed data for the calculations
        // We can figure out what to do with the various actuators, servos and mosfets
        // to drive the motion platform and the various effects we want to simulate.

        // Sensor readings needed to calculate the outputs
        // ===============================================
        // 1. Platform Yaw Servo input
        // 2. Steering rack servo input
        // 3. DRS-enabled rear wing servo 
        // 4. Electric motor inputs for front wheel spin / lockup
        // 5. Linear actuator inputs for FL, FR, RL, RR suspension elevation
        // 6. Linear actuator inputs for FL, FR, RL platform base elevation
        // 7. 3x 12v fans to simulate wind / breeze from motion FL, FR, Up
        // 8. ws2832 addressable light inputs for spark simulator
        // 9. Smoke machine
        // 10. Water pump
        // 11. 11x Rev Lights LED strip
        // 12. 1x tricolour LED to simulate the DRS activation light on the steering wheel (Off / Ready / Open)
        // 13. 3x Red LED Panel rear lights on 2 control circuits (1x grid of LEDs 40cm from floor, 1x circuit to the RED LEDS on the 2x rear wing edges) (raining - all flash, Off, ERS Harvesting - 40cm rapid flash)
        // 14. 4x mini ws2812 lights for tyre band colours (FL, FR, RL, RR) (White Hard, Yellow Medium, Red Soft, Blue Wet, Green Intermediate)
        // 15. 1x Pit board external display to show the current lap time and position in view of the driver on the right side of the cockpit
        // 16. 4x Haptic vibrator piezo pad under the seat to show rumble and ground effects FL, FR, RL, RR of the wheel contact with the ground - should be Surface specific for effects, if the car skids or catches air, the haptic effect should stop on that corner to show the loss of grip 

        // We can use the pitch and roll data for the platform actuators and yaw for the main platform servo motor
        // Calculate feed into the Yaw servo motor

        // yaw, pitch and roll are in radians
        // we need to convert them to degrees
        int yawdeg = carMotionData->m_yaw * 180 / 3.14159265359;
        int pitchdeg = carMotionData->m_pitch * 180 / 3.14159265359;
        int rolldeg = carMotionData->m_roll * 180 / 3.14159265359;

        // Steering rack servo input
        int steerdeg = map(carTelemetryData->m_steer, -1, 1, -45, 45);

        // DRS enabler servo 
        int drsdeg = 70 + (carTelemetryData->m_drs * 40); // 70 degrees == off (closed) or 85 degrees == open (enabled)
        if(carTelemetryData->m_drsFault != 0 )
            drsdeg = 70; 

        
        // Electric motor inputs for front wheel spin / lockup
        int FLmotor = carTelemetryData->m_wheelSpeed[0];
        int FRmotor = carTelemetryData->m_wheelSpeed[1];
        int RLmotor = carTelemetryData->m_wheelSpeed[2];
        int RRmotor = carTelemetryData->m_wheelSpeed[3];

        // Linear actuator inputs for FL, FR, RL, RR suspension elevation
        int FLSuspensionHeight = map(motionExData->m_suspensionPosition[0], 0, 100, 0, 255);
        int FRSuspensionHeight = map(motionExData->m_suspensionPosition[1], 0, 100, 0, 255);
        int RLSuspensionHeight = map(motionExData->m_suspensionPosition[2], 0, 100, 0, 255);
        int RRSuspensionHeight = map(motionExData->m_suspensionPosition[3], 0, 100, 0, 255);

        // Linear actuator inputs for FL, FR, RL platform base elevation from the ground (pitch and roll)
        // carMotionData->m_pitch and carMotionData->m_roll

        // Calculate the relative height of the platform base from the ground at each corner actuator using the float pitch and roll measurements
        // IF FL is the front left corner, FR is the front right corner, RL is the rear left corner, and RR is the rear right corner
        // We can calculate the height of each corner as the base height + an adjustment for the pitch and roll values
        // So if the car is pitched forwards (nose down) then the front corners will be higher than the rear corners
        // If the car is rolled to the left then the left corners will be higher than the right corners
        // We can use the pitch and roll values to adjust the height of each corner actuator to simulate the car's pitch and roll in the game

        int FLPlatformHeight = 128; // centered at 128
        int FRPlatformHeight = 128; // centered at 128
        int RLPlatformHeight = 128; // centered at 128
        int RRPlatformHeight = 128; // centered at 128

        // Now adjust the relative height based on the pitch and roll values
        FLPlatformHeight += carMotionData->m_pitch * 10; // 10 is the scaling factor for the pitch
        FRPlatformHeight += carMotionData->m_pitch * 10; // 10 is the scaling factor for the pitch
        RLPlatformHeight += carMotionData->m_pitch * 10; // 10 is the scaling factor for the pitch
        RRPlatformHeight += carMotionData->m_pitch * 10; // 10 is the scaling factor for the pitch

        FLPlatformHeight += carMotionData->m_roll * 10; // 10 is the scaling factor for the roll
        FRPlatformHeight -= carMotionData->m_roll * 10; // 10 is the scaling factor for the roll
        RLPlatformHeight -= carMotionData->m_roll * 10; // 10 is the scaling factor for the roll
        RRPlatformHeight += carMotionData->m_roll * 10; // 10 is the scaling factor for the roll

        // 3x 12v fans to simulate wind / breeze from motion FL, FR, Up
        // We can use the local velocity data on the x and y axis to drive the PWM fans for the breeze effect as we are moving
        int FLBreezeStrength = map((carMotionData->m_worldVelocityX + carMotionData->m_worldVelocityY), -1, 1, 0, 255);
        int FRBreezeStrength = map((carMotionData->m_worldVelocityX + carMotionData->m_worldVelocityY), -1, 1, 0, 255);

        // and the z-axis to drive the floor fan, if we are moving upwards in the world then we are catching air and should 
        // feel a breeze from below
        int UpBreezeStrength = map(carMotionData->m_worldVelocityZ, -1, 1, 0, 255);

        // ws2832 addressable light inputs for spark simulator
        // We can use the suspension travel and CG value to detect if the vehicle is bottoming out and will spark             
        int SparkHeight = 0;
        if (motionExData->m_suspensionPosition[0] < 10 || motionExData->m_suspensionPosition[1] < 10 || motionExData->m_suspensionPosition[2] < 10 || motionExData->m_suspensionPosition[3] < 10)
        {
            // Ok so we should spark, but how much?
            // We can use the center of gravity height to determine how much the car is bottoming out
            // If the car is bottoming out then we should spark more
            SparkHeight = map(motionExData->m_centerOfGravityHeight, 0, 100, 0, 255);

        }
        
        // Smoke machine
        // We can use the wheel slip ratio values for each wheel to drive the smoke machine
        int SmokeLevel = 0;
        if (motionExData->m_wheelSlipRatio[0] > 0.5 || motionExData->m_wheelSlipRatio[1] > 0.5 || motionExData->m_wheelSlipRatio[2] > 0.5 || motionExData->m_wheelSlipRatio[3] > 0.5)
        {
            // Ok so we should smoke, but how much?
            // We can use the wheel slip ratio to determine how much the car is spinning out
            // If the car is spinning out then we should smoke more
            SmokeLevel = map(motionExData->m_wheelSlipRatio[0] + motionExData->m_wheelSlipRatio[1] + motionExData->m_wheelSlipRatio[2] + motionExData->m_wheelSlipRatio[3], 0, 4, 0, 255);
        }

        // Water pump
        int rainIntensity = 0;

        // We can use the weather value to drive the water 
        // if its greater than =2 then we should spray water unless fog (6)
        if (sessionData->m_weather >= 2 && sessionData->m_weather != 6)
        {
            // Ok so we should spray water, but how much?
            // We can use the weather value to determine how much water to spray
            // If the weather is heavy rain then we should spray more water
            rainIntensity = (sessionData->m_weather - 3) * 30; // 0 = clear, 30 = light rain, 60 = heavy rain, 90 = storm, 150 = hail, 180 = snow
        }
        
        // 11x Rev Lights LED strip
        // use the bitwise RPM Level for the display on the wheel
        bool RPMLed0 = carTelemetryData->m_revLightsBitValue & 1;
        bool RPMLed1 = carTelemetryData->m_revLightsBitValue & 2;
        bool RPMLed2 = carTelemetryData->m_revLightsBitValue & 4;
        bool RPMLed3 = carTelemetryData->m_revLightsBitValue & 8;
        bool RPMLed4 = carTelemetryData->m_revLightsBitValue & 16;
        bool RPMLed5 = carTelemetryData->m_revLightsBitValue & 32;
        bool RPMLed6 = carTelemetryData->m_revLightsBitValue & 64;
        bool RPMLed7 = carTelemetryData->m_revLightsBitValue & 128;
        bool RPMLed8 = carTelemetryData->m_revLightsBitValue & 256;
        bool RPMLed9 = carTelemetryData->m_revLightsBitValue & 512; 
        bool RPMLed10 = carTelemetryData->m_revLightsBitValue & 1024;
        
        // 1x tricolour LED to simulate the DRS activation light on the steering wheel (Off = disabled / Ready = purple / Open = green, orange = FAULT)
        // 0 = off, 1 = ready, 2 = open
        int DRSLED = 0; // disabled

        int DRSLedBlue = 0;
        int DRSLedGreen = 0;
        int DRSLedRed = 0;

        if (statusData->m_drsAllowed == 1) 
        {
            // colour the LED purple -- DRS is enabled
            DRSLED = 1;
            DRSLedBlue = 255;
            DRSLedGreen = 0;
            DRSLedRed = 255;
        }
        if (carTelemetryData->m_drs == 1) 
        {    
            // colour the LED green -- DRS is open
            DRSLED = 2;
            DRSLedBlue = 0;
            DRSLedGreen = 255;
            DRSLedRed = 0;
        }
        if (carTelemetryData->m_drsFault != 0) 
        {
            // colour the LED orange -- DRS Has a fault
            DRSLED = 3;
            DRSLedBlue = 255;
            DRSLedGreen = 165;
            DRSLedRed = 0;
        }

        // 3x Red LED Panel rear lights on 2 control circuits 
        // (1x grid of LEDs 40cm from floor, 1x circuit to the RED LEDS on the 2x rear wing edges) 
        // (raining - all flash, Off, ERS Harvesting - 40cm rapid flash)

        int LEDGridFlash = 0
        int LEDWingFlash = 0;

        if (sessionData->m_weather >= 3){
            // it's poor viz all flash slowly
            LEDGridFlash = 1;
            LEDWingFlash = 1;
        }
        if (sessionData->m_weather >= 3)
        {
            // ERS Harvesting - so 40cm block rapid flash
            LEDGridFlash = 2;
        }

        // 4x mini ws2812 lights for tyre band colours (FL, FR, RL, RR)
        // (White Hard, Yellow Medium, Red Soft, Blue Wet, Green Intermediate)
        int TyreColourR = 0;
        int TyreColourG = 0;
        int TyreColourB = 0;

        // read the CarStatusData for m_visualTyreCompound
        // 7 = inter (GREEN), 8 = wet (BLUE), 9 = dry (GREY), 10 = wet (BLUE), 11 = super soft (PINK), 
        // 12 = soft (RED), 13 = medium (YELLOW) , 14 = hard (WHITE), 15 = wet (BLUE), 16 = soft (RED), 
        // 17 = medium (YELLOW), 18 = hard (WHITE),  19 = super soft (PINK), 20 = soft (RED), 21 = medium (YELLOW) , 22 = hard (White)

        switch(statusData=>m_visualTyreCompound)
        {
            case: 7:
                TyreColourR = 0;
                TyreColourG = 255;
                TyreColourB = 0;
                break;
            case: 8:
                TyreColourR = 0;
                TyreColourG = 0;
                TyreColourB = 255;
                break;
            case: 9:
                TyreColourR = 64;
                TyreColourG = 64;
                TyreColourB = 64;
                break;
            case: 10:
                TyreColourR = 0;
                TyreColourG = 0;
                TyreColourB = 255;
                break;
            case: 11:
                TyreColourR = 255;
                TyreColourG = 0;
                TyreColourB = 255;
                break;
            case: 12:
                TyreColourR = 255;
                TyreColourG = 0;
                TyreColourB = 0;
                break;
            case: 13:
                TyreColourR = 255;
                TyreColourG = 255;
                TyreColourB = 0;
                break;
            case: 14:
                TyreColourR = 255;
                TyreColourG = 255;
                TyreColourB = 255;
                break;
            case: 15:
                TyreColourR = 0;
                TyreColourG = 0;
                TyreColourB = 255;
                break;
            case: 16:
                TyreColourR = 255;
                TyreColourG = 0;
                TyreColourB = 0;
                break;
            case: 17:
                TyreColourR = 255;
                TyreColourG = 255;
                TyreColourB = 0;
                break;
            case: 18:
                TyreColourR = 255;
                TyreColourG = 255;
                TyreColourB = 255;
                break;
            case: 19:
                TyreColourR = 255;
                TyreColourG = 0;
                TyreColourB = 255;
                break;
            case: 20:
                TyreColourR = 255;
                TyreColourG = 0;
                TyreColourB = 0;
                break;
            case: 21:
                TyreColourR = 255;
                TyreColourG = 255;
                TyreColourB = 0;
                break;
            case: 22:
                TyreColourR = 255;
                TyreColourG = 255;
                TyreColourB = 255;
                break;
            default:
                TyreColourR = 64;
                TyreColourG = 64;
                TyreColourB = 64;
                break;
        }

        // 4x Haptic vibrator piezo pad under the seat to show rumble and ground effects FL, FR, RL, RR 
        // of the wheel contact with the ground - should be Surface specific for effects, 
        // if the car skids or catches air, the haptic effect should stop on that corner, to show the loss of grip

        // We can use the surface type data to drive the haptic feedback actuators
        // We can use the wheel_slip_Ratio and angle data for each wheel to drive the haptic feedback actuators
        // 1 - 255 for the haptic feedback actuators
        // -1 to -255 for intermittent feedback to simulate bumps

        // 0 for no feedback (air or water / mudslide)
        uint_8 intermittentFeedbackPeriod = 25; // ms

        int FLHaptic = 0;
        int FRHaptic = 0;
        int RLHaptic = 0;
        int RRHaptic = 0;

        // Check the surface type for each wheel and set the haptic feedback accordingly
        FLHaptic = setHapticSurface(carTelemetryData->m_surfaceType[0], *intermittentFeedbackPeriod, FLPlatformHeight);
        FRHaptic = setHapticSurface(carTelemetryData->m_surfaceType[1], *intermittentFeedbackPeriod, FRPlatformHeight);
        RLHaptic = setHapticSurface(carTelemetryData->m_surfaceType[2], *intermittentFeedbackPeriod, RLPlatformHeight);
        RRHaptic = setHapticSurface(carTelemetryData->m_surfaceType[3], *intermittentFeedbackPeriod, RRPlatformHeight);

        // 1x tricolour ERS Available, ERS Harvesting, ERS Deployed LED indicators 
        // (off = all deployed, Green = Available, Blue = Harvesting, red = hotlap, purple = Overtake)
        int ERSLEDblue = 0;
        int ERSLEDgreen = 0;
        int ERSLEDred = 0;

        if (carTelemetryData->m_drsFault == 0)
        {
            if (carTelemetryData->m_ersStoreEnergy > 0) // available energy
            {
                if (carTelemetryData->m_ersDeployMode == 2) // hotlap
                {
                    ERSLEDred = 255;
                    ERSLEDblue = 0;
                    ERSLEDgreen = 0;
                }
                else if (carTelemetryData->return  == 3) // overtake
                {
                    ERSLEDred = 255;
                    ERSLEDblue = 255;
                    ERSLEDgreen = 0;    

                }
                else // normal - energy available
                {
                    ERSLEDblue = 0;
                    ERSLEDgreen = 255;
                    ERSLedred = 0;
                }
            }
            else // depleted store - off regardless of mode 
            {
                ERSLEDblue = 0;
                ERSLEDgreen = 0;
                ERSLEDred = 0;
            }
        else
        {
            // ERS Fault - light up orange
            ERSLEDblue = 255;
            ERSLEDgreen = 128;
            ERSLEDred = 0;
        }

        // 18. Gear display for the in-wheel display (1-8 + R) + Suggested Gear for the wheel + Delta to the actual gear 
        // 19. Speedometer for the in-wheel display (0-230 mph)
        // 20. Last Lap time, Current Lap Time, Best Lap Time for the in-wheel display (0:00.000 - 9:59.999)
        // 21. Delta to my best lap time for the in-wheel display (0:00.000 - 9:59.999)
        // 22. Delta to the leader for the in-wheel display (0:00.000 - 9:59.999)

        // Mini 18.3 x 6.8 x 7cm display with extra buttons
        // 1x display to show the current lap time and position in view of the driver on the right side of the cockpit
        
        // Display Layout 
        //   DRS            RPM             ERS
        //   (*)   * * * * * * * * * * *    (*) 
        //---------------------------------------
        // G:1/5 (+3) S:120mph F:20 FR RR RL FL 
        // Lap 01/05 Lst 1:20.000 Bst 1:18.000 In 
        // S1:30.000 S2:40.000 S3:30.000 Frn:7.3 Ldr:2.0 -- blank, yellow or green or purple -- Sector Graphics
        //---------------------------------------



        int currentGear = carTelemetryData->m_gear;
        int suggestedGear = telemetryData->m_suggestedGear;
        int gearDelta = suggestedGear - currentGear;
        int speed = carTelemetryData->m_speed;

        int position = sessionData->m_carPosition;
        
        int sector1time = lapData->m_sector1TimeInMS;
        int sector2time = lapData->m_sector2TimeInMS;
        int sector3time = lapData->m_sector3TimeInMS;
        int lastLapTime = lapData->m_lastLapTimeInMS;
        int currentLapTime = lapData->m_currentLapTimeInMS;

        int deltaToCarInFront = lapData->m_deltaToCarInFrontInMS;
        int deltaToRaceLeader = lapData->m_deltaToRaceLeaderInMS;

        int currentLap = sessionData->m_currentLap;
        int totalLaps = sessionData->m_totalLaps;

        int fuelLapsLeft = carStatusData->m_fuelRemainingLaps;

        int FLTyreWearPer = carStatusData->m_tyresWear[0];
        int FRTyreWearPer = carStatusData->m_tyresWear[1];
        int RLTyreWearPer = carStatusData->m_tyresWear[2];
        int RRTyreWearPer = carStatusData->m_tyresWear[3];

        
        delay(10); // Add a small delay to avoid excessive CPU usage to keep the system cool
    }
}
int setHapticSurface(uint_8 surface, *uint_8 intermittent, int PlatformHeightCorner)
{

        // if the suspension is that extended then we are probably in the air over a crest or upside down and should not have any feedback
        if (PlatformHeightCorner > 250)
        {
            return 0;
        }

        // Check the surface type for each wheel and set the haptic feedback accordingly
        switch(carTelemetryData->m_surfaceType[0])
        {
            case SurfaceType::Tarmac:
                return 5;
                break;
            case SurfaceType::Gravel:
                return 192
                intermitentFeedbackPeriod = 3;
                break;
            case SurfaceType::Sand:
                return 128;
                break;
            case SurfaceType::Grass:
                return 64;
                break;
            case SurfaceType::RumbleStrip:
                return 255;
                intermitentFeedbackPeriod = 2;
                break;
            case SurfaceType::Water:
                return -2;
                intermitentFeedbackPeriod = 50;
                break;
            case SurfaceType::Mud:
                return 3;
                break;
            case SurfaceType::Rock:
                return -120;
                intermitentFeedbackPeriod = 5;
                break;
            case SurfaceType::Concrete:
                return 20;
                break;
            case SurfaceType::Cobblestone:
                FLHaptic =-128;
                intermitentFeedbackPeriod = 128;
                break;
            case SurfaceType::Metal:
                return 1;
                break;
            case SurfaceType::Ridged:
                return -256;
                intermitentFeedbackPeriod = 64;
                break;
        }
}

// Surface types, These types are from physics data and show what type of contact each wheel is experiencing.
enum SurfaceType : uint8_t
{
    Tarmac = 0,
    RumbleStrip = 1,
    Concrete = 2,
    Rock = 3,
    Gravel = 4,
    Mud = 5,
    Sand = 6,
    Grass = 7,
    Water = 8,
    Cobblestone = 9,
    Metal = 10,
    Ridged = 11
};

enum PacketID : uint8_t
{
    Motion = 0,              // Contains all motion data for player’s car – only sent while player is in control
    Session = 1,             // Data about the session – track, time left
    LapData = 2,             // Data about all the lap times of cars in the session
    Event = 3,               // Various notable events that happen during a session
    Participants = 4,        // List of participants in the session, mostly relevant for multiplayer
    CarSetups = 5,           // Packet detailing car setups for cars in the race
    CarTelemetry = 6,        // Telemetry data for all cars
    CarStatus = 7,           // Status data for all cars
    FinalClassification = 8, // Final classification confirmation at the end of a race
    LobbyInfo = 9,           // Information about players in a multiplayer lobby
    CarDamage = 10,          // Damage status for all cars
    SessionHistory = 11,     // Lap and tyre data for session
    TyreSets = 12,           // Extended tyre set data
    MotionEx = 13            // Extended motion data for player car
};

// Packet Header : Each packet has the following header
struct PacketHeader
{
    uint16_t m_packetFormat;           // 2023
    uint8_t m_gameYear;                // Game year - last two digits e.g. 23
    uint8_t m_gameMajorVersion;        // Game major version - "X.00"
    uint8_t m_gameMinorVersion;        // Game minor version - "1.XX"
    uint8_t m_packetVersion;           // Version of this packet type, all start from 1
    uint8_t m_packetId;                // Identifier for the packet type, see below
    uint64_t m_sessionUID;             // Unique identifier for the session
    float m_sessionTime;               // Session timestamp
    uint32_t m_frameIdentifier;        // Identifier for the frame the data was retrieved on
    uint32_t m_overallFrameIdentifier; // Overall identifier for the frame the data was retrieved on, doesn't go back after flashbacks
    uint8_t m_playerCarIndex;          // Index of player's car in the array
    uint8_t m_secondaryPlayerCarIndex; // Index of secondary player's car in the array (splitscreen), 255 if no second player
};

// Motion Packet
// The motion packet gives physics data for all the cars being driven.For the normalized vectors below, to convert to float values divide by 32767.0f – 16-bit signed values are used to pack the data and on the assumption that direction values are always between -1.0f and 1.0f.
struct CarMotionData
{
    float m_worldPositionX;     // World space X position - meters
    float m_worldPositionY;     // World space Y position
    float m_worldPositionZ;     // World space Z position
    float m_worldVelocityX;     // Velocity in world space X – meters/s
    float m_worldVelocityY;     // Velocity in world space Y
    float m_worldVelocityZ;     // Velocity in world space Z
    int16_t m_worldForwardDirX; // World space forward X direction (normalized)
    int16_t m_worldForwardDirY; // World space forward Y direction (normalized)
    int16_t m_worldForwardDirZ; // World space forward Z direction (normalized)
    int16_t m_worldRightDirX;   // World space right X direction (normalized)
    int16_t m_worldRightDirY;   // World space right Y direction (normalized)
    int16_t m_worldRightDirZ;   // World space right Z direction (normalized)
    float m_gForceLateral;      // Lateral G-Force component
    float m_gForceLongitudinal; // Longitudinal G-Force component
    float m_gForceVertical;     // Vertical G-Force component
    float m_yaw;                // Yaw angle in radians
    float m_pitch;              // Pitch angle in radians
    float m_roll;               // Roll angle in radians
};

// Define the structure for motion packet data
struct PacketMotionData
{
    PacketHeader m_header;             // Header
    CarMotionData m_carMotionData[22]; // Data for all cars on track
};

// Session Packet - The session packet includes details about the current session in progress.

struct MarshalZone
{
    float m_zoneStart; // Fraction (0..1) of way through the lap the marshal zone starts
    int8_t m_zoneFlag; // -1 = invalid/unknown, 0 = none, 1 = green, 2 = blue, 3 = yellow
};

struct WeatherForecastSample
{
    uint8_t m_sessionType;           // 0 = unknown, 1 = P1, 2 = P2, 3 = P3, 4 = Short P, 5 = Q1, 6 = Q2, 7 = Q3, 8 = Short Q, 9 = OSQ, 10 = R, 11 = R2 , 12 = R3, 13 = Time Trial
    uint8_t m_timeOffset;            // Time in minutes the forecast is for
    uint8_t m_weather;               // Weather - 0 = clear, 1 = light cloud, 2 = overcast 3 = light rain, 4 = heavy rain, 5 = storm
    int8_t m_trackTemperature;       // Track temp. in degrees Celsius
    int8_t m_trackTemperatureChange; // Track temp. change – 0 = up, 1 = down, 2 = no change
    int8_t m_airTemperature;         // Air temp. in degrees celsius
    int8_t m_airTemperatureChange;   // Air temp. change – 0 = up, 1 = down, 2 = no change
    uint8_t m_rainPercentage;        // Rain percentage (0-100)
};

struct PacketSessionData
{
    PacketHeader m_header;                              // Header
    uint8_t m_weather;                                  // Weather - 0 = clear, 1 = light cloud, 2 = overcast, 3 = light rain, 4 = heavy rain, 5 = storm
    int8_t m_trackTemperature;                          // Track temp. in degrees celsius
    int8_t m_airTemperature;                            // Air temp. in degrees celsius
    uint8_t m_totalLaps;                                // Total number of laps in this race
    uint16_t m_trackLength;                             // Track length in metres
    uint8_t m_sessionType;                              // 0 = unknown, 1 = P1, 2 = P2, 3 = P3, 4 = Short P, 5 = Q1, 6 = Q2, 7 = Q3, 8 = Short Q, 9 = OSQ, 10 = R, 11 = R2, 12 = R3, 13 = Time Trial
    int8_t m_trackId;                                   // -1 for unknown, see appendix
    uint8_t m_formula;                                  // Formula, 0 = F1 Modern, 1 = F1 Classic, 2 = F2, 3 = F1 Generic, 4 = Beta, 5 = Supercars, 6 = Esports, 7 = F2 2021
    uint16_t m_sessionTimeLeft;                         // Time left in session in seconds
    uint16_t m_sessionDuration;                         // Session duration in seconds
    uint8_t m_pitSpeedLimit;                            // Pit speed limit in kilometres per hour
    uint8_t m_gamePaused;                               // Whether the game is paused – network game only
    uint8_t m_isSpectating;                             // Whether the player is spectating
    uint8_t m_spectatorCarIndex;                        // Index of the car being spectated
    uint8_t m_sliProNativeSupport;                      // SLI Pro support, 0 = inactive, 1 = active
    uint8_t m_numMarshalZones;                          // Number of marshal zones to follow
    MarshalZone m_marshalZones[21];                     // List of marshal zones – max 21
    uint8_t m_safetyCarStatus;                          // 0 = no safety car, 1 = full, 2 = virtual, 3 = formation lap
    uint8_t m_networkGame;                              // 0 = offline, 1 = online
    uint8_t m_numWeatherForecastSamples;                // Number of weather samples to follow
    WeatherForecastSample m_weatherForecastSamples[56]; // Array of weather forecast samples
    uint8_t m_forecastAccuracy;                         // 0 = Perfect, 1 = Approximate
    uint8_t m_aiDifficulty;                             // AI Difficulty rating – 0-110
    uint32_t m_seasonLinkIdentifier;                    // Identifier for season - persists across saves
    uint32_t m_weekendLinkIdentifier;                   // Identifier for weekend - persists across saves
    uint32_t m_sessionLinkIdentifier;                   // Identifier for session - persists across saves
    uint8_t m_pitStopWindowIdealLap;                    // Ideal lap to pit on for current strategy (player)
    uint8_t m_pitStopWindowLatestLap;                   // Latest lap to pit on for current strategy (player)
    uint8_t m_pitStopRejoinPosition;                    // Predicted position to rejoin at (player)
    uint8_t m_steeringAssist;                           // 0 = off, 1 = on
    uint8_t m_brakingAssist;                            // 0 = off, 1 = low, 2 = medium, 3 = high
    uint8_t m_gearboxAssist;                            // 1 = manual, 2 = manual & suggested gear, 3 = auto
    uint8_t m_pitAssist;                                // 0 = off, 1 = on
    uint8_t m_pitReleaseAssist;                         // 0 = off, 1 = on
    uint8_t m_ERSAssist;                                // 0 = off, 1 = on
    uint8_t m_DRSAssist;                                // 0 = off, 1 = on
    uint8_t m_dynamicRacingLine;                        // 0 = off, 1 = corners only, 2 = full
    uint8_t m_dynamicRacingLineType;                    // 0 = 2D, 1 = 3D
    uint8_t m_gameMode;                                 // Game mode id - see appendix
    uint8_t m_ruleSet;                                  // Ruleset - see appendix
    uint32_t m_timeOfDay;                               // Local time of day - minutes since midnight
    uint8_t m_sessionLength;                            // 0 = None, 2 = Very Short, 3 = Short, 4 = Medium, 5 = Medium Long, 6 = Long, 7 = Full
    uint8_t m_speedUnitsLeadPlayer;                     // 0 = MPH, 1 = KPH
    uint8_t m_temperatureUnitsLeadPlayer;               // 0 = Celsius, 1 = Fahrenheit
    uint8_t m_speedUnitsSecondaryPlayer;                // 0 = MPH, 1 = KPH
    uint8_t m_temperatureUnitsSecondaryPlayer;          // 0 = Celsius, 1 = Fahrenheit
    uint8_t m_numSafetyCarPeriods;                      // Number of safety cars called during session
    uint8_t m_numVirtualSafetyCarPeriods;               // Number of virtual safety cars called
    uint8_t m_numRedFlagPeriods;                        // Number of red flags called during session
};


// Car Setups Packet - This packet details the car setups for each vehicle in the session. Note that in multiplayer games, other player cars will appear as blank, you will only be able to see your own car setup, regardless of the “Your Telemetry” setting. Spectators will also not be able to see any car setups.
struct CarSetupData
{
    uint8_t m_frontWing;             // Front wing aero
    uint8_t m_rearWing;              // Rear wing aero
    uint8_t m_onThrottle;            // Differential adjustment on throttle (percentage)
    uint8_t m_offThrottle;           // Differential adjustment off throttle (percentage)
    float m_frontCamber;             // Front camber angle (suspension geometry)
    float m_rearCamber;              // Rear camber angle (suspension geometry)
    float m_frontToe;                // Front toe angle (suspension geometry)
    float m_rearToe;                 // Rear toe angle (suspension geometry)
    uint8_t m_frontSuspension;       // Front suspension
    uint8_t m_rearSuspension;        // Rear suspension
    uint8_t m_frontAntiRollBar;      // Front anti-roll bar
    uint8_t m_rearAntiRollBar;       // Front anti-roll bar
    uint8_t m_frontSuspensionHeight; // Front ride height
    uint8_t m_rearSuspensionHeight;  // Rear ride height
    uint8_t m_brakePressure;         // Brake pressure (percentage)
    uint8_t m_brakeBias;             // Brake bias (percentage)
    float m_rearLeftTyrePressure;    // Rear left tyre pressure (PSI)
    float m_rearRightTyrePressure;   // Rear right tyre pressure (PSI)
    float m_frontLeftTyrePressure;   // Front left tyre pressure (PSI)
    float m_frontRightTyrePressure;  // Front right tyre pressure (PSI)
    uint8_t m_ballast;               // Ballast
    float m_fuelLoad;                // Fuel load
};
struct PacketCarSetupData
{
    PacketHeader m_header; // Header
    CarSetupData m_carSetups[22];
};

// Car Telemetry Packet- This packet details telemetry for all the cars in the race. It details various values that would be recorded on the car such as speed, throttle application, DRS etc. Note that the rev light configurations are presented separately as well and will mimic real life driver preferences.
struct CarTelemetryData
{
    uint16_t m_speed;                     // Speed of car in kilometres per hour
    float m_throttle;                     // Amount of throttle applied (0.0 to 1.0)
    float m_steer;                        // Steering (-1.0 (full lock left) to 1.0 (full lock right))
    float m_brake;                        // Amount of brake applied (0.0 to 1.0)
    uint8_t m_clutch;                     // Amount of clutch applied (0 to 100)
    int8_t m_gear;                        // Gear selected (1-8, N=0, R=-1)
    uint16_t m_engineRPM;                 // Engine RPM
    uint8_t m_drs;                        // 0 = off, 1 = on
    uint8_t m_revLightsPercent;           // Rev lights indicator (percentage)
    uint16_t m_revLightsBitValue;         // Rev lights (bit 0 = leftmost LED, bit 14 = rightmost LED)
    uint16_t m_brakesTemperature[4];      // Brakes temperature (celsius)
    uint8_t m_tyresSurfaceTemperature[4]; // Tyres surface temperature (celsius)
    uint8_t m_tyresInnerTemperature[4];   // Tyres inner temperature (celsius)
    uint16_t m_engineTemperature;         // Engine temperature (celsius)
    float m_tyresPressure[4];             // Tyres pressure (PSI)
    uint8_t m_surfaceType[4];             // Driving surface, see appendices
};

struct PacketCarTelemetryData
{
    PacketHeader m_header; // Header
    CarTelemetryData m_carTelemetryData[22];
    uint8_t m_mfdPanelIndex;                // Index of MFD panel open - 255 = MFD closed Single player, race – 0 = Car setup, 1 = Pits, 2 = Damage, 3 =  Engine, 4 = Temperatures
    uint8_t m_mfdPanelIndexSecondaryPlayer; // See above
    int8_t m_suggestedGear;                 // Suggested gear for the player (1-8) 0 if no gear suggested
};

// Car Status Packet-This packet details car statuses for all the cars in the race.
struct CarStatusData
{
    uint8_t m_tractionControl;        // Traction control - 0 = off, 1 = medium, 2 = full
    uint8_t m_antiLockBrakes;         // 0 (off) - 1 (on)
    uint8_t m_fuelMix;                // Fuel mix - 0 = lean, 1 = standard, 2 = rich, 3 = max
    uint8_t m_frontBrakeBias;         // Front brake bias (percentage)
    uint8_t m_pitLimiterStatus;       // Pit limiter status - 0 = off, 1 = on
    float m_fuelInTank;               // Current fuel mass
    float m_fuelCapacity;             // Fuel capacity
    float m_fuelRemainingLaps;        // Fuel remaining in terms of laps (value on MFD)
    uint16_t m_maxRPM;                // Cars max RPM, point of rev limiter
    uint16_t m_idleRPM;               // Cars idle RPM
    uint8_t m_maxGears;               // Maximum number of gears
    uint8_t m_drsAllowed;             // 0 = not allowed, 1 = allowed
    uint16_t m_drsActivationDistance; // 0 = DRS not available, non-zero - DRS will be available in [X] metres
    uint8_t m_actualTyreCompound;     // F1 Modern - 16 = C5, 17 = C4, 18 = C3, 19 = C2, 20 = C1, 21 = C0, 7 = inter, 8 = wet
                                      // F1 Classic - 9 = dry, 10 = wet
                                      // F2 – 11 = super soft, 12 = soft, 13 = medium, 14 = hard, 15 = wet
    uint8_t m_visualTyreCompound;     // F1 visual (can be different from actual compound)
                                      // 16 = soft, 17 = medium, 18 = hard, 7 = inter, 8 = wet
                                      // F1 Classic – same as above
                                      // F2 ‘19, 15 = wet, 19 – super soft, 20 = soft
                                      // 21 = medium , 22 = hard
    uint8_t m_tyresAgeLaps;           // Age in laps of the current set of tyres
    int8_t m_vehicleFiaFlags;         // -1 = invalid/unknown, 0 = none, 1 = green, 2 = blue, 3 = yellow
    float m_enginePowerICE;           // Engine power output of ICE (W)
    float m_enginePowerMGUK;          // Engine power output of MGU-K (W)
    float m_ersStoreEnergy;           // ERS energy store in Joules
    uint8_t m_ersDeployMode;          // ERS deployment mode, 0 = none, 1 = medium, 2 = hotlap, 3 = overtake
    float m_ersHarvestedThisLapMGUK;  // ERS energy harvested this lap by MGU-K
    float m_ersHarvestedThisLapMGUH;  // ERS energy harvested this lap by MGU-H
    float m_ersDeployedThisLap;       // ERS energy deployed this lap
    uint8_t m_networkPaused;          // Whether the car is paused in a network game
};

struct PacketCarStatusData
{
    PacketHeader m_header; // Header
    CarStatusData m_carStatusData[22];
};

// Final Classification Packet - This packet details the final classification at the end of the race, and the data will match with the post race results screen. This is especially useful for multiplayer games where it is not always possible to send lap times on the final frame because of network delay.
struct FinalClassificationData
{
    uint8_t m_position;             // Finishing position
    uint8_t m_numLaps;              // Number of laps completed
    uint8_t m_gridPosition;         // Grid position of the car
    uint8_t m_points;               // Number of points scored
    uint8_t m_numPitStops;          // Number of pit stops made
    uint8_t m_resultStatus;         // Result status - 0 = invalid, 1 = inactive, 2 = active,3 = finished, 4 = didnotfinish, 5 = disqualified ,6 = not classified, 7 = retired
    uint32_t m_bestLapTimeInMS;     // Best lap time of the session in milliseconds
    double m_totalRaceTime;         // Total race time in seconds without penalties
    uint8_t m_penaltiesTime;        // Total penalties accumulated in seconds
    uint8_t m_numPenalties;         // Number of penalties applied to this driver
    uint8_t m_numTyreStints;        // Number of tyres stints up to maximum
    uint8_t m_tyreStintsActual[8];  // Actual tyres used by this driver
    uint8_t m_tyreStintsVisual[8];  // Visual tyres used by this driver
    uint8_t m_tyreStintsEndLaps[8]; // The lap number stints end on
};

struct PacketFinalClassificationData
{
    PacketHeader m_header; // Header
    uint8_t m_numCars;     // Number of cars in the final classification
    FinalClassificationData m_classificationData[22];
};

// Lobby Info Packet- This packet details the players currently in a multiplayer lobby. It details each player’s selected car, any AI involved in the game and also the ready status of each of the participants.

struct LobbyInfoData
{
    uint8_t m_aiControlled; // Whether the vehicle is AI (1) or Human (0) controlled
    uint8_t m_teamId;       // Team id - see appendix (255 if no team currently selected)
    uint8_t m_nationality;  // Nationality of the driver
    uint8_t m_platform;     // 1 = Steam, 3 = PlayStation, 4 = Xbox, 6 = Origin, 255 = unknown
    char m_name[48];        // Name of participant in UTF-8 format – null terminated Will be truncated with ... (U+2026) if too long
    uint8_t m_carNumber;    // Car number of the player
    uint8_t m_readyStatus;  // 0 = not ready, 1 = ready, 2 = spectating
};

struct PacketLobbyInfoData
{
    PacketHeader m_header; // Header
    uint8_t m_numPlayers;  // Number of players in the lobby data
    LobbyInfoData m_lobbyPlayers[22];
};

// Car Damage Packet - This packet details car damage parameters for all the cars in the race.
struct CarDamageData
{
    float m_tyresWear[4];           // Tyre wear (percentage)
    uint8_t m_tyresDamage[4];       // Tyre damage (percentage)
    uint8_t m_brakesDamage[4];      // Brakes damage (percentage)
    uint8_t m_frontLeftWingDamage;  // Front left wing damage (percentage)
    uint8_t m_frontRightWingDamage; // Front right wing damage (percentage)
    uint8_t m_rearWingDamage;       // Rear wing damage (percentage)
    uint8_t m_floorDamage;          // Floor damage (percentage)
    uint8_t m_diffuserDamage;       // Diffuser damage (percentage)
    uint8_t m_sidepodDamage;        // Sidepod damage (percentage)
    uint8_t m_drsFault;             // Indicator for DRS fault, 0 = OK, 1 = fault
    uint8_t m_ersFault;             // Indicator for ERS fault, 0 = OK, 1 = fault
    uint8_t m_gearBoxDamage;        // Gear box damage (percentage)
    uint8_t m_engineDamage;         // Engine damage (percentage)
    uint8_t m_engineMGUHWear;       // Engine wear MGU-H (percentage)
    uint8_t m_engineESWear;         // Engine wear ES (percentage)
    uint8_t m_engineCEWear;         // Engine wear CE (percentage)
    uint8_t m_engineICEWear;        // Engine wear ICE (percentage)
    uint8_t m_engineMGUKWear;       // Engine wear MGU-K (percentage)
    uint8_t m_engineTCWear;         // Engine wear TC (percentage)
    uint8_t m_engineBlown;          // Engine blown, 0 = OK, 1 = fault
    uint8_t m_engineSeized;         // Engine seized, 0 = OK, 1 = fault
};

struct PacketCarDamageData
{
    PacketHeader m_header; // Header
    CarDamageData m_carDamageData[22];
};

// Session History Packet - This packet contains lap times and tyre usage for the session. 
// This packet works slightly differently to other packets. To reduce CPU and bandwidth, each packet relates to a specific
// vehicle and is sent every 1/20 s, and the vehicle being sent is cycled through. 
// Therefore in a 20 car race you should receive an update for each vehicle at least once per second. Note that at the end of the race, after the final classification packet has been sent, a final bulk update of all the session histories for the vehicles in that session will be sent. -- 20 per second but cycling through cars

struct LapHistoryData
{
    uint32_t m_lapTimeInMS;       // Lap time in milliseconds
    uint16_t m_sector1TimeInMS;   // Sector 1 time in milliseconds
    uint8_t m_sector1TimeMinutes; // Sector 1 whole minute part
    uint16_t m_sector2TimeInMS;   // Sector 2 time in milliseconds
    uint8_t m_sector2TimeMinutes; // Sector 2 whole minute part
    uint16_t m_sector3TimeInMS;   // Sector 3 time in milliseconds
    uint8_t m_sector3TimeMinutes; // Sector 3 whole minute part
    uint8_t m_lapValidBitFlags;   // 0x01 bit set-lap valid,      0x02 bit set-sector 1 valid
                                  // 0x04 bit set-sector 2 valid, 0x08 bit set-sector 3 valid
};

struct TyreStintHistoryData
{
    uint8_t m_endLap;             // Lap the tyre usage ends on (255 of current tyre)
    uint8_t m_tyreActualCompound; // Actual tyres used by this driver
    uint8_t m_tyreVisualCompound; // Visual tyres used by this driver
};

struct PacketSessionHistoryData
{
    PacketHeader m_header;                // Header
    uint8_t m_carIdx;                     // Index of the car this lap data relates to
    uint8_t m_numLaps;                    // Num laps in the data (including current partial lap)
    uint8_t m_numTyreStints;              // Number of tyre stints in the data
    uint8_t m_bestLapTimeLapNum;          // Lap the best lap time was achieved on
    uint8_t m_bestSector1LapNum;          // Lap the best Sector 1 time was achieved on
    uint8_t m_bestSector2LapNum;          // Lap the best Sector 2 time was achieved on
    uint8_t m_bestSector3LapNum;          // Lap the best Sector 3 time was achieved on
    LapHistoryData m_lapHistoryData[100]; // 100 laps of data max
    TyreStintHistoryData m_tyreStintsHistoryData[8];
};

// Tyre Sets Packet- This packets gives a more in-depth details about tyre sets assigned to a vehicle during the session. 20 per second but cycling through cars

struct TyreSetData
{
    uint8_t m_actualTyreCompound; // Actual tyre compound used
    uint8_t m_visualTyreCompound; // Visual tyre compound used
    uint8_t m_wear;               // Tyre wear (percentage)
    uint8_t m_available;          // Whether this set is currently available
    uint8_t m_recommendedSession; // Recommended session for tyre set
    uint8_t m_lifeSpan;           // Laps left in this tyre set
    uint8_t m_usableLife;         // Max number of laps recommended for this compound
    uint16_t m_lapDeltaTime;      // Lap delta time in milliseconds compared to fitted set
    uint8_t m_fitted;             // Whether the set is fitted or not
};

struct PacketTyreSetsData
{
    PacketHeader m_header;         // Header
    uint8_t m_carIdx;              // Index of the car this data relates to
    TyreSetData m_tyreSetData[20]; // 20 sets of tyre data
    uint8_t m_fittedIdx;           // Index into array of fitted tyre
};

// Motion Ex Packet - The motion packet gives extended data for the car being driven with the goal of being able to drive a motion platform setup.
struct PacketMotionExData
{
    PacketHeader m_header;             // Header
    float m_suspensionPosition[4];     // Note: All wheel arrays have the following order:
    float m_suspensionVelocity[4];     // RL, RR, FL, FR
    float m_suspensionAcceleration[4]; // RL, RR, FL, FR
    float m_wheelSpeed[4];             // Speed of each wheel
    float m_wheelSlipRatio[4];         // Slip ratio for each wheel
    float m_wheelSlipAngle[4];         // Slip angles for each wheel
    float m_wheelLatForce[4];          // Lateral forces for each wheel
    float m_wheelLongForce[4];         // Longitudinal forces for each wheel
    float m_heightOfCOGAboveGround;    // Height of centre of gravity above ground
    float m_localVelocityX;            // Velocity in local space – metres/s
    float m_localVelocityY;            // Velocity in local space
    float m_localVelocityZ;            // Velocity in local space
    float m_angularVelocityX;          // Angular velocity x-component – radians/s
    float m_angularVelocityY;          // Angular velocity y-component
    float m_angularVelocityZ;          // Angular velocity z-component
    float m_angularAccelerationX;      // Angular acceleration x-component – radians/s/s
    float m_angularAccelerationY;      // Angular acceleration y-component
    float m_angularAccelerationZ;      // Angular acceleration z-component
    float m_frontWheelsAngle;          // Current front wheels angle in radians
    float m_wheelVertForce[4];         // Vertical forces for each wheel
};

// Lap Data Packet - The lap data packet gives details of all the cars in the session.
struct LapData
{
    uint32_t m_lastLapTimeInMS;            // Last lap time in milliseconds
    uint32_t m_currentLapTimeInMS;         // Current time around the lap in milliseconds
    uint16_t m_sector1TimeInMS;            // Sector 1 time in milliseconds
    uint8_t m_sector1TimeMinutes;          // Sector 1 whole minute part
    uint16_t m_sector2TimeInMS;            // Sector 2 time in milliseconds
    uint8_t m_sector2TimeMinutes;          // Sector 2 whole minute part
    uint16_t m_deltaToCarInFrontInMS;      // Time delta to car in front in milliseconds
    uint16_t m_deltaToRaceLeaderInMS;      // Time delta to race leader in milliseconds
    float m_lapDistance;                   // Distance vehicle is around current lap in metres – could be negative if line hasn’t been crossed yet
    float m_totalDistance;                 // Total distance travelled in session in metres – could be negative if line hasn’t been crossed yet
    float m_safetyCarDelta;                // Delta in seconds for safety car
    uint8_t m_carPosition;                 // Car race position
    uint8_t m_currentLapNum;               // Current lap number
    uint8_t m_pitStatus;                   // 0 = none, 1 = pitting, 2 = in pit area
    uint8_t m_numPitStops;                 // Number of pit stops taken in this race
    uint8_t m_sector;                      // 0 = sector1, 1 = sector2, 2 = sector3
    uint8_t m_currentLapInvalid;           // Current lap invalid - 0 = valid, 1 = invalid
    uint8_t m_penalties;                   // Accumulated time penalties in seconds to be added
    uint8_t m_totalWarnings;               // Accumulated number of warnings issued
    uint8_t m_cornerCuttingWarnings;       // Accumulated number of corner cutting warnings issued
    uint8_t m_numUnservedDriveThroughPens; // Num drive through pens left to serve
    uint8_t m_numUnservedStopGoPens;       // Num stop go pens left to serve
    uint8_t m_gridPosition;                // Grid position the vehicle started the race in
    uint8_t m_driverStatus;                // Status of driver - 0 = in garage, 1 = flying lap, 2 = in lap, 3 = out lap, 4 = on track
    uint8_t m_resultStatus;                // Result status - 0 = invalid, 1 = inactive, 2 = active, 3 = finished, 4 = didnotfinish, 5 = disqualified, 6 = not classified, 7 = retired
    uint8_t m_pitLaneTimerActive;          // Pit lane timing, 0 = inactive, 1 = active
    uint16_t m_pitLaneTimeInLaneInMS;      // If active, the current time spent in the pit lane in ms
    uint16_t m_pitStopTimerInMS;           // Time of the actual pit stop in ms
    uint8_t m_pitStopShouldServePen;       // Whether the car should serve a penalty at this stop
};

struct PacketLapData
{
    PacketHeader m_header;          // Header
    LapData m_lapData[22];          // Lap data for all cars on track
    uint8_t m_timeTrialPBCarIdx;    // Index of Personal Best car in time trial (255 if invalid)
    uint8_t m_timeTrialRivalCarIdx; // Index of Rival car in time trial (255 if invalid)
};

// // Event Packet This packet gives details of events that happen during the course of a session.
// // The event details packet is different for each type of event - Make sure only the correct type is interpreted.

// // Event String Codes
// // Event	Code	Description
// // Session Started	“SSTA”	Sent when the session starts
// // Session Ended	“SEND”	Sent when the session ends
// // Fastest Lap	“FTLP”	When a driver achieves the fastest lap
// // Retirement	“RTMT”	When a driver retires
// // DRS enabled	“DRSE”	Race control have enabled DRS
// // DRS disabled	“DRSD”	Race control have disabled DRS
// // Team mate in pits	“TMPT”	Your team mate has entered the pits
// // Chequered flag	“CHQF”	The chequered flag has been waved
// // Race Winner	“RCWN”	The race winner is announced
// // Penalty Issued	“PENA”	A penalty has been issued – details in event
// // Speed Trap Triggered	“SPTP”	Speed trap has been triggered by fastest speed
// // Start lights	“STLG”	Start lights – number shown
// // Lights out	“LGOT”	Lights out
// // Drive through served	“DTSV”	Drive through penalty served
// // Stop go served	“SGSV”	Stop go penalty served
// // Flashback	“FLBK”	Flashback activated
// // Button status	“BUTN”	Button status changed
// // Red Flag	“RDFL”	Red flag shown
// // Overtake	“OVTK”	Overtake occurred

// union EventDataDetails
// {
//     struct
//     {
//         uint8_t vehicleIdx; // Vehicle index of car achieving fastest lap
//         float lapTime;      // Lap time is in seconds
//     } FastestLap;

//     struct
//     {
//         uint8_t vehicleIdx; // Vehicle index of car retiring
//     } Retirement;

//     struct
//     {
//         uint8_t vehicleIdx; // Vehicle index of team mate
//     } TeamMateInPits;

//     struct
//     {
//         uint8_t vehicleIdx; // Vehicle index of the race winner
//     } RaceWinner;

//     struct
//     {
//         uint8_t penaltyType;      // Penalty type – see Appendices
//         uint8_t infringementType; // Infringement type – see Appendices
//         uint8_t vehicleIdx;       // Vehicle index of the car the penalty is applied to
//         uint8_t otherVehicleIdx;  // Vehicle index of the other car involved
//         uint8_t time;             // Time gained, or time spent doing action in seconds
//         uint8_t lapNum;           // Lap the penalty occurred on
//         uint8_t placesGained;     // Number of places gained by this
//     } Penalty;

//     struct 
//     {
//         uint8_t vehicleIdx;                 // Vehicle index of the vehicle triggering speed trap
//         float speed;                        // Top speed achieved in kilometres per hour
//         uint8_t isOverallFastestInSession;  // Overall fastest speed in session = 1, otherwise 0
//         uint8_t isDriverFastestInSession;   // Fastest speed for driver in session = 1, otherwise 0
//         uint8_t fastestVehicleIdxInSession; // Vehicle index of the vehicle that is the fastest in this session
//         float fastestSpeedInSession;        // Speed of the vehicle that is the fastest  in this session
//     } SpeedTrap;

//     struct
//     {
//         uint8_t numLights; // Number of lights showing
//     } StartLIghts;

//     struct
//     {
//         uint8_t vehicleIdx; // Vehicle index of the vehicle serving drive through
//     } DriveThroughPenaltyServed;

//     struct
//     {
//         uint8_t vehicleIdx; // Vehicle index of the vehicle serving stop go
//     } StopGoPenaltyServed;

//     struct
//     {
//         uint32_t flashbackFrameIdentifier; // Frame identifier flashed back to
//         float flashbackSessionTime;      // Session time flashed back to
//     } Flashback;

//     struct
//     {
//         uint32_t buttonStatus; // Bit flags specifying which buttons are being pressed
//                              // currently - see appendices
//     } Buttons;

//     struct
//     {
//         uint8_t overtakingVehicleIdx;     // Vehicle index of the vehicle overtaking
//         uint8_t beingOvertakenVehicleIdx; // Vehicle index of the vehicle being overtaken
//     } Overtake;
// };

// struct PacketEventData
// {
//     PacketHeader m_header; // Header
//     uint8_t m_eventStringCode[4];    // Event string code, see below
//     EventDataDetails m_eventDetails; // Event details - should be interpreted differently
// };

// // Participants Packet - This is a list of participants in the race. If the vehicle is controlled by AI, then the name will be the driver name. If this is a multiplayer game, the names will be the Steam Id on PC, or the LAN name if appropriate.
// // N.B. on Xbox One, the names will always be the driver name, on PS4 the name will be the LAN name if playing a LAN game, otherwise it will be the driver name.
// // The array should be indexed by vehicle index.
// struct ParticipantData
// {
//     uint8_t m_aiControlled;    // Whether the vehicle is AI (1) or Human (0) controlled
    uint8_t m_driverId;        // Driver id - see appendix, 255 if network human
    uint8_t m_networkId;       // Network id – unique identifier for network players
    uint8_t m_teamId;          // Team id - see appendix
    uint8_t m_myTeam;          // My team flag – 1 = My Team, 0 = otherwise
    uint8_t m_raceNumber;      // Race number of the car
    uint8_t m_nationality;     // Nationality of the driver
    char m_name[48];           // Name of participant in UTF-8 format – null terminated Will be truncated with … (U+2026) if too long
    uint8_t m_yourTelemetry;   // The player's UDP setting, 0 = restricted, 1 = public
    uint8_t m_showOnlineNames; // The player's show online names setting, 0 = off, 1 = on
    uint8_t m_platform;        // 1 = Steam, 3 = PlayStation, 4 = Xbox, 6 = Origin, 255 = unknown
};
struct PacketParticipantsData
{
    PacketHeader m_header;   // Header
    uint8_t m_numActiveCars; // Number of active cars in the data – should match number of cars on HUD
    ParticipantData m_participants[22];
};
