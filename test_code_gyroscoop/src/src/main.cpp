#include <Arduino.h>

#include "motor.hpp"
#include "motor_control.hpp"
#include "mpu6050.hpp"
#include "dummyControl.hpp"
#include "steer_control.hpp"
#include "travel_control.hpp"

#include <FreeRTOS.h>

using namespace asn;

xTaskHandle motor_control_task_handle;
xTaskHandle steer_control_task_handle;
xTaskHandle travel_control_task_handle;
xTaskHandle dummy_control_task_handle;

static uint8_t motor_pins[7] = { 22, 21, 20, 19, 18, 12, 13 };
static uint8_t button_pins[4] = { 16, 17, 26, 27 };
Kalman kalmanFilter;
MPU6050 gyro( Wire );
Mpu6050 mpu( gyro, kalmanFilter );
MotorControl motor_control( motor_pins );
SteerControl steer_control( mpu, motor_control );
TravelControl travel_control( motor_control, steer_control);
DummyControl dummy_control( travel_control );



void motorControlTask( void* pvParameters ){
    motor_control.main();
}

void steerControlTask( void* pvParameters ){
    steer_control.main();
}

void travelControlTask( void* pvParameters ){
    travel_control.main();
}

void dummyControlTask( void* pvParameters ){
    dummy_control.main();
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.setUpGyro();

    vTaskDelay(pdMS_TO_TICKS(2000));

    Serial.println("Creating tasks...");
    // auto return_motor = xTaskCreate(motorControlTask, "MotorControl task", 2048, NULL, 1, &motor_control_task_handle);
    auto return_steer = xTaskCreate(steerControlTask, "SteerControl task", 2048, NULL, 1, &steer_control_task_handle);
    // auto return_travel = xTaskCreate(travelControlTask, "TravelControl task", 2048, NULL, 1, &travel_control_task_handle);
    auto return_dummy = xTaskCreate(dummyControlTask, "DummyControl task", 2048, NULL, 1, &dummy_control_task_handle);
    if (return_dummy != pdPASS) {
        Serial.println("Motor task creation failed");
        vTaskDelete(dummy_control_task_handle);
    } else {
        Serial.println("Tasks created successfully.");
    }
}

void loop() {
    // Serial.print( "graden: " + String( round( gyro.getCurrent_z() ) ) );
    // if ( round( gyro.getCurrent_z() ) < gyro.getSetpoint() ) {
    //     Serial.println( " left " );
    //     motorControl.move( motorControl.direction_t::LEFT );
    //     delay( 50 );
    // } else if ( round( gyro.getCurrent_z() ) > gyro.getSetpoint() ) {
    //     Serial.println( " right " );
    //     motorControl.move( motorControl.direction_t::RIGHT );
    //     delay( 50 );
    // }
    // if ( digitalRead( button_pins[0] ) == HIGH ) {
    //     motorControl.move( motorControl.direction_t::FORWARD );
    //     delay( 50 );
    //     motorControl.move( motorControl.direction_t::STOP );
    // }
    // if ( digitalRead( button_pins[1] ) == HIGH ) {
    //     motorControl.move( motorControl.direction_t::BACKWARD );
    //     delay( 50 );
    //     motorControl.move( motorControl.direction_t::STOP );
    // }
    // if ( digitalRead( button_pins[2] ) == HIGH ) {
    //     motorControl.move( motorControl.direction_t::UP );
    //     delay( 50 );
    //     motorControl.move( motorControl.direction_t::STOP );
    // }
    // if ( digitalRead( button_pins[3] ) == HIGH ) {
    //     motorControl.move( motorControl.direction_t::DOWN );
    //     delay( 50 );
    //     motorControl.move( motorControl.direction_t::STOP );
    // }
}
