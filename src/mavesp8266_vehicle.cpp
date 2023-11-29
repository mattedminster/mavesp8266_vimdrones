/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavesp8266_vehicle.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_component.h"

#include <FastLED.h>
#include <HardwareSerial.h>
#include <ardupilotmega/mavlink.h>



HardwareSerial ESP32Serial1(1);


// Define a structure to store the DEBUG_VECT message data
struct DebugVectMessage {
    uint64_t timestamp;
    float x;
    float y;
    float z;
};

// Parse the DEBUG_VECT message and store the data in the provided struct
bool parseDebugVectMessage(mavlink_message_t message, DebugVectMessage& debugVectMessage) {
    if (message.msgid != MAVLINK_MSG_ID_DEBUG_VECT) {
        return false; // Not a DEBUG_VECT message
    }

    mavlink_debug_vect_t debugVect;
    mavlink_msg_debug_vect_decode(&message, &debugVect);

    // Copy the data from the mavlink_debug_vect_t struct to the DebugVectMessage struct
    //debugVectMessage.timestamp = debugVect.usec;
    debugVectMessage.x = debugVect.x;
    debugVectMessage.y = debugVect.y;
    debugVectMessage.z = debugVect.z;

    return true; // Successfully parsed the DEBUG_VECT message
}

CRGB leds[1];




//---------------------------------------------------------------------------------
MavESP8266Vehicle::MavESP8266Vehicle()
{
    _recv_chan = MAVLINK_COMM_0;
    _send_chan = MAVLINK_COMM_1;
}



//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Vehicle::begin(MavESP8266Bridge* forwardTo)
{
    MavESP8266Bridge::begin(forwardTo);


    //-- Start UART connected to UAS
    //ESP32Serial1.println("Resizing buffer...");
    FastLED.addLeds<WS2812, 32, GRB>(leds, 1);
    ESP32Serial1.setRxBufferSize(4096);
    ESP32Serial1.begin(getWorld()->getParameters()->getUartBaudRate(), SERIAL_8N1, 26, 27);
    ESP32Serial1.available();
    Serial.println("Starting Serial for UAS");
#ifdef ENABLE_DEBUG
#ifdef ARDUINO_ESP8266_ESP12
    //Serial.swap();
    //Serial.println("Serial swapped...");
#endif
#endif
    // raise serial buffer size (default is 256)
    //Serial.println("Resizing buffer...");
    //Serial.setRxBufferSize(4096);
}



//---------------------------------------------------------------------------------
//-- Read MavLink message from UAS
void
MavESP8266Vehicle::readMessage()
{
    //Serial.println("Enter Read Vehicle Msg");
    if (_readMessage()) {
        _forwardTo->sendMessage(&_msg);

        DebugVectMessage debugVectMessage;
    if (parseDebugVectMessage(_msg, debugVectMessage)) {
        // Successfully parsed the DEBUG_VECT message
        // Use the data in debugVectMessage as needed, e.g.:
        uint8_t color_r = debugVectMessage.x * 255;
        uint8_t color_g = debugVectMessage.y * 255;
        uint8_t color_b = debugVectMessage.z * 255;

        leds[0] = CRGB(color_g, color_r, color_b);
        FastLED.show();
    }
    
        
    }
    //-- Update radio status (1Hz)
    if(_heard_from && (millis() - _last_status_time > 1000)) {
        delay(0);
        _last_status_time = millis();
    }
}

void
MavESP8266Vehicle::readMessageRaw() {
    char buf[1024];
    int buf_index = 0;

    while(ESP32Serial1.available() && buf_index < 300)
    {
        int result = ESP32Serial1.read();
        if (result >= 0)
        {
            buf[buf_index] = (char)result;
            buf_index++;
        }
    }

    if (buf_index > 0) {
        _forwardTo->sendMessageRaw((uint8_t*)buf, buf_index);
    }
}

//---------------------------------------------------------------------------------
//-- Send MavLink message to UAS
int
MavESP8266Vehicle::sendMessage(mavlink_message_t* message) {
    // Translate message to buffer
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, message);
    // Send it
    while (ESP32Serial1.availableForWrite() < 32) {
        // don't spin in the send loop, wait for 25% of the FIFO to be free
        delay(1);
    }
    ESP32Serial1.write((uint8_t*)(void*)buf, len);
    _status.packets_sent++;
    return 1;
}

int
MavESP8266Vehicle::sendMessageRaw(uint8_t *buffer, int len) {
    ESP32Serial1.write(buffer, len);
    //Serial.flush();
    return len;
}

//---------------------------------------------------------------------------------
//-- We have some special status to capture when asked for
linkStatus*
MavESP8266Vehicle::getStatus()
{
    _status.queue_status = 0;
    return &_status;
}

//---------------------------------------------------------------------------------
//-- Read MavLink message from UAS
bool
MavESP8266Vehicle::_readMessage()
{
    //Serial.println("Vehicle - Entered Underscore Read Msg ");
#if defined(ARDUINO_ESP32_DEV) || defined(ARDUINO_ESP32S3_DEV) || defined (ARDUINO_ESP32C3_DEV)  
    char msgReceived = false;
#else    
    bool msgReceived = false;
#endif    
    int16_t avail = ESP32Serial1.available();
    if (avail <= 0 && _non_mavlink_len != 0 && _rxstatus.parse_state <= MAVLINK_PARSE_STATE_IDLE) {
        //Serial.println("<<<<< Serial is not active >>>>>");
        // flush out the non-mavlink buffer when there is nothing pending. This
        // allows us to gather non-mavlink msgs into a single write
        _forwardTo->sendMessageRaw(_non_mavlink_buffer, _non_mavlink_len);
        _non_mavlink_len = 0;
    }
    while (avail--)
    {
        //Serial.println(">>>>> Reading Serial... <<<<<");
        int result = ESP32Serial1.read();
        if (result >= 0)
        {
            // Parsing
            uint8_t last_parse_error = _rxstatus.parse_error;
            msgReceived = mavlink_frame_char_buffer(&_rxmsg,
                                                    &_rxstatus,
                                                    result,
                                                    &_msg,
                                                    &_mav_status);
            handle_non_mavlink(result, msgReceived);
            if (last_parse_error != _rxstatus.parse_error) {
                //Serial.println("Parsing Errors!!!!");
                _status.parse_errors++;
            }
            if(msgReceived) {
                Serial.println(_msg.msgid);
                _status.packets_received++;
                //-- Is this the first packet we got?
                //Serial.println("First packet_received");
                if(!_heard_from) {
                    if(_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        _heard_from     = true;
                        _component_id   = _msg.compid;
                        _system_id      = _msg.sysid;
                        _seq_expected   = _msg.seq + 1;
                        _last_heartbeat = millis();
                        //Serial.println("Vehicle - !_heard_from - MAVLINK_MSG_ID_HEARTBEAT");
                    }
                } else {
                    if(_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                        _last_heartbeat = millis();
                    //Serial.println("Vehicle - Last Heartbeat - MAVLINK_MSG_ID_HEARTBEAT");
                    _checkLinkErrors(&_msg);
                }
                if(_msg.msgid == MAVLINK_MSG_ID_NAMED_VALUE_FLOAT)
                {
                    //Serial.println("Vehicle - NAMED_VALUE_FLOAT");
                    mavlink_named_value_float_t named_value_float;
                    mavlink_msg_named_value_float_decode(&_msg, &named_value_float);
                    Serial.println(named_value_float.name);
                    Serial.println(named_value_float.value);
                
                }

                if(_msg.msgid == MAVLINK_MSG_ID_LED_CONTROL){
                    Serial.println("Vehicle - LED_CONTROL");

                }

                if (msgReceived == MAVLINK_FRAMING_BAD_CRC) {
                    // we don't process messages locally with bad CRC,
                    // but we do forward them, so when new messages
                    // are added we can bridge them
                    break;
                }

#ifdef MAVLINK_FRAMING_BAD_SIGNATURE
                if (msgReceived == MAVLINK_FRAMING_BAD_SIGNATURE) {
                    //Serial.println("Vehicle - MAVLINK_FRAMING_BAD_SIGNATURE");
                    break;
                }
#endif
                
                //-- Check for message we might be interested
                if(getWorld()->getComponent()->handleMessage(this, &_msg)){
                    msgReceived = false;
                    continue;
                }

                break;
            }
        }
    }
    if(!msgReceived) {
        if(_heard_from && (millis() - _last_heartbeat) > HEARTBEAT_TIMEOUT) {
            _heard_from = false;
            getWorld()->getLogger()->log("Heartbeat timeout from Vehicle\n");
        }
    }
    return msgReceived;
}

