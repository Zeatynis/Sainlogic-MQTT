#include "data_decode.h"
#include "ppm_tracker.h"

//https://github.com/RFD-FHEM/RFFHEM/blob/master/FHEM/14_SD_WS.pm
// Sainlogic weather station FT-0835
// ---------------------------------------------------------------
//          Byte: 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 
//        Nibble: 01 23 45 67 89 01 23 45 67 89 01 23 45 67 89 01 
// aa aa aa 2d d4 FF D4 C0 E4 00 00 5F 00 00 83 FD 51 FF FB FB 6B
//                PP PP PI IF SS GG DD RR RR FT TT HH BB BB UU CC
// P: 20 bit Preamble always 0xFFD4C
// I:  8 bit Ident
// F:  4 bit Flags: battery, MSB wind direction, MSB wind gust, MSB wind speed
// S:  8 bit LSB wind speed, in 1/10 m/s, resolution 0.1
// G:  8 bit LSB wind gust, in 1/10 m/s, resolution 0.1
// D:  8 bit LSB wind direction, in degree
// R: 16 bit rain counter, in 1/10 l/m², resolution 0.1
// F:  4 bit Flags: Sensors with brightness MSB brightness, 3 bit unknown, sensors without brightness always 0x8
// I: 12 bit Temperature, unsigned fahrenheit, offset by 400 and scaled by 10
// D:  8 bit Humidity, in percent
// R: 16 bit Brightness, sensors without brightness always 0xFFFB
// U:  8 bit UV, sensors without brightness always 0xFB
// C:  8 bit CRC8 over all 16 bytes must be 0

// Taken from https://reveng.sourceforge.io/crc-catalogue/1-15.htm#crc.cat.crc-8-nrsc-5
uint8_t CRC8_TAB[] = {
    0, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9,
    0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E, 0x43, 0x72,
    0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98,
    0xA9, 0x3E, 0xF, 0x5C, 0x6D, 0x86, 0xB7, 0xE4, 0xD5,
    0x42, 0x73, 0x20, 0x11, 0x3F, 0xE, 0x5D, 0x6C, 0xFB,
    0xCA, 0x99, 0xA8, 0xC5, 0xF4, 0xA7, 0x96, 1, 0x30,
    0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA,
    0xEB, 0x3D, 0xC, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13, 0x7E,
    0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6,
    0xA5, 0x94, 3, 0x32, 0x61, 0x50, 0xBB, 0x8A, 0xD9,
    0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 2, 0x33, 0x60, 0x51,
    0xC6, 0xF7, 0xA4, 0x95, 0xF8, 0xC9, 0x9A, 0xAB, 0x3C,
    0xD, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4,
    0xE7, 0xD6, 0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC,
    0xED, 0xC3, 0xF2, 0xA1, 0x90, 7, 0x36, 0x65, 0x54,
    0x39, 8, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80,
    0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17, 0xFC, 0xCD,
    0x9E, 0xAF, 0x38, 9, 0x5A, 0x6B, 0x45, 0x74, 0x27,
    0x16, 0x81, 0xB0, 0xE3, 0xD2, 0xBF, 0x8E, 0xDD, 0xEC,
    0x7B, 0x4A, 0x19, 0x28, 6, 0x37, 0x64, 0x55, 0xC2,
    0xF3, 0xA0, 0x91, 0x47, 0x76, 0x25, 0x14, 0x83, 0xB2,
    0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0xB, 0x58,
    0x69, 4, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A, 0xC1,
    0xF0, 0xA3, 0x92, 5, 0x34, 0x67, 0x56, 0x78, 0x49,
    0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF, 0x82, 0xB3, 0xE0,
    0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0xA, 0x59, 0x68,
    0xFF, 0xCE, 0x9D, 0xAC
};

// CRC-8/NRSC-5 calculation
uint8_t crc8(const uint8_t * pkt, size_t len) {
    uint8_t crc = 0xFF;
    for(size_t i = 0; i < len; i++) {
        crc = CRC8_TAB[crc ^ pkt[i]];
    }
    return crc;
}

// Check the 15 byte message bytes match the 1 byte CRC
bool check_crc(const uint8_t *msg) {
    return crc8(msg, MSG_BYTES - 1) == msg[MSG_BYTES - 1];
}

// Get the wind direction in degrees
float get_direction(const uint8_t *msg) {
    float dir = msg[6];
    if(msg[3] & 0b100) {
        dir += 256;
    }
    return dir;
}

// Get the temperature in degrees C
float get_temperature(const uint8_t *msg) {
    return float((((msg[9] & 0b111) * 256 + msg[10]) /  10.- 40.) - 32.) / 1.8;
}

// Get average wind speed in m/s
float get_avr_wind_speed(const uint8_t *msg) {
    return float(msg[4]) / 10.;
}

// Get gust wind speed in m/s
float get_gust_wind_speed(const uint8_t *msg) {
    return float(msg[5]) / 10.;
}

// Get rain measurement in mm
float get_rain(const uint8_t *msg) {
    return float(msg[7] * 256 + msg[8] - 25590) / 10.;
}

// Get humidity in %
float get_humidity(const uint8_t *msg) {
    return msg[11];
}
