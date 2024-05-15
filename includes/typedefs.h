#pragma once
#define min2(a, b) ((a) < (b) ? (a) : (b))
#define min3(a, b, c) ((a) < (b)) ? (((a) < (c)) ? (a) : (c)) : (((b) < (c)) ? (b) : (c))
#define max2(a, b) ((a) > (b) ? (a) : (b))
#define max3(a, b, c) ((a) > (b)) ? (((a) > (c)) ? (a) : (c)) : (((b) > (c)) ? (b) : (c))

// 0 - White
// 1 - Black
// 2 - Red
// 3 - Green
// 4 - Blue
// 5 - Yellow

typedef struct {
    tSensors nDeviceIndex;

    long rawRed;
    long rawGreen;
    long rawBlue;

    float normRed;
    float normGreen;
    float normBlue;

    float hue;
    float sat;
    float val;

    short minRed;
    short minGreen;
    short minBlue;

    short maxRed;
    short maxGreen;
    short maxBlue;

    short color;
} tCDValues;

tCDValues CDSensor1, CDSensor2;

void colorDetectInit() {
    CDSensor1.nDeviceIndex = S1;
    CDSensor1.minRed = 0;
    CDSensor1.maxRed = 156;
    CDSensor1.minGreen = 0;
    CDSensor1.maxGreen = 192;
    CDSensor1.minBlue = 0;
    CDSensor1.maxBlue = 136;

    CDSensor2.nDeviceIndex = S2;
    CDSensor2.minRed = 0;
    CDSensor2.maxRed = 346;
    CDSensor2.minGreen = 0;
    CDSensor2.maxGreen = 309;
    CDSensor2.minBlue = 0;
    CDSensor2.maxBlue = 341;
}