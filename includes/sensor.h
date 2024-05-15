void RGBtoHSV(float red, float green, float blue, float *hue, float *sat,
              float *value) {
    *hue = 0;
    *sat = 0;
    *value = 0;

    float _min = min3(red, green, blue);
    float _max = max3(red, green, blue);
    if (_min == _max) {
        *hue = 0;
    } else if (_max == red && green >= blue) {
        *hue = 60 * (green - blue) / (_max - _min);
    } else if (_max == red && green < blue) {
        *hue = 60 * (green - blue) / (_max - _min) + 360;
    } else if (_max == green) {
        *hue = 60 * (blue - red) / (_max - _min) + 120;
    } else if (_max == blue) {
        *hue = 60 * (red - green) / (_max - _min) + 240;
    }
    if (_max == 0) {
        *sat = 0;
    } else {
        *sat = 1 - (_min / _max);
    }
    *value = _max;
}

void getCDValues(tCDValues *CDSensor) {
    getColorRawRGB(CDSensor->nDeviceIndex, CDSensor->rawRed, CDSensor->rawGreen, CDSensor->rawBlue);

    #ifdef CALIBRATION
        CDSensor->normRed = 1;
        CDSensor->normGreen = 1;
        CDSensor->normBlue = 1;

        if ((CDSensor->maxRed - CDSensor->minRed) != 0){
            CDSensor->normRed = (float)(CDSensor->rawRed - CDSensor->minRed) /
                                (CDSensor->maxRed - CDSensor->minRed) * 255;
        }

        if ((CDSensor->maxGreen - CDSensor->minGreen) != 0){
            CDSensor->normGreen = (float)(CDSensor->rawGreen - CDSensor->minGreen) /
                                (CDSensor->maxGreen - CDSensor->minGreen) * 255;
        }

        if ((CDSensor->maxBlue - CDSensor->minBlue) != 0){
        CDSensor->normBlue = (float)(CDSensor->rawBlue - CDSensor->minBlue) /
                            (CDSensor->maxBlue - CDSensor->minBlue) * 255;
        }

    #endif

    #ifndef CALIBRATION
        CDSensor->normRed = (float)(CDSensor->rawRed - CDSensor->minRed) /
                            (CDSensor->maxRed - CDSensor->minRed) * 255;
        CDSensor->normGreen = (float)(CDSensor->rawGreen - CDSensor->minGreen) /
                            (CDSensor->maxGreen - CDSensor->minGreen) * 255;
        CDSensor->normBlue = (float)(CDSensor->rawBlue - CDSensor->minBlue) /
                            (CDSensor->maxBlue - CDSensor->minBlue) * 255;
    #endif

    RGBtoHSV(CDSensor->normRed, CDSensor->normGreen, CDSensor->normBlue,
             &(CDSensor->hue), &(CDSensor->sat), &(CDSensor->val));

    // 0 - White
    // 1 - Black
    // 2 - Red
    // 3 - Green
    // 4 - Blue
    // 5 - Yellow

    // if (CDSensor->val <= 40) {
    //     CDSensor->color = 1;
    // } else if (CDSensor->sat <= 0.15 && CDSensor->val >= 150) {
    //     CDSensor->color = 0;
    // }
    // else {
    //     if (CDSensor->hue < 15 || CDSensor->hue >= 300) {
    //         CDSensor->color = 2;
    //     } else if (CDSensor->hue >= 15 && CDSensor->hue < 90) {
    //         CDSensor->color = 5;
    //     } else if (CDSensor->hue >= 90 && CDSensor->hue < 170) {
    //         CDSensor->color = 3;
    //     } else {
    //         CDSensor->color = 4;
    //     }
    // }
}

void displayValues(tCDValues *CDSensor) {
    setMotorBrakeMode(motorA, motorCoast);
    setMotorBrakeMode(motorB, motorCoast);
    setMotorBrakeMode(motorC, motorCoast);
    setMotorBrakeMode(motorD, motorCoast);
    setSoundVolume(5);
    while (1){
        getCDValues(CDSensor);
        displayCenteredTextLine(1, "RGB_raw: %d %d %d", CDSensor->rawRed,
                                CDSensor->rawGreen, CDSensor->rawBlue);
        displayCenteredTextLine(3, "RGB_norm: %d %d %d", CDSensor->normRed,
                                CDSensor->normGreen, CDSensor->normBlue);
        displayCenteredTextLine(5, "HSV: %f %f %f", CDSensor->hue,
                                CDSensor->sat, CDSensor->val);
        displayBigStringAt(125, 20, "%d", CDSensor->color);
        clearSounds();
        if (CDSensor->color == 1) {
            playSoundFile("Right");
        } else if (CDSensor->color == 2) {
            playSoundFile("Yellow");
        } else if (CDSensor->color == 3) {
            playSoundFile("Green");
        } else if (CDSensor->color == 4) {
            playSoundFile("Blue");
        } else if (CDSensor->color == 5) {
            playSoundFile("Black");
        } else if (CDSensor->color == 6) {
            playSoundFile("White");
        } else {
            playSoundFile("Uh-oh");
        }
        sleep(400);
        eraseDisplay();
    }
}
