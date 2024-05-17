#pragma once

const float g_ArcKP = 3;
const float g_ArcKD = 5.5;
const float g_ArcKI = 0.015;

bool checkEncForArc(float startVA, float startVB, float encA, float encB, float enc) {
    return fabs(encA) + fabs(encB) < enc;
}

bool flagStillMove(float startVA, float startVB, float encA, float encB, float enc){
    return checkEncForArc(startVA, startVB, encA, encB, enc);
}

void arcEnc(float startVA, float startVB, float enc) {
    float ratio = 1;
    if (startVB != 0 && startVA != 0) ratio = fabs(startVA / startVB);
    float sign = -1 * sgn(startVA * startVB);
    float err, u, eold = 0, isum = 0;

    float tempVA, tempVB;
    enc *= 2;

    float encX, excess, smoothStartEncX;
    float maxVX, startVX, signVX;

    int ertSz = 20;
    int ert[20];
    for (int i = 0; i < ertSz; ++i) {
        ert[i] = 0;
    }
    int curErtIdx = 0, nwErtIdx;
    bool isSmoothA;

    startVX = startVB;
    isSmoothA = false;
    if (ratio > 1 || startVB == 0) {
        startVX = startVA;
        isSmoothA = true;
    }
    signVX = sgn(startVX);
    if (startVA == 0 || startVB == 0) enc /= 2;
    float curEncA, curEncB;
    float startEncA = nMotorEncoder[motorA], startEncB = nMotorEncoder[motorB];
    while (flagStillMove(startVA, startVB, (curEncA = nMotorEncoder[motorA] - startEncA), (curEncB = nMotorEncoder[motorB] - startEncB), enc)) {
        err = curEncA * sign + curEncB * ratio;
        if (startVA == 0) err = curEncA;
        else if (startVB == 0) err = curEncB;

        nwErtIdx = (curErtIdx + ertSz - 1) % ertSz;
        ert[nwErtIdx] = err;
        isum -= ert[curErtIdx];
        isum += ert[nwErtIdx];
        curErtIdx = (curErtIdx + 1) % ertSz;

        if (startVA == 0) {
            tempVA = -u;
            tempVB = startVB;
        } else if (startVB == 0) {
            tempVA = startVA;
            tempVB = -u;
        } else {
            u = (err - eold) * g_ArcKD + err * g_ArcKP + isum * g_ArcKI;
            tempVA = startVA - u * sign;
            tempVB = startVB - u;
        }

        saveRatioPID(&tempVA, &tempVB);
        motor[motorA] = tempVA;
        motor[motorB] = tempVB;
        eold = err;
        sleep(1);
    }
}


void arcAngle(float startVA, float startVB, float angle) {
    arcEnc(startVA, startVB, angleToEnc(startVA, startVB, angle));
}

void TurnLeft(float speed, float angle){
    arcEnc(speed, speed, angleToEnc(speed, speed, angle));
}

void TurnRight(float speed, float angle){
    arcEnc(-speed, -speed, angleToEnc(-speed, -speed, angle));
}

void Forward(float speed, float enc){
    arcEnc(-speed, speed, enc);
}

void Backward(float speed, float enc){
    arcEnc(speed, -speed, enc);
}

void arcToBase(int startV, int topV, int endV) {
    arcEnc(-startV, startV, g_distBetweenSensorsAndWheelBase);
}

void stopMove(int time){
    setMotorBrakeMode(motorA, motorBrake);
    setMotorBrakeMode(motorB, motorBrake);
    motor[motorA] = 0;
    motor[motorB] = 0;
    sleep(time);
    setMotorBrakeMode(motorA, motorCoast);
    setMotorBrakeMode(motorB, motorCoast);
}