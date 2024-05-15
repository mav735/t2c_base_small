float kpNow = 0.6;
float kdNow = 8;
float kiNow = 0.002;

const float kpConstBase = 0.5;
const float kdConstBase = 5;
const float kiConstBase = 0.001;

const float kpConstOne = 0.5;
const float kdConstOne = 5;
const float kiConstOne = 0.0015;

float calibrationPower = 50;

typedef struct {
    float minLine;
    float maxLine;
    float crossRoadMin;
    float crossRoadMax;
    short sensorsIndError; // 0, 1, 2
    short sensorsIndCross; // 0, 1, 2
    short inverse;
} tLFConfig, *tLFConfigPtr;

typedef struct {
    float error;
    bool cross;
    float firstValue;
    float secondValue;
} tPIDvalues, *tPIDvaluesPtr;

tLFConfig lineCFG;

void calcKF(float power, float *kp, float *kd, float *ki) {
    float temp = sqrt(power / calibrationPower);
    *kp = temp * kpNow;
    *kd = temp * kdNow;
    *ki = temp * kiNow;
}

void setBaseCoef(){
    kpNow = kpConstBase;
    kdNow = kdConstBase;
    kiNow = kiConstBase;
}

void setOneSensorCoef(){
    kpNow = kpConstOne;
    kdNow = kdConstOne;
    kiNow = kiConstOne;
}

void setDefaultLine() {
    setBaseCoef();
    lineCFG.maxLine = 255;
    lineCFG.minLine = 0;
    lineCFG.inverse = 1;
    lineCFG.crossRoadMax = 70;
    lineCFG.crossRoadMin = -100;
    lineCFG.sensorsIndError = 2;
    lineCFG.sensorsIndCross = 2;
}

void countValues(tCDValues *firstCD, tCDValues *secondCD, tPIDvaluesPtr PIDValues) {
    getCDValues(firstCD);
    getCDValues(secondCD);

    float rgbFirst[3] = {firstCD->normRed, firstCD->normGreen, firstCD->normBlue};
    float rgbSecond[3] = {secondCD->normRed, secondCD->normGreen, secondCD->normBlue};

    short amountValues = 3;
    float firstValue = 0;
    float secondValue = 0;

    for (short i = 0; i < 3; i++) {
        firstValue += rgbFirst[i];
        secondValue += rgbSecond[i];
    }

    firstValue = mapping(firstValue / amountValues, lineCFG.minLine, lineCFG.maxLine, 0, 100);
    secondValue = mapping(secondValue / amountValues, lineCFG.minLine, lineCFG.maxLine, 0, 100);

    float errors[3] = {(firstValue - 50) * lineCFG.inverse,
                       (secondValue - 50) * lineCFG.inverse,
                       (firstValue - secondValue) * lineCFG.inverse};

    firstValue = 0;
    secondValue = 0;
    amountValues = 3;
    for (short i = 0; i < 3; i++) {
        firstValue += rgbFirst[i];
        secondValue += rgbSecond[i];
    }

    firstValue /= amountValues;
    secondValue /= amountValues;

    bool firstCrossCheck = ((lineCFG.crossRoadMin <= firstValue) && (firstValue <= lineCFG.crossRoadMax));
    bool secondCrossCheck = ((lineCFG.crossRoadMin <= secondValue) && (secondValue <= lineCFG.crossRoadMax));

    bool crosses[3] = {firstCrossCheck, secondCrossCheck, (firstCrossCheck && secondCrossCheck)};

    PIDValues->cross = crosses[lineCFG.sensorsIndCross];
    PIDValues->error = errors[lineCFG.sensorsIndError];

}

void lineFollowCross(float power, short crossCount) {
    setMotorBrakeMode(motorA, motorCoast);
    setMotorBrakeMode(motorB, motorCoast);

    tPIDvalues PIDValues;
    PIDValues.cross = false;
    float e, ee = 0, U, P, I, D;
    float kp, kd, ki;
    float curPower = power, curPowerA, curPowerB;
    short curCnt = 0;
    bool flag = false;

    int errSz = 20;
    int err[20];
    for (int i = 0; i < errSz; ++i) {
        err[i] = 0;
    }
    int curErrIdx = 0;
    int nwErrIdx;

    I = 0;
    while (curCnt < crossCount) {
        calcKF(curPower, &kp, &kd, &ki);
        countValues(&CDSensor1, &CDSensor2, &PIDValues);
        if (PIDValues.cross) {
            if (!flag) {
                ++curCnt;
            }
            flag = true;
        } else {
            flag = false;
        }
        e = PIDValues.error;
        P = e * kp;
        nwErrIdx = (curErrIdx + errSz - 1) % errSz;
        err[nwErrIdx] = e;
       	I -= err[curErrIdx];
        I += err[nwErrIdx];
        curErrIdx = (curErrIdx + 1) % errSz;
        D = (e - ee) * kd;
        U = P + I * ki + D;
        curPowerA = -curPower - U;
        curPowerB = curPower - U;
        saveRatioPID(&curPowerA, &curPowerB);
        motor[motorA] = curPowerA;
        motor[motorB] = curPowerB;
        ee = e;
        sleep(1);
    }
}

void lineFollowEncoder(float power, int encoder) {
    setMotorBrakeMode(motorA, motorCoast);
    setMotorBrakeMode(motorB, motorCoast);

    tPIDvalues PIDValues;
    PIDValues.cross = false;
    float e, ee = 0, U, P, I, D;
    float kp, kd, ki;
    float curPower = power, curPowerA, curPowerB;
    float oldAveEnc = (nMotorEncoder[motorB] - nMotorEncoder[motorA]) / 2;
    short curEnc = 0;
    int errSz = 20;
    int err[20];
    for (int i = 0; i < errSz; ++i) {
        err[i] = 0;
    }
    int curErrIdx = 0;
    int nwErrIdx;
    I = 0;

    while ((nMotorEncoder[motorB] - nMotorEncoder[motorA]) / 2 - oldAveEnc < encoder) {
        calcKF(curPower, &kp, &kd, &ki);
        countValues(&CDSensor1, &CDSensor2, &PIDValues);
        e = PIDValues.error;
        P = e * kp;

        nwErrIdx = (curErrIdx + errSz - 1) % errSz;
        err[nwErrIdx] = e;
        I -= err[curErrIdx];
        I += err[nwErrIdx];
        curErrIdx = (curErrIdx + 1) % errSz;

        D = (e - ee) * kd;
        U = P + I * ki + D;
        curPowerA = -curPower - U;
        curPowerB = curPower - U;
        saveRatioPID(&curPowerA, &curPowerB);
        motor[motorA] = curPowerA;
        motor[motorB] = curPowerB;
        ee = e;
        sleep(1);
    }
}
