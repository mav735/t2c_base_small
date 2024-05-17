const float g_wheelDiameter = 62.3;                                                // mm
const float g_wheelBase = (178) / g_wheelDiameter / PI * 360;                      // mm -> degrees
const float g_distBetweenSensors = (34) / g_wheelDiameter / PI * 360;              // mm -> degrees
const float g_distBetweenSensorsAndWheelBase = 74 / g_wheelDiameter / PI * 360;    // mm -> degrees

void saveRatioPID(float *va, float *vb) {
    if (fabs(*va) > 100){
        short sgnA = sgn(*va);
        *vb = *vb / *va * 100 * sgnA;
        *va = 100 * sgnA;
    }
    else if (fabs(*vb) > 100){
        short sgnB = sgn(*vb);
        *va = *va / *vb * 100 * sgnB;
        *vb = 100 * sgnB;
    }
}

float mapping(float raw, float min, float max, float normMin, float normMax) {
    return (raw - min) / (max - min) * (normMax - normMin) + normMin;
}

float angleToEnc(float vB, float vC, float angle) {
    return angle / 180 * PI * g_wheelBase * (vB == 0 || vC == 0 ? 1 : (1 / fabs(-vB / vC - 1) + 1 / fabs(-vC / vB - 1)) / 2.);
}

