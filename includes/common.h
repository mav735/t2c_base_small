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