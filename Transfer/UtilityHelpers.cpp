#include "UtilityHelpers.h"

Color GetColorForMass(double mass) {
    if (mass <= BodyMassOptions::Light) {
        return ColorLibrary::White;
    }
    else if (mass <= BodyMassOptions::Medium) {
        return ColorLibrary::Green;
    }
    else {
        return ColorLibrary::Blue;
    }
};
