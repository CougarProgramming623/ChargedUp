#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>

//#include "Util.h"


Class LED {

    public:

        LED();
        void LEDInit();
        void PaintSolid(frc::Color allianceColor);
        void PaintSolid(int R, int G, int B);
        void LowBatery();



    private:

        const int kSTART_BL = 13;
        const int kSTART_TL = 39;
        const int kSTART_TR = 74;
        const int kSTART_BR = 100;

};