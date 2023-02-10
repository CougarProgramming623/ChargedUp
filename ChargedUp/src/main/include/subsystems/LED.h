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

        //set values
        const int kSTART_BL = 0;
        const int kSTART_TL = 0;
        const int kSTART_TR = 0;
        const int kSTART_BR = 0;

};