#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>

//#include "Util.h"


Class LED {

    public:

        LED();
        void LEDInit();

        void LowBatery();

        void SponsorBoardSolid(frc::Color allianceColor);
        void SponsorBoardSolid(int R, int G, int B);

        void SponsorBoardFlash(frc::Color allianceColor);     
        void SponsorBoardFlash(int R, int G, int B);     

        void EyesSolid(frc::Color allianceColor);
        void EyesSolid(int R, int G, int B);

        void EyesAngry();
        void EyesSleepy();
        void EyeRoll();

    private:

        //set values
        const int kSTART_BL = 0;
        const int kSTART_TL = 0;
        const int kSTART_TR = 0;
        const int kSTART_BR = 0;

        const int kSTART_EYE_1  = 0;
        const int kSTART_EYE_2  = 0;

};