#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>
#include <frc2/command/InstantCommand.h>


//#include "Util.h"

class LED{
    
    public:
    
        LED();
        void Init();

        void LowBattery();

        void SponsorBoardAllianceColor();
        void SponsorBoardSolid(frc::Color allianceColor);
        void SponsorBoardSolid(int R, int G, int B);

        void SponsorBoardRainbow();

        void SponsorBoardFlash(frc::Color allianceColor);     
        void SponsorBoardFlash(int R, int G, int B);     

        void EyesAllianceColor();
        void EyesSolid(frc::Color allianceColor);
        void EyesSolid(int R, int G, int B);

        void EyesAngry();
        void EyesSleepy();
        void EyeRoll();



    private:

        frc::AddressableLED m_AddressableLED{9};
        std::array <frc::AddressableLED::LEDData, 110> m_LEDBuffer; //FIX Length
        int m_IterationTracker;
};