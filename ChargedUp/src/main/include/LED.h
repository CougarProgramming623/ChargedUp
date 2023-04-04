#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Button.h>

//#include "Util.h"

class LED{
    
    public:
    
        LED();
        void Init();

        void LowBattery();

        void SetEyesData();
        void SetBoardData();
        void EndGame();
        void Cube();
        void Cone();

        void SponsorBoardAllianceColor();
        void SponsorBoardSolid(frc::Color allianceColor);
        void SponsorBoardSolid(int R, int G, int B);

        void SponsorBoardRainbow();

        void SponsorBoardFlash(frc::Color allianceColor);     
        void SponsorBoardFlash(int R, int G, int B);     

        void EyesAllianceColor();
        void EyesSolidYellow(frc::Color);
        void EyesSolidPurple(frc::Color);
        void EyesSolid(frc::Color allianceColor);
        void EyesSolid(int R, int G, int B);

        void EyesAngry();
        void EyesSleepy();
        void EyeRoll();



    private:

        frc::AddressableLED m_Eyes{9};
        frc::AddressableLED m_Sponsorboard{0};
        std::array <frc::AddressableLED::LEDData, 28> m_EyesBuffer;
        std::array <frc::AddressableLED::LEDData, 40> m_BoardBuffer; //FIX Length
        int m_IterationTracker;

        frc2::Button m_YellowButton;
        frc2::Button m_PurpleButton;
};