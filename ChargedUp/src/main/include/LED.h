#pragma once

#include <frc/util/Color.h>
#include <frc/AddressableLED.h>

class LED{
    
    public:
    
        LED();
        void Init();

        void SponsorBoardSolid(frc::Color color);

    private:

	    frc::AddressableLED m_AddressableLED{9};
        std::array <frc::AddressableLED::LEDData, 28> m_LEDBuffer; //FIX Length


};