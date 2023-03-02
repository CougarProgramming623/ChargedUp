#include "LED.h"

//set values
const int kSTART_BL = 0;
const int kSTART_TL = 0;
const int kSTART_TR = 0;
const int kSTART_BR = 0;

const int kSTART_EYE_1  = 0;
const int kEND_EYE_1    = 0;
const int kSTART_EYE_2  = 0;
const int kEND_EYE_2    = 0;

const int kNUM_LED = -1;

frc::AddressableLED m_LED{9};
std::array<frc::AddressableLED::LEDData, 280> m_LEDBuffer; //FIX Length

void LED::LowBatery(){}

void LED::SponsorBoardAlianceColor(){}
void LED::SponsorBoardSolid(frc::Color color){}
void LED::SponsorBoardSolid(int R, int G, int B){}

void LED::SponsorBoardFlash(frc::Color color){}     
void LED::SponsorBoardFlash(int R, int G, int B){}

void LED::EyesAlianceColor(){

}
frc2::InstantCommand LED::EyesSolid(frc::Color color){

    for(int i = kSTART_EYE_1; i < kEND_EYE_1; i++)
        m_LEDBuffer[i].SetLED(color);
    for(int i = kSTART_EYE_2; i < kEND_EYE_2; i++)
        m_LEDBuffer[i].SetLED(color);
    return frc2::InstantCommand();
}
void LED::EyesSolid(int R, int G, int B){}

void LED::EyesAngry(){}
void LED::EyesSleepy(){}
void LED::EyeRoll(){}