#include "LED.h"

void LED::LowBatery(){}

void LED::SponsorBoardAlianceColor(){}
void LED::SponsorBoardSolid(frc::Color color){}
void LED::SponsorBoardSolid(int R, int G, int B){}

void LED::SponsorBoardFlash(frc::Color color){}     
void LED::SponsorBoardFlash(int R, int G, int B){}

void LED::EyesAlianceColor(){

}
void LED::EyesSolid(frc::Color color){

    for(int i = kSTART_EYE_1; i < kEND_EYE_1; i++)
        m_LEDBuffer[i].SetLED(color);
    for(int i = kSTART_EYE_2; i < kEND_EYE_2; i++)
        m_LEDBuffer[i].SetLED(color);
}
void LED::EyesSolid(int R, int G, int B){}

void LED::EyesAngry(){}
void LED::EyesSleepy(){}
void LED::EyeRoll(){}