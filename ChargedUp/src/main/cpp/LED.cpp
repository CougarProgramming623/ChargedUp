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
using frc::Color;

frc::Color colorArray[] = {frc::Color::kRed,frc::Color::kOrange, frc::Color::kYellow, frc::Color::kGreen, frc::Color::kBlue, frc::Color::kViolet, frc::Color::kPink };
LED::LED(){}

void LED::Init(){
    m_AddressableLED.SetLength(280);
    m_AddressableLED.Start();
}

void LED::LowBatery(){
    for(int i = 0; i < 7; i++){
        for(int j = i*40; j < (i+1)*40; j++){
            m_LEDBuffer[j].SetLED(colorArray[i]);
        }
    }
    m_AddressableLED.SetData(m_LEDBuffer);
}

void LED::SponsorBoardAlianceColor(){
    for(int i = 0; i < 280; i++){
        m_LEDBuffer[i].SetLED(frc::Color::kBlue);
    }
    m_AddressableLED.SetData(m_LEDBuffer);
}
void LED::SponsorBoardSolid(frc::Color color){}
void LED::SponsorBoardSolid(int R, int G, int B){}

void LED::SponsorBoardFlash(frc::Color color){}     
void LED::SponsorBoardFlash(int R, int G, int B){}

void LED::EyesAlianceColor(){}

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