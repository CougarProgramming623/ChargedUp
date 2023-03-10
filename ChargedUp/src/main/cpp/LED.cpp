#include "LED.h"
#include "Util.h"

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


const int kNum_LED = 110;

frc::Color colorArray[] = {frc::Color::kBlue, frc::Color::kGreen, frc::Color::kYellow,frc::Color::kOrangeRed, frc::Color::kRed, frc::Color::kBlue, frc::Color::kGreen, frc::Color::kYellow,frc::Color::kOrangeRed, frc::Color::kRed};


LED::LED(){}

void LED::Init(){
    m_AddressableLED.SetLength(kNum_LED);
    m_AddressableLED.Start();
    m_IterationTracker = 0;
    //SponsorBoardAllianceColor();
    EyesAllianceColor();
} 

void LED::LowBattery(){
}



void LED::SponsorBoardAllianceColor(){
    if(/*COB_GET_ENTRY(COB_KEY_IS_RED).getBoolean(false)*/ true){
        SponsorBoardSolid(frc::Color::kRed);
    } else {
        SponsorBoardSolid(frc::Color::kBlue);
    }
}

void LED::SponsorBoardSolid(frc::Color color){
    for(int i = 0; i < 280; i++){
            m_LEDBuffer[i].SetLED(color);
    }
    m_AddressableLED.SetData(m_LEDBuffer);
}

void LED::SponsorBoardSolid(int R, int G, int B){
    for(int i = 0; i < 280; i++){
        m_LEDBuffer[i].SetRGB(R, G, B);
    }
    m_AddressableLED.SetData(m_LEDBuffer);
}


/*
Start from 255 0 0, then count up g to 255 255 0, then count down red to 0 255 0,
then count up blue to 0 255 255, then count down green to 0 0 255,
then count up red to 255 0 255, then count down blue to 255 0 0.
*/
void LED::SponsorBoardRainbow(){
    for(int i = 0; i < 8; i++){
        DebugOutF(std::to_string(i));
        for(int j = (i*7); j < (i+1)*7; j++){
            DebugOutF(std::to_string(j));
            m_LEDBuffer[j].SetLED(colorArray[i]);
        }
    }
    m_AddressableLED.SetData(m_LEDBuffer);
}

void LED::SponsorBoardFlash(frc::Color color){
    if(m_IterationTracker % 2 == 0){
        SponsorBoardSolid(color);
    } else {
        SponsorBoardSolid(0, 0, 0);
    }

}     
void LED::SponsorBoardFlash(int R, int G, int B){
    if(m_IterationTracker % 2 == 0){
        SponsorBoardSolid(R, G, B);
    } else {
        SponsorBoardSolid(0, 0, 0);
    }
}

void LED::EyesAllianceColor(){}

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