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

frc::Color colorArray[] = {frc::Color::kRed, frc::Color::kOrangeRed, frc::Color::kYellow, frc::Color::kGreen, frc::Color::kBlue, frc::Color::kViolet, frc::Color::kWhite};
frc::Color redWhiteArray[] = {frc::Color::kWhite, frc::Color::kRed};

LED::LED(){}

void LED::Init(){
    m_AddressableLED.SetLength(kNum_LED);
    m_AddressableLED.Start();
    m_IterationTracker = 0;
    //SponsorBoardAllianceColor();
    EyesAllianceColor();
} 

void LED::LowBattery(){
    for(int i = 0; i <= 11; i++){
        m_LEDBuffer[i * 10 - 1].SetLED(frc::Color::kRed);
    }
    
    // for (int j = m_IterationTracker; j < 11 + m_IterationTracker; j++){
    //     m_LEDBuffer[j % kNum_LED].SetLED(colorArray[0]);
    // }
    
    // for(int i = 0; i < 2; i++){
    //     // DebugOutF(std::to_string(i));
    //     for(int j = (i*11) + m_IterationTracker; j < ((i+1)*11) + m_IterationTracker; j++){
    //         m_LEDBuffer[j % kNum_LED].SetLED(redWhiteArray[i]);
    //     }
    // }

    // m_IterationTracker++;
    // if (m_IterationTracker == 110){
    //     m_IterationTracker = 0;
    // }

    m_AddressableLED.SetData(m_LEDBuffer);
}



void LED::SponsorBoardAllianceColor(){
    if(/*COB_GET_ENTRY(COB_KEY_IS_RED).getBoolean(false)*/ true){
        SponsorBoardSolid(frc::Color::kRed);
    } else {
        SponsorBoardSolid(frc::Color::kBlue);
    }
}

void LED::SponsorBoardSolid(frc::Color color){
    for(int i = 0; i < kNum_LED; i++){
            m_LEDBuffer[i].SetLED(color);
    }
    m_AddressableLED.SetData(m_LEDBuffer);
}

void LED::SponsorBoardSolid(int R, int G, int B){
    for(int i = 0; i < kNum_LED; i++){
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
    for(int i = 0; i < 6; i++){
        // DebugOutF(std::to_string(i));
        for(int j = (i*18) + m_IterationTracker; j < ((i+1)*18) + m_IterationTracker; j++){
            m_LEDBuffer[j % kNum_LED].SetLED(colorArray[i]);
        }
    }
    // DebugOutF(std::to_string(m_IterationTracker));
    m_IterationTracker++;
    if (m_IterationTracker == 110){
        m_IterationTracker = 0;
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

// void LED::EyesSolid(frc::Color){
//     for(int i = kSTART_EYE_1; i < kEND_EYE_1; i++)
//         m_LEDBuffer[i].SetLED(frc::Color::kYellow);
//     for(int i = kSTART_EYE_2; i < kEND_EYE_2; i++)
//         m_LEDBuffer[i].SetLED(frc::Color::kYellow);
// }
// void LED::EyesSolidPurple(frc::Color){
//     for(int i = kSTART_EYE_1; i < kEND_EYE_1; i++)
//         m_LEDBuffer[i].SetLED(frc::Color::kPurple);
//     for(int i = kSTART_EYE_2; i < kEND_EYE_2; i++)
//         m_LEDBuffer[i].SetLED(frc::Color::kPurple);
// }
void LED::EyesSolid(int R, int G, int B){}

void LED::EyesAngry(){}
void LED::EyesSleepy(){}
void LED::EyeRoll(){}