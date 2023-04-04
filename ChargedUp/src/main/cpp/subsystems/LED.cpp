#include "./subsystems/LED.h"

const int kNum_LED = 28;

LED::LED(){}

void LED::Init(){
    m_AddressableLED.SetLength(kNum_LED);
    m_AddressableLED.Start();
    //m_IterationTracker = 0;
    //SponsorBoardAllianceColor();
    //EyesAllianceColor();
    SponsorBoardSolid(frc::Color::kPurple);
} 

void LED::SponsorBoardSolid(frc::Color color){
    for(int i = 0; i < kNum_LED; i++){
            m_LEDBuffer[i].SetLED(color);
    }
    m_AddressableLED.SetData(m_LEDBuffer);
}
    