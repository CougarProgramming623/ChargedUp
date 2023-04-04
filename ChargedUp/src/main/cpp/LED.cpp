#include "LED.h"
#include "Util.h"
#include "Constants.h"
#include "Robot.h"

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


const int kNum_LED_Eyes = 28;
const int kNum_LED_Board = 40;

frc::Color colorArray[] = {frc::Color::kRed, frc::Color::kOrangeRed, frc::Color::kYellow, frc::Color::kGreen, frc::Color::kBlue, frc::Color::kPurple, frc::Color::kWhite};
frc::Color redWhiteArray[] = {frc::Color::kWhite, frc::Color::kRed};

LED::LED()  :
    m_YellowButton(BUTTON_L_TWO(LED_YELLOW)),
    m_PurpleButton(BUTTON_L_TWO(LED_PURPLE))
{}

void LED::Init(){
    m_Eyes.SetLength(kNum_LED_Eyes);
    m_Eyes.Start();
    m_Sponsorboard.SetLength(kNum_LED_Board);
    m_Sponsorboard.Start();
    m_IterationTracker = 0;
    //SponsorBoardAllianceColor();
    EyesAllianceColor();
} 

void LED::LowBattery(){    
   for(int i = 0; i < 10; i++){
        for(int j = (i*4) + m_IterationTracker; j < ((i+1)*4) + m_IterationTracker; j++){
            m_BoardBuffer[j % kNum_LED_Board].SetLED(frc::Color::kGreen);
            m_BoardBuffer[j + 4 % kNum_LED_Board].SetLED(frc::Color::kRed);
        }
    }
    
    m_IterationTracker++;
    if (m_IterationTracker == kNum_LED_Board){
        m_IterationTracker = 0;
    }
}

void LED::SetEyesData(){ m_Eyes.SetData(m_EyesBuffer); }
void LED::SetBoardData(){ m_Sponsorboard.SetData(m_BoardBuffer); }

void LED::EndGame(){
    if ((int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) <= 30 && (int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) > 28){
        if ((int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) % 1 == .5 || (int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) % 1 == 0){
            SponsorBoardSolid(frc::Color::kWhite);
        } else {
            SponsorBoardSolid(0,0,0);
        }
    } else if ((int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) <= 20){
        if ((int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) % 1 == .5 || (int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) % 1 == 0){
            SponsorBoardSolid(frc::Color::kWhite);
        } else {
            SponsorBoardSolid(0,0,0);
        }
    }
}

void LED::Cube(){ 
    if(Robot::GetRobot()->GetButtonBoard().GetRawButton(19)){
        EyesSolid(frc::Color::kYellow);
    }
}
void LED::Cone(){ 
    if(Robot::GetRobot()->GetButtonBoard().GetRawButton(18)){
        EyesSolid(frc::Color::kViolet);
    }
}

void LED::SponsorBoardAllianceColor(){
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
        SponsorBoardSolid(frc::Color::kRed);
    } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
        SponsorBoardSolid(frc::Color::kBlue);
    } else {
        SponsorBoardSolid(frc::Color::kPurple);
    }
}

void LED::SponsorBoardSolid(frc::Color color){
    for(int i = 0; i < kNum_LED_Board; i++){
            m_BoardBuffer[i].SetLED(color);
    }
}

void LED::SponsorBoardSolid(int R, int G, int B){
    for(int i = 0; i < kNum_LED_Board; i++){
        m_BoardBuffer[i].SetRGB(R, G, B);
    }
}

/*
Start from 255 0 0, then count up g to 255 255 0, then count down red to 0 255 0,
then count up blue to 0 255 255, then count down green to 0 0 255,
then count up red to 255 0 255, then count down blue to 255 0 0.
*/
void LED::SponsorBoardRainbow(){
    for (int i = 0; i <= 255; i++){
        m_BoardBuffer[i % kNum_LED_Board].SetRGB(i, 255, 255);
    }
    // m_IterationTracker++;
    // if (m_IterationTracker == kNum_LED_Board){
    //     m_IterationTracker = 0;
    // }
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

void LED::EyesSolid(frc::Color color){
    for(int i = 0; i < kNum_LED_Eyes; i++){
        m_EyesBuffer[i].SetLED(color);
    }
}

void LED::EyesSolid(int R, int G, int B){
    for(int i = 0; i < kNum_LED_Eyes; i++){
        m_EyesBuffer[i].SetRGB(R, G, B);
    }
}

void LED::EyesAngry(){}
void LED::EyesSleepy(){}
void LED::EyeRoll(){}
