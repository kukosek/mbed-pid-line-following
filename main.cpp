#include "mbed.h"
const int FREQUENCY=20; //menitelne
float WAITTIME;
DigitalOut led(LED1);

//levý line sensor
DigitalIn line_A1(PB_4);
DigitalIn line_A2(PB_1);
DigitalIn line_A3(PA_8);
//pravý line sensor
DigitalIn line_B1(PA_0);
DigitalIn line_B2(PA_1);
DigitalIn line_B3(PA_10);

DigitalOut driver_enable(PA_4);
//motory
DigitalOut driver_A2(PA_5);
PwmOut driver_A_speed(PA_11); //Nejmensi dutycycle 0us (1000), nejvetsi 40000us

DigitalOut driver_B2(PA_9);
PwmOut driver_B_speed(PB_0); //Nejmensi dutycycle 40000us, nejvetsi 0 (1000)

//dutycycly motoru
const int driver_A_min_dutycycle_us=0;
const int driver_A_center_dutycycle_us=10000;
const int driver_A_max_dutycycle_us=20000;

const int driver_B_min_dutycycle_us=20000;
const int driver_B_center_dutycycle_us=10000;
const int driver_B_max_dutycycle_us=0;

int driver_A_reverseSpeeds=false; //toto se samo nastavi na true pokud je driver max dutycycle mensi nez min dutycycke
int driver_B_reverseSpeeds=false; //toto se samo nastavi na true pokud je driver max dutycycle mensi nez min dutycycke

const int leftBaseSpeed=18000;
const int rightBaseSpeed=18000;

int leftSpeed;
int rightSpeed;

//Konstanty stavoveho automatu (nemenit)
const int NEJEDU=0;
const int JEDU=1;
const int BRZDIM=2;
const bool DOPREDU=true;
const bool DOZADU=false;

//Promenne pro meneni stavuu (nemenit)
bool amIarleadyOutOfFullLine=false;
int lineSeenCount=0;

int stav=NEJEDU; //Promenna stavoveho automatu (=) defaultni nastaveni, klidne menit

//Konstanty pro readLinePos
const int sensorCount=2; //pocet sensoru
int line_A;
int line_B;
//int line_C
//int line_D
//...

//Promenne pro readLinePos, nemenit
int lineList[sensorCount];
int fullLineCount=0; //Kolik sensoru vidi caru na vsech diodach? (Budem pouzivat k zastavovani)
bool fullLine=false; //Je fullLineCount roven sensorCount? jestli jo tak menime stavy...
int sensorLinePos;
float linePos; //finalni hodnota
float lastLinePos=10*sensorCount/2; //v teto promene se zaznamenava posledni hodnota cary. Pouziva se pro osetreni linePos v pripade, ze ztratil caru vpravo. Je nastavena na pulku sensoru, protoze pri prvnim pouziti by jinak byla undefined

//PID konstanty
float setpointValue=5.0;
double Kp=200;
float Kd=15;
double Ki=0;

//PID promenne
int currentTime; //promena pro integraci. prepisuje se v kazde smycce na us_ticker_read(), coz je pocet mikrosekund uplynulych od zapnuti desky
int lastTime; //promena pro integraci. lastTime je predchozi cas vykonani smycky. prepise se na konci smycky na current_time.
int elapsedTime=0; //promena pro integraci. currentTime-lastTime=ElapsedTime (uplynuly cas od posledniho vypoctu v mikrosekundach)
double cumError=0.0; //dlouhodoba promena pro integraci. Pricitaji se k ni hodnoty v kazde smycce, resetuje se pri ztrate cary

int lineError=0; //chyba cary. je prepsana v kazde smycce na linePos-setpointValue.
int lastError=0; //posledni chyba cary. prepisuje se na konci smycky na LineError
double motorSpeedChange=0.0; //Vysledna hodnota z vypoctu PID. Po vypoctu PID je pouzita ve voidu setMotorSpeeds.


void readLinePos(){
    sensorLinePos=0;
    linePos=0;
    fullLineCount=0;
    
    line_A=1*line_A1+2*line_A2+4*line_A3;
    line_B=1*line_B1+2*line_B2+4*line_B3;
    //line_C=1*line_C1+2*line_C2+4*line_C3;
    //...
    
    int lineList[sensorCount]={line_A, line_B}; //zadat sensory
    
    /*
    Nyni prevedem tyto divne cisla na smysluplne hodnoty (pro cernou caru)
    Divne - Smysluplne - Info
    0       0            nic nevidim
    4       2            uplne vlevo
    6       4            mirne vlevo
    2       5            stred
    3       6            mirne vpravo
    1       8            uplne vpravo
    7       10           vsechny
    */
    for (int i = 1; i <= sensorCount; ++i){ //for each kazdy sensor-modul
        switch(lineList[i-1]){ //Prevod hodnot z modulu na smysluplna skoro linearni cisla. Pocita se s tim, ze jsou vsechny sensory stejne zapojene
                case 0:
                    sensorLinePos=0;
                    break;
                case 4:
                    sensorLinePos=2;
                    break;
                case 6:
                    sensorLinePos=4;
                    break;
                case 2:
                    sensorLinePos=5;
                    break;
                case 3:
                    sensorLinePos=6;
                    break;
                case 1:
                    sensorLinePos=8;
                    break;
                case 7: //vidi vsechny
                    fullLineCount++;
                    sensorLinePos=5;
                    break;
                default:
                    sensorLinePos=0;
        }
        /* 
        (pro bilou)
        Divne - Smysluplne - Info
        7       0            nic nevidim
        3       2            uplne vlevo
        1       4            mirne vlevo
        5       5            stred
        4       6            mirne vpravo
        6       8            uplne vpravo
        0       10           vsechny
        NASLEDUJICI ODKOMENTOVAT POKUD JE CARA BILA
        switch(lineList[i]){ //Pocita se s tim, ze jsou vsechny sensory stejne zapojene
                case 7:
                    sensorLinePos=0;
                    break;
                case 3:
                    sensorLinePos=2;
                    break;
                case 1:
                    sensorLinePos=4;
                    break;
                case 5:
                    sensorLinePos=5;
                    break;
                case 4:
                    sensorLinePos=6;
                    break;
                case 6:
                    sensorLinePos=8;
                    break;
                case 0:
                    fullLineCount++;
                    sensorLinePos=5;
                    break;
                default:
                    sensorLinePos=0;
        }
        */
        
        //zmena celkove pozice podle hodnoty sensorlinepos
        if (sensorLinePos!=0 && linePos==0){
            linePos=linePos+sensorLinePos+(i-1)*10;
        }else if (sensorLinePos!=0 && linePos!=0) { //pokud uz byl predtim na nejakem ze sensoru nejaka hodnota jina nez nula
            linePos=(linePos+sensorLinePos+(i-1)*10)/i; //tak se hodnoty ze sensoruu zprumeruji
        }
    }
    
    
    if (linePos==0 && lastLinePos>=sensorCount*10/2){ //Osetreni: pokud ztratil caru (nevidi nic) a posledni zaznamenana cara byla na prave pulce vsech diod vsech sensoru
        linePos=sensorCount*10+2; //zmen hodnotu linePos z 0 na: nejvetsi hodnotu linePos pokud vidi caru plus 2
    }
    
    
    if (fullLineCount==sensorCount){ //kdyz vidim vsemi diodami vsech sensoru line
        amIarleadyOutOfFullLine=false;
        fullLine=true;    
    }else{
        fullLine=false;
        if (amIarleadyOutOfFullLine==false){
            amIarleadyOutOfFullLine=true;
            if (stav==NEJEDU){
                stav++; //zmen stav na dalsi
                lineSeenCount=0;
            }else{
            }
            if (lineSeenCount==0 && stav==JEDU){
                lineSeenCount++;
            }else if(lineSeenCount==1 && stav==JEDU){
                lineSeenCount++;
            }else if(lineSeenCount==2 && stav==JEDU){
                lineSeenCount++;
                stav++; 
            }
            cumError=0;
            lastLinePos=0;
            if (stav==3){ //pokud je cislo stavu vetsi nez pocet stavu
                stav=0; //vrat stav na nejedu
                
            }
        }
    }
    lastLinePos=linePos; //obnoveni lastLinePos na aktualni hodnotu
    
}

void calculatePID(){
    currentTime=us_ticker_read();
    elapsedTime=currentTime-lastTime; //kolik ubehlo us od posledniho vypoctu
    //Vypocitani chyby
    lineError=linePos-setpointValue;
    //Obnova integrovaneho lineErroru za cas
    cumError+=lineError*elapsedTime;
    
    //Vypocitani vysledneho PID ( o kolik se maji zmenit rychlosti)
    motorSpeedChange=Kp*lineError+Ki*cumError+Kd*((lineError-lastError)/elapsedTime);
    //Ulozeni momentalnich input hodnot pro pristi vypocet
    lastError=lineError;
    lastTime=currentTime;
}

void setMotorDirections(bool smer){
    if (smer==DOPREDU){
        driver_A2=0;
        driver_B2=1;   
    }else if (smer==DOZADU){
        driver_A2=0;
        driver_B2=1;
    }
}

int reverseSpeed(int speed, int center){ //prevrati dutycycli v zavislosti na centru dutycyclu
    return center-speed+center;
}

void setMotorSpeeds(){
    if (driver_A_reverseSpeeds){
        leftSpeed=reverseSpeed(leftBaseSpeed-motorSpeedChange, driver_A_center_dutycycle_us);
    }else{
        leftSpeed=leftBaseSpeed+motorSpeedChange;
    }
    
    if (driver_B_reverseSpeeds){
        rightSpeed=reverseSpeed(rightBaseSpeed-motorSpeedChange, driver_B_center_dutycycle_us);
    }else{
        rightSpeed=rightBaseSpeed+motorSpeedChange;
    }
    driver_A_speed.pulsewidth_us(leftSpeed);
    driver_B_speed.pulsewidth_us(rightSpeed);
    
}



int main() {
    Serial pc(USBTX, USBRX);
    driver_enable=1;
    pc.puts("\nHello world! Starting loop.\n\n");
    if (driver_A_max_dutycycle_us<driver_A_min_dutycycle_us){
       driver_A_reverseSpeeds=true;
    }
    
    if (driver_B_max_dutycycle_us<driver_B_min_dutycycle_us){
       driver_B_reverseSpeeds=true; 
    }
    
    setMotorDirections(DOPREDU);
    //nastaveni frekvencni na pwm ovladani motoru
    driver_A_speed.period_ms(20);
    driver_B_speed.period_ms(20);
    //vypnuti motoru
    stopMotors();
    WAITTIME=1.0/FREQUENCY;
    while(1) {
        //wait(WAITTIME);
        
        readLinePos();
        if (linePos==0||linePos==22){ //nedynamickke
            cumError=0; //reset integralu pid
        }
        
        if (stav==NEJEDU){
            driver_enable=0; //to by se nemelo opakovat poroad dokola ale jinak to nefunguje xddd
        }else if (stav==JEDU){
            driver_enable=1; //to by se taky nemelo opakovat poroad dokola ale jinak to nefunguje ekzfy
            calculatePID();
            setMotorSpeeds();
        }else if (stav==BRZDIM){
            stav=NEJEDU;
        }
        //pc.printf("stav: %d, leftSpeed: %d, rightSpeed: %d linePos: %f, microsecnds: %d\n",stav,leftSpeed, rightSpeed, linePos,elapsedTime);
    }
}

