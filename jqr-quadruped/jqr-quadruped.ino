//tratta da 3056C

//e spinta zampe posteriori forse i polpacci si devono aprire prima!!

//LIBRERIA WIRE MODIFICATA! riferirsi al sito per twi.c e twi.h modificate e istruzioni!
//ALTRA LIBREIRA MODIFICATA : MPU6050_6Axis_MotionApps20.h 
//VALORI UTILIZZATI : 0x02,   0x16,   0x02,   0x00, 0x03 // D_0_22 inv_set_fifo_rate 


/*
prova a :

A) Se il robot sta perdendo l'equilibrio durante fase ricarica: BLOCCARE RICARICA E PENSARE AD EQUILIBRIO!!
Magari come inizia fase ricarica, il robot puo' alzare leggermente la zampa per vedere se perde l'equilibrio!
Se SI interrompe e la zampa,dato che era appena alzata, torna subito al suo posto!
Se NO allora continua la fase ricarica!

B) la zampa che ha appena caricato NON deve essere utilizzata per finire l'equilibrio fatto in pre ricarica

C) Provare a mettere il peso del robot nella zampa che ha appena ricaricato in fase di spinta!

     

   
                                                            E) RIDURRE VALORE NUMERICO ANGOLI ROLL E PITCH!
                                                               ora sono troppo alti per l'inclinazione reale!
                                                               RISULTATO:
                                                               SE SI RIDUCE IN 90/M_PI, I GRADI AUMENTANO E DIMINUISCONO MENO DURANTE INCLINAZIONE!
                                                               Primo pensiero che viene e' che, con le attuali routine, si perde di precisione nell'
                                                               equilibrio del robot! ma ci sarebbe da approfondire il discorso e magari provare
                                                               un valore tra 180 e 90, diciamo 135 per trovare un compromesso che ci soddisfi piu' dell'
                                                               equilibrio settato con 180/M_PI    

   
D) e considera anche che potresti fare che una zampa quando 
troppo allungata (tipo zampe posteriori in spinta) allora
riduce o proprio non fa movimento equilibrio!


   
*/

/*
fare una routine che equilibria il robot, SOLO(?) in fase di ricarica, utilizzando i sensori tattili
e cercando di portare, se ad esempio si sta caricando zampa 1, tale zampa a un leggerissimo 
contatto con il terreno rispetto alle altre zampe.
Sarebbe gia' pronta per caricare senza rischio di far cadere il robot
posso gia' usare i tasti set target su processing!

EQUILIBRIO ZAMPA 1:
A) prima si agisce lateralmente, facendo inclinare JQR a sinistra aspettando che la pressione
della zampa 1 entri nel "leggerissimo"

B) Se l'inclinazione laterale non basta, parte l'inclinazione indietro. A fine corsa si dovrebbe
   aver ottenuto la zampa 1 con pressione nel terreno "leggerissima"
   

A e B possono anche partire e avanzare insieme eh!

oppure:
ALZARE LEGGERMENTE (braccio+avambraccio) LA ZAMPA OPPOSTA (quindi zampa 4) COME 
QUANDO SI RICARICA MA SOLTANTO DI POCHISSIMO, IL TANTO CHE BASTA PER FARE 
UNA PICCOLA CADUTA DALLA PARTE OPPOSTA ALLA ZAMPA CHE DEVE RICARICARE (LA 1)
*/


#include "MemoryFree.h" 
//#include <SoftwareSerial.h>
//#include "avr/wdt.h" WATCHDOG


//--------------------------------- GYRO
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
    //#include "NBWire.h"
#endif

MPU6050 accelgyro(0x69); //serve? cmq prova anche con 69

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION

//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

// ZAMPA 1 -AVANTI DESTRA
//AVAMBRACCIO 1
#define SERVOMIN_ava1  200 //180 - 630
#define SERVOMAX_ava1  650

//BRACCIO 1 -- ORA E' SPALLA ZAMPA 4
#define SERVOMIN_bra1  165 //180
#define SERVOMAX_bra1  615 // 640

//SPALLA 1
#define SERVOMIN_spa1  180 //330-600 piu aumenta piu la spalla si allarga
#define SERVOMAX_spa1  630

//-------------------------------------------------------------------

// ZAMPA 2 -AVANTI SINISTRA
//AVAMBRACCIO 2
#define SERVOMIN_ava2  150 //era 190 640 ora devo usare 160 610 alzandolo si abbassa l'avambraccio
#define SERVOMAX_ava2  600

//BRACCIO 2
#define SERVOMIN_bra2  165 //braccio deve andare leggerm avanti riducendo il valore!
#define SERVOMAX_bra2  615 

//SPALLA 2
#define SERVOMIN_spa2  100 //120
#define SERVOMAX_spa2  550 //570

//-------------------------------------------------------------------

// ZAMPA 3 -DIETRO DESTRA
//AVAMBRACCIO 3
#define SERVOMIN_ava3  150 
#define SERVOMAX_ava3  600 

//BRACCIO 3
#define SERVOMIN_bra3  150       //140-590
#define SERVOMAX_bra3  600

//SPALLA 3
#define SERVOMIN_spa3  70 //90-540
#define SERVOMAX_spa3  520 

//-------------------------------------------------------------------

// ZAMPA 4 -DIETRO SINISTRA
//AVAMBRACCIO 4
#define SERVOMIN_ava4  150 
#define SERVOMAX_ava4  600 //prova abbassandolo

//BRACCIO 4
#define SERVOMIN_bra4  130 //164-600 alzando il valore torna indietro
#define SERVOMAX_bra4  580

//SPALLA 4 
#define SERVOMIN_spa4  90 //90-540
#define SERVOMAX_spa4  540 


// our servo # counter
uint8_t servonum = 0;


//---------------------------- SENSORI TATTILI -----------------------------
    int fsrAnalogPin1 = 0; // FSR is connected to analog 0
    int fsrAnalogPin2 = 1; // FSR is connected to analog 1
    int fsrAnalogPin3 = 2; // FSR is connected to analog 2 
    int fsrAnalogPin4 = 3; // FSR is connected to analog 3
    int fsrReading1; // the analog reading from the FSR resistor divider
    int fsrReading2; // the analog reading from the FSR resistor divider
    int fsrReading3; // the analog reading from the FSR resistor divider
    int fsrReading4; // the analog reading from the FSR resistor divider
    int fsrReading1_check = 0, fsrReading2_check = 0, fsrReading3_check = 0, fsrReading4_check = 0;
//--------------------------------------------------------------------------
//per camminata furtiva
int fase_mov_zampe = 0, fase_mov_zampa2 = 0, fase_mov_zampa3 = 0, fase_mov_zampa4 = 0;
String equilibrio_str = "";

//per camminata test (furtiva 2)
int routine_equilibrio = 1;// 1 --> on / 0 --> off routine equilibrio
int target_x = 0, target_y = 0;
int valore_tar_sin2 = 0,valore_tar_sin4 = 0, valore_tar_des1 = 0, valore_tar_des3 = 0;
int valore_tar_sin2b = 0,valore_tar_sin4b = 0, valore_tar_des1b = 0, valore_tar_des3b = 0;
int valore_tar_des1_SOMMA = 0, valore_tar_des3_SOMMA = 0, valore_tar_sin2_SOMMA = 0, valore_tar_sin4_SOMMA = 0;
int valore_tar_des1_FIN = 0, valore_tar_des3_FIN = 0, valore_tar_sin2_FIN = 0, valore_tar_sin4_FIN = 0;
int valore_vel_des1 = 0, valore_vel_des3 = 0, valore_vel_sin2 = 0, valore_vel_sin4 = 0;
int valore_vel_des1b = 0, valore_vel_des3b = 0, valore_vel_sin2b = 0, valore_vel_sin4b = 0;
int valore_vel_des1_FIN = 0, valore_vel_des3_FIN = 0, valore_vel_sin2_FIN = 0, valore_vel_sin4_FIN = 0;
int dist_angolo_roll_con_target_x;
int dist_angolo_pitch_con_target_y;
int velocita_mov, velocita_movb;
int flag_kfin = 0;
int pillozzo = 0;
int pillozzoB = 0;
int timer_raggiung_target_x = 0;
int timer_raggiung_target_y = 0;
int timer_generico = 0, timer_generico2 = 0, timer_generico3 = 0;
//int timer_generico3 = 0;
int durata_timer = 0;
int target_x_appo = 0;
int target_y_appo = 0;
int tempo_attesa_appo = 0;
int durata_spinte = 0;
int timer_check_assi = 0;
int fase_equilibrio = 0;
int inclinazione_a = 0; //da togliere
int zona_velocita = 0;
int fase_equilibrio_B = 0, fase_equilibrio_C = 0;
int zampa1_no_equilibrio = 0, zampa2_no_equilibrio = 0, zampa3_no_equilibrio = 0, zampa4_no_equilibrio = 0;
int speed_zona_velocita1 , speed_zona_velocita2;
int riflessi_massimi = 1;
int spinta_zampe = 0;

int targ_crono_vel_equilib = 0;

int pulselengthz;
int pulselengthzb;
int pulselengthzc;

int new_pos_braccio1;
int new_pos_braccio2;
int new_pos_avambraccio1;
int new_pos_avambraccio2;


int new_pos_spalla1;
int new_pos_spalla2;
int new_pos_spalla3;
int new_pos_spalla4;
int new_pos_braccio4;
int new_pos_avambraccio3;
int new_pos_avambraccio4;


//-------------------- CONFIGURAZIONE INIZIALE ZAMPE ><
//-------------------------------ZAMPA NUMERO 1 - AVANTI DESTRA---------------------------
//AVAMBRACCIO 1
int pos_avambraccio1 = 60; // 140=MAX (avambr tutto aperto)//     60        
                           // 0=MIN  (avambr tutto chiuso)
                           // 70=MED
//BRACCIO 1
int pos_braccio1 = 115; // 140=MAX(braccio tutto dietro //   115               
                       // 0=MIN (braccio tutto avanti)
                       // 70=MED
//SPALLA 1
int pos_spalla1 = 60; //120=MAX (spalla tutta chiusa)  95 -valore minore chiude ascella
                      // 0=MIN (spalla tutta aperta)
                      //60 = MED

//-------------------------------ZAMPA NUMERO 2 - AVANTI SINISTRA---------------------------
//AVAMBRACCIO 2
int pos_avambraccio2 = 60; // 140=MAX (avambr tutto aperto)           60
                           // 0=MIN  (avambr tutto chiuso)
                           // 70=MED
//BRACCIO 2
int pos_braccio2 = 115; // 140=MAX(braccio tuto dietro                115
                       // 0=MIN (braccio tutto avanti)
                       // 70=MED
//SPALLA 2
int pos_spalla2 = 60; // 120 =MAX (spalla tutta chiusa)
                      //  0 =MIN (spalla tutta aperta)
                      //  60 = MED




//-------------------------------ZAMPA NUMERO 3 - DIETRO DESTRA---------------------------
//AVAMBRACCIO 3
int pos_avambraccio3 = 60; // 140=MAX (avambr tutto aperto)        60
                           // 0=MIN  (avambr tutto chiuso)
                          // 70=MED   OK 60
                          
//BRACCIO 3
int pos_braccio3 = 35; // 140=MAX(braccio tutto dietro             35
                       // 0=MIN (braccio tutto avanti)
                       // 70=MED     
//SPALLA 3
int pos_spalla3 = 60; //120=MAX (spalla tutta chiusa)
                      // 0=MIN (spalla tutta aperta)
                      //60 = MED

//-------------------------------ZAMPA NUMERO 4 - DIETRO SINISTRA---------------------------
//AVAMBRACCIO 4
int pos_avambraccio4 = 60; // 140=MAX (avambr tutto aperto)         60
                           // 0=MIN  (avambr tutto chiuso)
                          // 70=MED
//BRACCIO 4
int pos_braccio4 = 35; // 140=MAX(braccio tutto dietro              35
                       // 0=MIN (braccio tutto avanti)
                       // 70=MED
//SPALLA 4
int pos_spalla4 = 60; //120=MAX (spalla tutta chiusa)
                      // 0=MIN (spalla tutta aperta)
                      //60 = MED


/*
//-------------------- CONFIGURAZIONE INIZIALE A MO DI SPRINTER IN POSIZIONE
//-------------------------------ZAMPA NUMERO 1 - AVANTI DESTRA---------------------------
//AVAMBRACCIO 1
int pos_avambraccio1 = 60; // 140=MAX (avambr tutto aperto)//             80
                           // 0=MIN  (avambr tutto chiuso)
                           // 70=MED
//BRACCIO 1
int pos_braccio1 = 130; // 140=MAX(braccio tutto dietro //                  90
                       // 0=MIN (braccio tutto avanti)
                       // 70=MED
//SPALLA 1
int pos_spalla1 = 60; //160=MAX (spalla tutta chiusa)  85
                      // 0=MIN (spalla tutta aperta)
                      //90 = MED

//-------------------------------ZAMPA NUMERO 2 - AVANTI SINISTRA---------------------------
//AVAMBRACCIO 2
int pos_avambraccio2 = 65; // 140=MAX (avambr tutto aperto)
                           // 0=MIN  (avambr tutto chiuso)
                           // 70=MED
//BRACCIO 2
int pos_braccio2 = 105; // 140=MAX(braccio tuto dietro 60
                       // 0=MIN (braccio tutto avanti)
                       // 70=MED
//SPALLA 2
int pos_spalla2 = 60; //160=MAX (spalla tutta chiusa)
                      // 0=MIN (spalla tutta aperta)
                      //90 = MED




//-------------------------------ZAMPA NUMERO 3 - DIETRO DESTRA---------------------------
//AVAMBRACCIO 3
int pos_avambraccio3 = 55; // 140=MAX (avambr tutto aperto) 6
                           // 0=MIN  (avambr tutto chiuso)
                          // 70=MED
                          
//BRACCIO 3
int pos_braccio3 = 25; // 140=MAX(braccio tutto dietro 60
                       // 0=MIN (braccio tutto avanti)
                       // 70=MED
//SPALLA 3
int pos_spalla3 = 60; //160=MAX (spalla tutta chiusa)
                      // 0=MIN (spalla tutta aperta)
                      //90 = MED

//-------------------------------ZAMPA NUMERO 4 - DIETRO SINISTRA---------------------------
//AVAMBRACCIO 4
int pos_avambraccio4 = 65; // 140=MAX (avambr tutto aperto) 60
                           // 0=MIN  (avambr tutto chiuso)
                          // 70=MED
//BRACCIO 4
int pos_braccio4 = 45; // 140=MAX(braccio tutto dietro  60             45
                       // 0=MIN (braccio tutto avanti)
                       // 70=MED
//SPALLA 4
int pos_spalla4 = 60; //160=MAX (spalla tutta chiusa)
                      // 0=MIN (spalla tutta aperta)
                      //90 = MED
*/



//int ginolillo = 0;
int pos_avambraccio1_fin = pos_avambraccio1;
int pos_braccio1_fin = pos_braccio1;
int pos_spalla1_fin = pos_spalla1;
int pos_avambraccio2_fin = pos_avambraccio2; 
int pos_braccio2_fin = pos_braccio2;
int pos_spalla2_fin = pos_spalla2;
int pos_avambraccio3_fin = pos_avambraccio3;
int pos_braccio3_fin = pos_braccio3;
int pos_spalla3_fin = pos_spalla3;
int pos_avambraccio4_fin = pos_avambraccio4;
int pos_braccio4_fin = pos_braccio4;
int pos_spalla4_fin = pos_spalla4;

int appo_pos_braccio1_fin, appo_pos_avambraccio1_fin, appo_pos_spalla1_fin;
int appo_pos_braccio2_fin, appo_pos_avambraccio2_fin, appo_pos_spalla2_fin;
int appo_pos_braccio3_fin, appo_pos_avambraccio3_fin, appo_pos_spalla3_fin;
int appo_pos_braccio4_fin, appo_pos_avambraccio4_fin, appo_pos_spalla4_fin;
int ok_leg_back_position = 0;


int val_stand_attesa = 2;
int timer_bra1 = 0, attesa_bra1 = val_stand_attesa;
int timer_ava1 = 0, attesa_ava1 = val_stand_attesa;
int timer_spa1 = 0, attesa_spa1 = val_stand_attesa;
int timer_bra2 = 0, attesa_bra2 = val_stand_attesa;
int timer_ava2 = 0, attesa_ava2 = val_stand_attesa;
int timer_spa2 = 0, attesa_spa2 = val_stand_attesa;
int timer_bra3 = 0, attesa_bra3 = val_stand_attesa;
int timer_ava3 = 0, attesa_ava3 = val_stand_attesa;
int timer_spa3 = 0, attesa_spa3 = val_stand_attesa;
int timer_bra4 = 0, attesa_bra4 = val_stand_attesa;
int timer_ava4 = 0, attesa_ava4 = val_stand_attesa;
int timer_spa4 = 0, attesa_spa4 = val_stand_attesa;

int moto_zampa1 = 0; // 0 = zampa eseguira' codice - 10 = zampa disabilitata
int moto_zampa2 = 0;
int moto_zampa3 = 0;
int moto_zampa4 = 0;

//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//fai ricarica zampa3! prendi dati posizione zampe nel momento in cui ricarica
//la zampa 3 e poi crea la scenetta per provare nuova ricarica (che devi ancora fare!)


int start = 7; //7;      //0= AVVIENE SOLO IL SETTAGGIO INIZIALE - 5= MOVIMENTI ZAMPE TROTTO -7= MOVIMENTO FURTIVO - 10= TEST 
int velocita_spinta; //12
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

int camminata_test = 0;
int incomingByte;
int reset_saltelli = 0;
int timer_ricarica1 = 0, timer_ricarica2 = 0, timer_ricarica3 = 0, timer_ricarica4 = 0;
int posiz_appo = 0;
int posiz_appo2 = 0;

// 0= spinta /  1= ricarica  / 10 = disabilitata spinta
int ok_zampa1 = 0;
int ok_zampa2 = 0;
int ok_zampa3 = 0;
int ok_zampa4 = 0;
int pausa = 0;
int movimento_on = -1;
int movimento_on_flag = 0;//serve nell'attesa prima dello show!
int angolo_rotta_target = 0;
int secondi;
int fase = 0;
String risposta_direzione;

// angolo Z - YAWN BUSSOLA
int angolo_rotta_attuale_reale = 0;
int angolo_rotta_attuale = 0;
int conto_rotta = 0;
int angolo_rotta_attuale_appo1 = 0, angolo_rotta_attuale_appo2 = 0, angolo_rotta_attuale_appo3 = 0;

// angolo X - LATERAL ROLL
float angolo_roll_reale = 0;
int angolo_roll_attuale = 0;
int conto_roll = 0;
int angolo_roll_appo1 = 0, angolo_roll_appo2 = 0, angolo_roll_appo3 = 0;
 
// angolo Y - PITCH
int angolo_pitch_reale = 0;
int angolo_pitch_attuale = 0;
int conto_pitch = 0;
int angolo_pitch_appo1 = 0, angolo_pitch_appo2 = 0, angolo_pitch_appo3 = 0; 

int z; //asse z gyro -yaw- RAW (bussola)
int x; //asse x gyro -roll- RAW
int y; //asse y gyro -pitch- RAW


//------------------------------------ ARRAY VISTA

byte array_vista[461];
int i = 1;

int timer_for = 0;


int secondi_test;
int decimi_test;
int centesimi_test;
int loading_scansione = 0;
int index = 0;


//____________________________________________SETTING INIZIALE_________________________________________
void setup() 
{
  
    //Memorizzazione INIZIALE attuale posizione zampe 
    pos_braccio2_fin = pos_braccio2;
    pos_avambraccio2_fin = pos_avambraccio2;
  
  for(i = 1; i <= 461; i = i + 1)
  {
   array_vista[i] = 0;
  }
  
  i = 1; //pronta per dopo!
  
  
  /*
    if(timer_for == 0)
    {
        for(i = 1; i < 121; i = i + 1)
        {
          //Serial.println(myPins[i]);
          array_vista[i] = -1;
           
        }
         timer_for = 1;  
    }    
  */
  
  //wdt_enable(WDTO_2S); WATCHDOG
  //-------------------------- GYRO
     // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 12; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(200, true); //o 400????
    #endif
    Serial.begin(115200); //115200
    //Serial.begin(57600);    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    mpu.initialize();
    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); //1788 // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {

    }
    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);

    
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates


//30 170    ---  10 150 (se invertito)
//DEVI CONVERTIRE PER OTTENERE DI POTER ORDINARE MOVIMENTI DA 0 a 140

      //************************************************************************************** ZAMPA 1 SETUP 
      //SETTAGGIO INIZIALE AVAMBRACCIO - 0
      //pos_avambraccio1 = pos_avambraccio1 + 10; //+10 perche' e'in mirror se no sarebbe +30
      new_pos_avambraccio1 = 180 - pos_avambraccio1;
      pulselengthz = map(new_pos_avambraccio1, 0, 180, SERVOMIN_ava1, SERVOMAX_ava1);
      pwm.setPWM(0, 0, pulselengthz); 
      
      //SETTAGGIO INIZIALE BRACCIO - 1
      //pos_braccio1 = pos_braccio1 + 10; //+10 perche' e'in mirror se no sarebbe +30
      //new_pos_braccio1 = 180 - pos_braccio1;
      pulselengthzb = map(pos_braccio1, 0, 180, SERVOMIN_bra1, SERVOMAX_bra1);
      pwm.setPWM(1, 0, pulselengthzb); 
           
      //SETTAGGIO INIZIALE SPALLA - 2
      /*
      new_pos_spalla1 = 180 - pos_spalla1;
      pulselengthzc = map(new_pos_spalla1, 0, 180, SERVOMIN_spa1, SERVOMAX_spa1);
      pwm.setPWM(2, 0, pulselengthzc);   
      */ 
      //SETTAGGIO INIZIALE SPALLA - 2
      new_pos_spalla1 = 180 - pos_spalla1;
      pulselengthzc = map(new_pos_spalla1, 0, 180, SERVOMIN_spa1, SERVOMAX_spa1);
      pwm.setPWM(2, 0, pulselengthzc);   


            
      //************************************************************************************** ZAMPA 2 SETUP
      //SETTAGGIO INIZIALE AVAMBRACCIO - 4
      //pos_avambraccio2 = pos_avambraccio2 + 30; //se fosse in mirror sarebbe +10
      new_pos_avambraccio2 = 180 - pos_avambraccio2;      
      pulselengthz = map(new_pos_avambraccio2, 0, 180, SERVOMIN_ava2, SERVOMAX_ava2);
      pwm.setPWM(4, 0, pulselengthz); 
     
      //SETTAGGIO INIZIALE BRACCIO - 6
      //pos_braccio2 = pos_braccio2 + 10; //se fosse in mirror sarebbe +10  
      new_pos_braccio2 = 180 - pos_braccio2;
      pulselengthzb = map(new_pos_braccio2, 0, 180, SERVOMIN_bra2, SERVOMAX_bra2); 
      pwm.setPWM(6, 0, pulselengthzb); 


      
      //SETTAGGIO INIZIALE SPALLA - 7
      new_pos_spalla2 = 180 - pos_spalla2;
      pulselengthzc = map(new_pos_spalla2, 0, 180, SERVOMIN_spa2, SERVOMAX_spa2);
      pwm.setPWM(7, 0, pulselengthzc);   





      
      
      //************************************************************************************** ZAMPA 3 SETUP
      //SETTAGGIO INIZIALE AVAMBRACCIO - 8
      pos_avambraccio3 = pos_avambraccio3 + 0;// e' mirror quindi + 10
      new_pos_avambraccio3 = 180 - pos_avambraccio3;
      pulselengthz = map(new_pos_avambraccio3, 0, 180, SERVOMIN_ava3, SERVOMAX_ava3);
      pwm.setPWM(8, 0, pulselengthz); 
   
      //SETTAGGIO INIZIALE BRACCIO - 9
      //pos_braccio3 = pos_braccio3 + 30;//non e' mirror quindi + 30
      //new_pos_braccio3 = 190 - pos_braccio3;
      pulselengthzb = map(pos_braccio3, 0, 180, SERVOMIN_bra3, SERVOMAX_bra3);
      pwm.setPWM(9, 0, pulselengthzb);  

     
      //SETTAGGIO INIZIALE SPALLA - 10
      new_pos_spalla3 = 180 - pos_spalla3;
      pulselengthzc = map(new_pos_spalla3, 0, 180, SERVOMIN_spa3, SERVOMAX_spa3);
      pwm.setPWM(10, 0, pulselengthzc);     
 
    
     
      //************************************************************************************** ZAMPA 4 SETUP
  //SETTAGGIO INIZIALE AVAMBRACCIO - 12
  //pos_avambraccio4 = pos_avambraccio4 + 10;//e' mirror quindi + 10
  //new_pos_avambraccio4 = 180 - pos_avambraccio4;
  new_pos_avambraccio4 = pos_avambraccio4;
  //new_pos_avambraccio4 = pos_avambraccio4;  
  pulselengthz = map(new_pos_avambraccio4, 0, 180, SERVOMIN_ava4, SERVOMAX_ava4);
  pwm.setPWM(12, 0, pulselengthz); 
    
      //SETTAGGIO INIZIALE BRACCIO - 13
      //pos_braccio4 = pos_braccio4 + 30;//non e' mirror quindi + 30
      new_pos_braccio4 = 180 - pos_braccio4;      
      pulselengthzb = map(new_pos_braccio4, 0, 180, SERVOMIN_bra4, SERVOMAX_bra4);
      pwm.setPWM(13, 0, pulselengthzb);  
     
      //SETTAGGIO INIZIALE SPALLA - 15
      //new_pos_spalla4 =  180 - pos_spalla4;
      new_pos_spalla4 = pos_spalla4;      
      pulselengthzc = map(new_pos_spalla4, 0, 180, SERVOMIN_spa4, SERVOMAX_spa4);
      pwm.setPWM(15, 0, pulselengthzc);   



    
}
//______________________________________________________________________________________________________

     int flag_memorizz_coord_bra1, flag_memorizz_coord_bra2, flag_memorizz_coord_bra3, flag_memorizz_coord_bra4;

     int flag_memorizz_coord_avambra1, flag_memorizz_coord_avambra2, flag_memorizz_coord_avambra3, flag_memorizz_coord_avambra4;

     int flag_memorizz_coord_zampa = 0;    
  
     int flag_fine_raddrizzamento1 = 0, flag_fine_raddrizzamento2 = 0, flag_fine_raddrizzamento3 = 0, flag_fine_raddrizzamento4 = 0;      



// VARIABILI ALTEZZA ROBOT
int routine_altezza = 0;
int step_altezza = 5; //di quanto deve diminuire altezza robot
int flag_altezza = 0, flag_altezzaB = 0; //setup
int fine_altezz_zampa1 = 0, fine_altezz_zampa2 = 0, fine_altezz_zampa3 = 0, fine_altezz_zampa4 = 0;
int alt_posizione_bra1 , alt_posizione_bra2, alt_posizione_bra3, alt_posizione_bra4;
int alt_posizione_avambra1, alt_posizione_avambra2, alt_posizione_avambra3, alt_posizione_avambra4;
int flag_altezza_serial = 0;
int pos_avambraccio1_alt_targ, pos_avambraccio2_alt_targ, pos_avambraccio3_alt_targ, pos_avambraccio4_alt_targ;
int modo_altezza1 = 0, modo_altezza2 = 0, modo_altezza3 = 0, modo_altezza4 = 0;
int timer_generico_equilibrio = 0;
int flag_generico1 = 0;

int pausa_reflex = 1; 
int speed_reflex = 1; //per il braccio sara' 1 mentre per l'avambr sara' speed_reflex * 2 

int cilla = 0;
int equilibrio_off = 0;

int crono_spinte = 0;
int lentezza_spinte = 1;
int no_mov_zampa1 = 0, no_mov_zampa2 = 0;
int altezza_robot_totale = 0;
int altezza_robot_zampa1 = 0, altezza_robot_zampa2 = 0, altezza_robot_zampa3 = 0, altezza_robot_zampa4 = 0;
int timerk = 0;
int flag_pippo = 0;
int flag_incomingByte = 1;
int flag_equilib_zampe = 0;

//____________________________________________LOOP PRINCIPALE___________________________________________
void loop() 
{


   
 //ram_rimasta = freeRam();   


  Serial.print(angolo_rotta_attuale);  // rotta attuale
  Serial.print(",");                
  Serial.print(angolo_rotta_target);  // rotta target
  Serial.print(",");               
  Serial.print(risposta_direzione);  // risposta direzione
  Serial.print(",");
  Serial.print(movimento_on,DEC); //movimento on
  Serial.print(",");
  Serial.print(angolo_roll_attuale);  // x roll
  Serial.print(",");  
  Serial.print(angolo_pitch_attuale);  // y pitch
  Serial.print(","); 
  Serial.print(fase_mov_zampe,DEC);  // fase_mov_zampe ---> fase movimento furtivo 
  Serial.print(",");
  Serial.print(equilibrio_str);  // equilibrio_str
  Serial.print(",");     
  Serial.print(target_x);  // target_x
  Serial.print(",");     
  Serial.print(target_y);  // target_y
  Serial.print(","); 
  
  Serial.print(velocita_mov);  // 
  Serial.print(","); 

  Serial.print(flag_kfin);  // 
  Serial.print(",");  
 
  Serial.print(valore_vel_sin2_FIN);  // 
  Serial.print(",");  
 
  Serial.print(valore_vel_sin4_FIN);  // 
  Serial.print(",");  
 
  Serial.print(valore_vel_des1_FIN);  // 
  Serial.print(",");

  Serial.print(valore_vel_des3_FIN);  // 
  Serial.print(",");
  //----------------
  Serial.print(valore_tar_sin2_SOMMA);  // era valore_tar_sin2_FIN = valore target per esempio 105 (posizione da raggiungere)
  Serial.print(",");    
  Serial.print(valore_tar_sin4_SOMMA);  // 
  Serial.print(",");   
  Serial.print(valore_tar_des1_SOMMA);  // 
  Serial.print(",");   
  Serial.print(valore_tar_des3_SOMMA);  // 
  Serial.print(","); 

  Serial.print(pos_braccio1_fin);  // 
  Serial.print(",");

  Serial.print(pos_avambraccio1_fin);  // 
  Serial.print(",");

  Serial.print(valore_tar_des1_FIN);  // 
  Serial.print(",");
  
  Serial.print(fsrReading1);  // 
  Serial.print(",");

  Serial.print(fsrReading2);  // 
  Serial.print(",");

  Serial.print(fsrReading3);  // 
  Serial.print(",");

  Serial.print(fsrReading4);  // 
  Serial.print(",");
 
 
 
 /*
  Serial.print(fase_equilibrio);  // 
  Serial.print(","); 
 
  Serial.print(timer_target_x_raggiunto);  // 
  Serial.print(",");  
 
  Serial.print(angolo_roll_attuale);  // 
  Serial.print(",");  
 */
  Serial.print(velocita_mov);  // 
  Serial.print(",");  
  Serial.print(valore_tar_sin2);  // 
  Serial.print(",");  
  Serial.print(dist_angolo_roll_con_target_x);  // 
  Serial.print(",");  
  Serial.print(inclinazione_a);  // 
  Serial.print(",");  
  
  Serial.print(fase_equilibrio);  // 
  Serial.print(",");  
  Serial.print(timer_check_assi);  // 
  Serial.print(","); 
  Serial.print(zona_velocita);  // 
  Serial.print(",");   
  //Serial.print(ok_zampa2);  // 
  //Serial.print(",");
  
  Serial.print(array_vista[0]);
  Serial.print(",");
  Serial.print(array_vista[1]);  
  Serial.print(",");
  Serial.print(array_vista[2]);
  Serial.print(",");
  Serial.print(array_vista[3]);
  Serial.print(",");  
  Serial.print(array_vista[4]);
  Serial.print(",");
  Serial.print(array_vista[5]);
  Serial.print(",");
  Serial.print(array_vista[6]);
  Serial.print(",");
  Serial.print(array_vista[7]);
  Serial.print(",");
  Serial.print(array_vista[8]);
  Serial.print(",");
  Serial.print(array_vista[9]);
  Serial.print(",");  

  Serial.print(array_vista[10]);
  Serial.print(",");
  Serial.print(array_vista[11]);  
  Serial.print(",");
  Serial.print(array_vista[12]);
  Serial.print(",");
  Serial.print(array_vista[13]);
  Serial.print(",");  
  Serial.print(array_vista[14]);
  Serial.print(",");
  Serial.print(array_vista[15]);
  Serial.print(",");
  Serial.print(array_vista[16]);
  Serial.print(",");
  Serial.print(array_vista[17]);
  Serial.print(",");
  Serial.print(array_vista[18]);
  Serial.print(",");
  Serial.print(array_vista[19]); //54a variabile spedita
  Serial.print(",");  
Serial.print(ok_zampa3);
Serial.print(",");  
Serial.print(pos_avambraccio1_fin);
Serial.print(",");  
Serial.print(pos_braccio3_fin); 
Serial.print(",");  
Serial.print(pos_avambraccio3_fin); 
Serial.print(",");  

Serial.print(pos_braccio2_fin);
Serial.print(",");  
Serial.print(pos_avambraccio2_fin);
Serial.print(","); 
Serial.print(pos_braccio4_fin);
Serial.print(","); 
Serial.print(pos_avambraccio4_fin);
Serial.print(","); 


//Serial.print(",");   
  Serial.println();
 //Serial.print(ram_rimasta);   



  

   //---------------------------------------------------------------------------------- GYRO ENGINE
     // if programming failed, don't try to do anything

    if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

      
      // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) //1024    
    //if ((mpuIntStatus & 0x10) || fifoCount == 1024) //1024
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
       
        //Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) 
    {      
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);         
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;  
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
             z = ypr[0] * 180/M_PI; //yawn 
             x = ypr[1] * 270/M_PI; //roll SE SI RIDUCE IN 90/M_PI, I GRADI AUMENTANO MENO DURANTE INCLINAZIONE
             y = ypr[2] * 270/M_PI; //pitch SE SI RIDUCE IN 90M_PI, I GRADI AUMENTANO MENO DURANTE INCLINAZIONE 
//--------------------------------------------- valore asse Y - PITCH
             //angolo_pitch_reale = y;
             angolo_pitch_attuale = y;
//--------------------------------------------- valore asse X - LATERAL ROLL
             //angolo_roll_reale = x;
             angolo_roll_attuale = x;
//--------------------------------------------- valore asse Z - YAWN BUSSOLA (trasformato in 360)
             if (z <= 0 && z >= -179)
             {
              angolo_rotta_attuale = (z + 361) % 360;
             }
             if (z >= 0 && z <= 179)
             {
              angolo_rotta_attuale = (z + 360) % 360;
             }             

        #endif
}//fine else if (mpuIntStatus & 0x02) { 

       

  //------------------------------------------------- COMANDI DA TASTIERA E SERIALI DA PROCESSING
  if (Serial.available() > 0) 
  {
   incomingByte = Serial.read();
   if (incomingByte == 'p') 
   {
     if (pausa == 0)
     {
      pausa = 1;
     } 
     else
     {
      pausa = 0;     
     }
   }  

   if (incomingByte == 'q') 
   {
    flag_incomingByte = 0; 
   } //fine if (incomingByte == 'q')


        if (flag_incomingByte == 0)
        {             
              if (movimento_on == 0 || movimento_on == -1)
              {
               movimento_on = 1;
               movimento_on_flag = 0;
               flag_incomingByte = 1;
              }
        }// fine if (flag_incomingByte == 0)

        if (flag_incomingByte == 0)
        {         
              if (movimento_on == 1)
              {
               movimento_on = -1; 
               movimento_on_flag = 0;
               flag_incomingByte == 1;               
              }  
        }

   
  
   
   
   
   
   if (incomingByte == 'w')  
   {
    reset_saltelli = 1; 
    movimento_on = 0; // trotto sul posto
    movimento_on_flag = 0;
   }      
   if (incomingByte == 'e') 
   {
    movimento_on = -1; // stop movimenti
    movimento_on_flag = 0;
   }     
   
   
   if (incomingByte == '+') 
   {
     angolo_rotta_target = angolo_rotta_target +5;     
     fase = 0;    
   }
   if (incomingByte == '-') 
   {
     angolo_rotta_target = angolo_rotta_target -5; 
     fase = 0;    
   }   
   
   if (incomingByte == 'z') {
     target_x = target_x +1;}   
   if (incomingByte == 'x') {
     target_x = target_x -1;}      
   
   if (incomingByte == 'c') {
     target_y = target_y +1;}   
   if (incomingByte == 'v') {
     target_y = target_y -1;}    


 //-----------------------------------------------------
   //****************** ROBOT AUMENTO ALTEZZA
   if (incomingByte == 'o') //DEVI ESSERE IN MODALITA' 1 CAMMINATA SE NO NON FUNGE
   {     
     pos_avambraccio1_alt_targ = pos_avambraccio1_alt_targ + 5;
     //flag_altezza_serial = 1;  
   }// fine if (incomingByte == 'o')    
   
   //****************** ROBOT DIMINUZIONE ALTEZZA
   if (incomingByte == 'i') //DEVI ESSERE IN MODALITA' 1 CAMMINATA SE NO NON FUNGE
   {     
     pos_avambraccio1_alt_targ = pos_avambraccio1_alt_targ - 5;
     //flag_altezza_serial = 2;  
   }// fine if (incomingByte == 'o')     
    
   //****************** SET TARGET ALTEZZA ROBOT
   if (incomingByte == 'k') //DEVI ESSERE IN MODALITA' 1 CAMMINATA SE NO NON FUNGE
   {     
     flag_altezza_serial = 1;  
   }// fine if (incomingByte == 'k')    
    
 //-----------------------------------------------------

 
   
  }//fine if (Serial.available() > 0) 
  
 

  


  
  
  
 //if (pausa != 1)
 //{


  //------------------------------ SENSORI TATTILI -----------------------------------
  // RALLENTA DI BRUTTO! 
    fsrReading1 = analogRead(fsrAnalogPin1);
    fsrReading2 = analogRead(fsrAnalogPin2);
    fsrReading3 = analogRead(fsrAnalogPin3); 
    fsrReading4 = analogRead(fsrAnalogPin4);    
  //-----------------------------------------------------------------------------------

















































if (start == 10)
{ 
     if (pillozzo == 0)//RICORDA CHE IL CODICE EQUILIBRIO E'DISABILITATO EH
     {
      ok_zampa3 = 0; 
      moto_zampa3 = 1;

      ok_zampa4 = 0; 
      moto_zampa4 = 1;
      
      pillozzo = 1;      
     }
   


 if (movimento_on == 1)
 { 


  if(flag_altezza_serial == 1)
  {
    /*
    if(pillozzo == 0)
    {
     ok_zampa1 = 0;  
     moto_zampa1 = 1;
     pillozzo = 1;
     //zampa1_no_equilibrio = 1;
    }
    */
   ricarica_zampa3(); 
   ricarica_zampa4();
  }






   
   /*
   movimento_zampa_new2(1,2,120); //avambraccio
   movimento_zampa_new2(2,1,70); //braccio 
   
   movimento_zampa_new4(1,2,110); //avambraccio
   movimento_zampa_new4(2,1,75); //braccio
   */
   
  //timerk = timerk + 1;  
  //if (timerk == 5)
  //{ 
   //ricarica_zampa4();
   //timerk = 0;
  //}
  
  
   //lentezza_spinte = 3;
   //spinta_zampe_codice();
  
   
   //proviamo a modificare la massima inclinazione del robot in fase di equilibrtio a 10 avambr e 20 braccio
 
 /*  
   timer_generico = timer_generico + 1;
   
      //movimento_zampa_new2(2, 1, 20);//braccio      
   
   if(timer_generico <= 20)
   {       
      movimento_zampa_new1(1, 1, 130);//avambraccio 
      movimento_zampa_new1(2, 1, 20);//braccio    

      movimento_zampa_new2(1, 1, 130);//avambraccio 
      movimento_zampa_new2(2, 1, 20);//braccio       
     
      movimento_zampa_new3(1, 1, 130);//avambraccio 
      movimento_zampa_new3(2, 1, 20);//braccio        
      
      movimento_zampa_new4(1, 1, 130);//avambraccio 
      movimento_zampa_new4(2, 1, 20);//braccio          
      
   }      

   if (timer_generico > 20 && timer_generico <= 30) //se dopo che l'avambr si e' fermato, il tasto flag_altezza_serial, l'avambr riprende fino a nuova destinazione
   {  
      movimento_zampa_new1(1, 5, 60);//avambraccio
      movimento_zampa_new1(2, 3, 130);//braccio    
     
      movimento_zampa_new2(1, 5, 60);//avambraccio
      movimento_zampa_new2(2, 3, 130);//braccio 
      
      movimento_zampa_new3(1, 5, 60);//avambraccio
      movimento_zampa_new3(2, 3, 130);//braccio  
 
      movimento_zampa_new4(1, 5, 60);//avambraccio
      movimento_zampa_new4(2, 3, 130);//braccio      
   }  
   if (timer_generico > 30)
   {
    movimento_zampa_new1(1, 1, 130);//avambraccio
    movimento_zampa_new1(2, 5, 30);//braccio      
     
    movimento_zampa_new2(1, 1, 130);//avambraccio
    movimento_zampa_new2(2, 5, 30);//braccio 
    
    movimento_zampa_new3(1, 1, 130);//avambraccio
    movimento_zampa_new3(2, 5, 30);//braccio  
 
    movimento_zampa_new4(1, 1, 130);//avambraccio
    movimento_zampa_new4(2, 5, 30);//braccio    
   }   
   
   */
   
   
  
 
      // QUESTO E' L'ULTIMO CODICE GESTIONE SERVI! 8/1/2017
      //************************************************************************************** ZAMPA 1 SETUP 
      //SETTAGGIO INIZIALE AVAMBRACCIO - 0
      new_pos_avambraccio1 = 180 - pos_avambraccio1_fin;
      pulselengthz = map(new_pos_avambraccio1, 0, 180, SERVOMIN_ava1, SERVOMAX_ava1);
      pwm.setPWM(0, 0, pulselengthz); 
      
      //SETTAGGIO INIZIALE BRACCIO - 1
      pulselengthzb = map(pos_braccio1_fin, 0, 180, SERVOMIN_bra1, SERVOMAX_bra1);
      pwm.setPWM(1, 0, pulselengthzb); 
           
      //SETTAGGIO INIZIALE SPALLA - 2
      new_pos_spalla1 = 180 - pos_spalla1_fin;
      pulselengthzc = map(new_pos_spalla1, 0, 180, SERVOMIN_spa1, SERVOMAX_spa1);
      pwm.setPWM(2, 0, pulselengthzc);  
   
      //************************************************************************************** ZAMPA 2 SETUP ---------- Con variabili fin!
      //SETTAGGIO INIZIALE AVAMBRACCIO - 4
      new_pos_avambraccio2 = 180 - pos_avambraccio2_fin;      
      pulselengthz = map(new_pos_avambraccio2, 0, 180, SERVOMIN_ava2, SERVOMAX_ava2);
      pwm.setPWM(4, 0, pulselengthz); 
     
      //SETTAGGIO INIZIALE BRACCIO - 6 
      new_pos_braccio2 = 180 - pos_braccio2_fin;
      pulselengthzb = map(new_pos_braccio2, 0, 180, SERVOMIN_bra2, SERVOMAX_bra2); 
      pwm.setPWM(6, 0, pulselengthzb); 

      //SETTAGGIO INIZIALE SPALLA - 7
      new_pos_spalla2 = 180 - pos_spalla2_fin;
      pulselengthzc = map(new_pos_spalla2, 0, 180, SERVOMIN_spa2, SERVOMAX_spa2);
      pwm.setPWM(7, 0, pulselengthzc);         

 
      //************************************************************************************** ZAMPA 3 SETUP
      //SETTAGGIO INIZIALE AVAMBRACCIO - 8
      pos_avambraccio3_fin = pos_avambraccio3_fin + 0;// e' mirror quindi + 10
      new_pos_avambraccio3 = 180 - pos_avambraccio3_fin;
      pulselengthz = map(new_pos_avambraccio3, 0, 180, SERVOMIN_ava3, SERVOMAX_ava3);
      pwm.setPWM(8, 0, pulselengthz); 
   
      //SETTAGGIO INIZIALE BRACCIO - 9
      pulselengthzb = map(pos_braccio3_fin, 0, 180, SERVOMIN_bra3, SERVOMAX_bra3);
      pwm.setPWM(9, 0, pulselengthzb);  

      //SETTAGGIO INIZIALE SPALLA - 10
      //new_pos_spalla3 = 180 - pos_spalla3_fin;
      //pulselengthzc = map(new_pos_spalla3, 0, 180, SERVOMIN_spa3, SERVOMAX_spa3);
      //pwm.setPWM(10, 0, pulselengthzc);      
      
 
      //************************************************************************************** ZAMPA 4 SETUP
      //SETTAGGIO INIZIALE AVAMBRACCIO - 12
      new_pos_avambraccio4 = pos_avambraccio4_fin; 
      pulselengthz = map(new_pos_avambraccio4, 0, 180, SERVOMIN_ava4, SERVOMAX_ava4);
      pwm.setPWM(12, 0, pulselengthz); 
    
      //SETTAGGIO INIZIALE BRACCIO - 13
      new_pos_braccio4 = 180 - pos_braccio4_fin;      
      pulselengthzb = map(new_pos_braccio4, 0, 180, SERVOMIN_bra4, SERVOMAX_bra4);
      pwm.setPWM(13, 0, pulselengthzb);  
     
      //SETTAGGIO INIZIALE SPALLA - 15
      //new_pos_spalla4 = pos_spalla4_fin;      
      //pulselengthzc = map(new_pos_spalla4, 0, 180, SERVOMIN_spa4, SERVOMAX_spa4);
      //pwm.setPWM(15, 0, pulselengthzc);  
 
      

 }//fine if (movimento_on == 1)
}// fine start == 10













 ///////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////// 
 ///////////////////////////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////// CAMMINATA TEST FURTIVA /////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////// 
 ///////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////// 
   //---------------------------- ROUTINE EQUILIBRIO GIROSCOPIO ---------------------
   //jizzy si inclina a destra   ---> x negativo
   //jizzy si inclina a sinistra ---> x positivo

if (start == 7)
{ 
  
  secondi = millis() / 1000; 
  if (secondi < 22) //DEVONO ESSERE 22 PER PERMETTERE AL GYRO DI CALIBRARSI!
  {
   risposta_direzione = "CALIBRAZIONE";
   //Serial.println(F("*************   CALIBRAZIONE   GYRO    ***************"));  
   camminata_test = 1;
  } 
  else
  {
   secondi = 0; 
   risposta_direzione = "GYRO OK";


if(movimento_on == 1)
{
   //PULSANTE RIDUZIONE ALTEZZA ZAMPA  (deve stare qui perche' il codice di movimento dei servi e' dentro movimento_on = 1     
   //if (flag_altezza_serial == 1)
   //{    
   // step_altezza = 15;              
   // riduzione_altezza_zampe(); 
   //} 


//*********************************************************************************************
//*********************************************************************************************
//**********************************                         **********************************
//*********************************  ROUTINE EQUILIBRIO GYRO  *********************************
//**********************************                         **********************************
//*********************************************************************************************
//*********************************************************************************************

  //LENTA
  if (angolo_roll_attuale <= target_x+4 || angolo_roll_attuale >= target_x-4                                   || angolo_pitch_attuale <= target_y+4 || angolo_pitch_attuale >= target_y-4) 
  {
   zona_velocita = 1; //lenta
   //fase_equilibrio_B == 0; //quando siamo in zona_velocita' lenta, la fase dell'altra zona viene resettata!
   targ_crono_vel_equilib = 3;
  }
  
  //MEDIA
  
    
  //VELOCE
  if (angolo_roll_attuale > target_x+4 || angolo_roll_attuale < target_x-4                                     || angolo_pitch_attuale > target_y+4 || angolo_pitch_attuale < target_y-4)
  {
   zona_velocita = 2; //2; //veloce
   //fase_equilibrio = 0; //quando siamo in zona_velocita' veloce, la fase dell'altra zona viene resettata!
   targ_crono_vel_equilib = 1;
  }



//************************************************************************************************************************************
//************************************************************************************************************************************
//************************************ ESECUZIONE EQUILIBRIO *************************************************************************
//************************************************************************************************************************************
//************************************************************************************************************************************
if(spinta_zampe == 0)
{
    //----------------------------------------------------------------------    
    //----------------------------------------------------- ASSE X
    //----------------------------------------------------------------------       
         //ROBOT DRITTO
         if (angolo_roll_attuale == target_x)
         {          
          //fermi 2 e 4          
          valore_tar_sin2 = 0; 
          valore_tar_sin4 = 0;
          //fermi 1 e 3  
          valore_tar_des1 = 0; 
          valore_tar_des3 = 0;          
         }      
  
         //INCLINAZIONE A SINISTRA
         if (angolo_roll_attuale > target_x)
         {  
                     
          //Apertura 2 e 4  
          valore_tar_sin2 = 1; 
          valore_tar_sin4 = 1;
          //Chiusura 1 e 3  
          valore_tar_des1 = -1; 
          valore_tar_des3 = -1;  
          //fase_equilibrio = 1;         
         }
       
         //INCLINAZIONE A DESTRA
         if (angolo_roll_attuale < target_x)
         {
          //Apertura 2 e 4  
          valore_tar_sin2 = -1; 
          valore_tar_sin4 = -1;
          //Chiusura 1 e 3  
          valore_tar_des1 = 1; 
          valore_tar_des3 = 1;  
          //fase_equilibrio = 1;          
         }   
            
    //----------------------------------------------------------------------    
    //----------------------------------------------------- ASSE Y
    //----------------------------------------------------------------------          
         //ROBOT DRITTO
         if (angolo_pitch_attuale == target_y)
         {
          //fermi 1 e 2           
          valore_tar_des1b = 0; 
          valore_tar_sin2b = 0;           
          //fermi 3 e 4        
          valore_tar_des3b = 0; 
          valore_tar_sin4b = 0;         
         }      
         
         //INCLINAZIONE CABRATA
         if (angolo_pitch_attuale > target_y)
         {  
          // 1 e 2 chiudono          
          valore_tar_des1b = -1; 
          valore_tar_sin2b = -1;           
          //3 e 4 aprono       
          valore_tar_des3b = 1; 
          valore_tar_sin4b = 1;            
         }         
        
         //INCLINAZIONE PICCHIATA
         if (angolo_pitch_attuale < target_y)
         {  
          // 1 e 2 aprono          
          valore_tar_des1b = 1; 
          valore_tar_sin2b = 1;           
          //3 e 4 chiudono       
          valore_tar_des3b = -1; 
          valore_tar_sin4b = -1;            
         }                 
 
 
//-------------------------


        //SOMMIAMO I VALORI TAR IN AMBITO X con QUELLI IN AMBITO Y
          valore_tar_sin2_SOMMA = valore_tar_sin2 + valore_tar_sin2b;
          valore_tar_sin4_SOMMA = valore_tar_sin4 + valore_tar_sin4b;
          valore_tar_des1_SOMMA = valore_tar_des1 + valore_tar_des1b;
          valore_tar_des3_SOMMA = valore_tar_des3 + valore_tar_des3b;
          
          
          if (valore_tar_sin2_SOMMA > 1)
          {
            valore_tar_sin2_SOMMA = 1;
          }
          if (valore_tar_sin2_SOMMA < -1)
          {
            valore_tar_sin2_SOMMA = -1;
          }
          //-----
          if (valore_tar_sin4_SOMMA > 1)
          {
            valore_tar_sin4_SOMMA = 1;
          }
          if (valore_tar_sin4_SOMMA < -1)
          {
            valore_tar_sin4_SOMMA = -1;
          }
          //-----
          if (valore_tar_des1_SOMMA > 1)
          {
            valore_tar_des1_SOMMA = 1;
          }
          if (valore_tar_des1_SOMMA < -1)
          {
            valore_tar_des1_SOMMA = -1;
          }
          //-----
          if (valore_tar_des3_SOMMA > 1)
          {
            valore_tar_des3_SOMMA = 1;
          }
          if (valore_tar_des3_SOMMA < -1)
          {
            valore_tar_des3_SOMMA = -1;
          }
         
 
/* 
 --------- riprova aggiungendo questo pezzo
   ///////////////////////////////////////////////////////////////////////////////////////
  //  FASE 1 - SET TAR FINALE
  //                   
  ///////////////////////////////////////////////////////////////////////////////////////
  
      //POSIZIONE DA RAGGIUNGERE PER OGNI AVAMBRACCIO, i valori "valore_tar_SOMMA" possono essere 1 o -1 quindi
      //                                               l'avambraccio si chiude o si apre 1 pixel alla volta
      valore_tar_sin2_FIN = pos_avambraccio2_fin + valore_tar_sin2_SOMMA;
      valore_tar_sin4_FIN = pos_avambraccio4_fin + valore_tar_sin4_SOMMA;
      valore_tar_des1_FIN = pos_avambraccio1_fin + valore_tar_des1_SOMMA;
      valore_tar_des3_FIN = pos_avambraccio3_fin + valore_tar_des3_SOMMA;
      
      //POSIZIONE MASSIMA RAGGIUNGIBILE DAGLI AVAMBRACCI
      //Zampe 2 e 4 avambraccia
      if(valore_tar_sin2_FIN > 120){valore_tar_sin2_FIN = 120;} 
      if(valore_tar_sin2_FIN < 20){valore_tar_sin2_FIN = 20;}  
      if(valore_tar_sin4_FIN > 110){valore_tar_sin4_FIN = 110;}  
      if(valore_tar_sin4_FIN < 30){valore_tar_sin4_FIN = 30;} 

      //Zampe 1 e 3 avambraccia        
      if(valore_tar_des1_FIN > 120){valore_tar_des1_FIN = 120;}  
      if(valore_tar_des1_FIN < 20){valore_tar_des1_FIN = 20;}            
      if(valore_tar_des3_FIN > 110){valore_tar_des3_FIN = 110;}  
      if(valore_tar_des3_FIN < 30){valore_tar_des3_FIN = 30;} 
-------
*/  
 
  ///////////////////////////////////////////////////////////////////////////////////////
  //  FASE 2 - MOVIMENTO ZAMPE
  //                   
  ///////////////////////////////////////////////////////////////////////////////////////
  //movimento_zampa_new1(int servo_num, int st_speed, int target)
  //                         1 = avambr      2
  //                         2 = bra         1
  //                         3 = spa



//se mi ritrovo nel counter ma parte la velocita' fast, eseguo senza aspettare piu' il counter!
timer_generico3 = timer_generico3 + 1;
if(timer_generico3 == targ_crono_vel_equilib || zona_velocita == 2)
{
     //--------------------------------------------------------- ZAMPA 2
     if(zampa2_no_equilibrio == 0)
     {    
        //ZAMPA 2       
        if(valore_tar_sin2_SOMMA > 0) //la zampa si apre
        {
          movimento_zampa_new2(1,2,120); //avambr
          if (pos_braccio2_fin >= 70)
          {
           movimento_zampa_new2(2,1,70); //bra 
          }          
        }
        
        if(valore_tar_sin2_SOMMA < 0) // la zampa si chiude
        {
          if(fsrReading2 > 250)
          {
              movimento_zampa_new2(1,2,20); //avambr
              if (pos_braccio2_fin >= 70)
              {          
               movimento_zampa_new2(2,1,130); //bra 
              }  
          }  
          else
          {
            /*
             //----------- LA ZAMPA 2 SI RIAPRE IN EMERGENZA PER RIATTACCARSI AL SUOLO!
             movimento_zampa_new2(1,2,120); //avambr
             if (pos_braccio2_fin >= 70)
             {
              movimento_zampa_new2(2,1,70); //bra 
             }   
            */ 
          }


        }
     }


     //--------------------------------------------------------- ZAMPA 4
     if(zampa4_no_equilibrio == 0)
     {      
        //ZAMPA 4
        if(valore_tar_sin4_SOMMA > 0) //zampa si apre
        {
          movimento_zampa_new4(1,2,110); //avambr
          if (pos_braccio4_fin < 80)
          {  
           movimento_zampa_new4(2,1,75); //bra
          } 
      
        } // fine if(valore_tar_sin4_SOMMA > 0)
        
        if(valore_tar_sin4_SOMMA < 0) //zampa si chiude
        {
          
         if(fsrReading4 > 250)  
         { 
              movimento_zampa_new4(1,2,10); //avambr 
              if (pos_braccio4_fin < 80)
              {            
               movimento_zampa_new4(2,1,20); //bra       
              }
         }
         else
         {
           /*
             //----------- LA ZAMPA 4 SI RIAPRE IN EMERGENZA PER RIATTACCARSI AL SUOLO!
             movimento_zampa_new4(1,2,110); //avambr
             if (pos_braccio4_fin < 80)
             {  
              movimento_zampa_new4(2,1,75); //bra
             } 
           */ 
         }         
          
        } //fine if(valore_tar_sin4_SOMMA < 0) 
     }
     
     
     //--------------------------------------------------------- ZAMPA 1
     if(zampa1_no_equilibrio == 0)
     {    
        //ZAMPA 1       
        if(valore_tar_des1_SOMMA > 0)
        {
          movimento_zampa_new1(1,2,120); //avambr
          if (pos_braccio1_fin >= 70)
          {
           movimento_zampa_new1(2,1,70); //bra 
          } 
       
        }
        
        if(valore_tar_des1_SOMMA < 0)
        {
          
          if(fsrReading1 > 250)
          {
              movimento_zampa_new1(1,2,20); //avambr
              if (pos_braccio1_fin >= 70)
              {          
               movimento_zampa_new1(2,1,130); //bra 
              }
          } 
          else
          {
            /*
             //----------- LA ZAMPA 1 SI RIAPRE IN EMERGENZA PER RIATTACCARSI AL SUOLO!
             movimento_zampa_new1(1,2,120); //avambr
             if (pos_braccio1_fin >= 70)
             {
              movimento_zampa_new1(2,1,70); //bra 
             }      
            */      
          }          
                          
        }
     }


     //--------------------------------------------------------- ZAMPA 3    
     if(zampa3_no_equilibrio == 0)
     {      
        //ZAMPA 3
        if(valore_tar_des3_SOMMA > 0) //zampa si apre
        {
          movimento_zampa_new3(1,2,110); //avambr          
          if (pos_braccio3_fin < 80)
          {            
           movimento_zampa_new3(2,1,75); //bra  
          }   
          
        }//fine if(valore_tar_des3_SOMMA > 0)
        
        if(valore_tar_des3_SOMMA < 0) //zampa si chiude
        {
          
          if(fsrReading3 > 250)
          {
              movimento_zampa_new3(1,2,10);  //avambr     
              if (pos_braccio3_fin < 80) 
              {          
               movimento_zampa_new3(2,1,20);  //bra   
              }
          }   
          else
          {
            /*
              //----------- LA ZAMPA 3 SI RIAPRE IN EMERGENZA PER RIATTACCARSI AL SUOLO!
              movimento_zampa_new3(1,2,110); //avambr          
              if (pos_braccio3_fin < 80)
              {            
               movimento_zampa_new3(2,1,75); //bra  
              }   
            */ 
          }          
           
        }//fine if(valore_tar_des3_SOMMA < 0)
     }  
 
timer_generico3 = 0;
}//fine if(timer_generico3 == 3)  

}//fine if(spinta_zampe == 0)  

if(spinta_zampe == 1)
{
 spinta_zampe_codice();
}

//altezza_zampe();
//questo altezza zampe che ci fa qui???




//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************
//**********************************                           ********************************
//*********************************  ROUTINE CAMMINATA FURTIVA  *******************************
//**********************************                           ********************************
//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************

/*
     equilib pre ricarica zampa 3
     prima raggiungere X 20 e poi Y picchiare -2

     e verificare  quanto indietro e' la zampa1 nel freeze di
     equilibrio pre ricarica! se fosse piu' indietro, sarebbe
     piu' semplice equilibrare la zampa 3?
*/

// FASE 0 ----------------------------------------------------------------------------------------------------
//------------------------------- IL ROBOT SI STABILIZZA DRITTO PRIMA DI INIZIARE CAMMINATA ------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 0)
{
  //riflessi_massimi = 0;
  durata_timer = 2; // si puo' metter 2!
  durata_spinte = 15; //era 15
  tempo_attesa_appo = 2; //velocit equilibrio_per_ricarica
/*
    target_x_appo = 20;// inclinaz a sinistra ** 15 **
    target_y_appo = -5; //-5;// picchiata
    tempo_attesa_appo = 1; //serve per dire a che velocita' avverra' il raggiungimento di target_x_appo! piu' alta la variabile piu' lento il movimento
    equilibrio_per_ricarica();
*/  
  

//    timer_generico = timer_generico + 1;
//    if(timer_generico >= durata_timer)

//zampa1_no_equilibrio = 1;
//equilibrio pre ricarica zampa 1
//target X = 10   target Y = 5 con zampa 1 disabilitata! ed equilibra bene!!! 

  if(flag_altezza_serial == 1)
  {
    /*
    if(pillozzo == 0)
    {
     ok_zampa1 = 0;  
     moto_zampa1 = 1;
     pillozzo = 1;
     //zampa1_no_equilibrio = 1;
    }
    */
   //ricarica_zampa1(); 


   lentezza_spinte = 3;
   spinta_zampe_codice();
  }


  timer_generico2 = timer_generico2 + 1;

 //ok provare i nuovi limiti minimi di abbassamento delle zampe anteriori (le posteriori in ambito minimo/massimo ora vanno bene!)

  if(angolo_roll_attuale == target_x && angolo_pitch_attuale == target_y || timer_generico2 > 50)
  {    
    timer_generico = timer_generico + 1;
    if(timer_generico >= durata_timer) // durata_timer)
    {
          
     timer_generico = 0; 
     fase_mov_zampe = 1;//------------------------------------------------------------------------------------------------------------QUI     
     
     
    }     
  }   
  else
  {
   timer_generico = 0;
  }    
}



// FASE 1 ----------------------------------------------------------------------------------------------------
//-------------------------------------------- TUTTE LE ZAMPE SPINGONO! 0 oppure FASE TEST GENERICI ----------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 1)
{
/*  
  zampa1_no_equilibrio = 1;
  zampa2_no_equilibrio = 1;
  zampa3_no_equilibrio = 1;
  zampa4_no_equilibrio = 1; 


    altezza_zampe(); //220 e' l'altezza iniziale
  
    //SE ALTEZZA ROBOT INFERIORE ALLA MEDIA (211 / 229) ALLORA IL ROBOT SI ALZA
    if(altezza_robot_totale <= 190) 
    {  
     modo_altezza1 = 1; //zampa si allunga
     modo_altezza2 = 1; //zampa si allunga
     modo_altezza3 = 1; //zampa si allunga
     modo_altezza4 = 1; //zampa si allunga
    }    
    //SE ALTEZZA ROBOT SUPERIORE ALLA MEDIA (211 / 229) ALLORA IL ROBOT SI ABBASSA
    if(altezza_robot_totale >= 200) 
    {  
     modo_altezza1 = 2; //zampa si abbassa
     modo_altezza2 = 2; //zampa si abbassa
     modo_altezza3 = 2; //zampa si abbassa
     modo_altezza4 = 2; //zampa si abbassa
    }
    //SE ALTEZZA ROBOT RIENTRA NELL'AREA MEDIA (211 / 229) ALLORA IL ROBOT HA ESEGUITO IL RESET ALTEZZA!
    if(altezza_robot_totale > 190 && altezza_robot_totale < 200) 
    {

      modo_altezza1 = 0; //zampa si abbassa
      modo_altezza2 = 0; //zampa si abbassa
      modo_altezza3 = 0; //zampa si abbassa
      modo_altezza4 = 0; //zampa si abbassa
      
      zampa1_no_equilibrio = 0;
      zampa2_no_equilibrio = 0;
      zampa3_no_equilibrio = 0;
      zampa4_no_equilibrio = 0;  
      fase_mov_zampe = 2;
    } 
*/  
  
  
  
  
  
  fase_mov_zampe = 2;
   

}


// FASE 2 ----------------------------------------------------------------------------------------------------
//---------------------------------- EQUILIBRA PER RICARICA ZAMPA 1 ----------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 2)
{
/*  
     //-------------------------------------------------------------- MEMORIZZIAMO COORDINATE ZAMPE
     if (flag_memorizz_coord_zampa == 0)
     {
      //flag_memorizz_coord_bra1 = pos_braccio1_fin;
      //flag_memorizz_coord_avambra1 = pos_avambraccio1_fin;                   
      flag_memorizz_coord_zampa = 1; 
     }
*/

    if (flag_equilib_zampe == 0)
    {
      zampa1_no_equilibrio = 1;
      zampa2_no_equilibrio = 1;
      zampa3_no_equilibrio = 1;
      zampa4_no_equilibrio = 1;
      
      altezza_zampe();
      modo_altezza2 = 2; //zampa 2 si chiude
      modo_altezza4 = 2; //zampa 4 si chiude
      modo_altezza3 = 1; //zampa 3 si apre
    }

    if (fsrReading1 <= 250 && angolo_roll_attuale >= 5)
    {
      flag_equilib_zampe = 1;
     
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
           zampa1_no_equilibrio = 0;
           zampa2_no_equilibrio = 0;
           zampa3_no_equilibrio = 0;
           zampa4_no_equilibrio = 0;        
                
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           flag_equilib_zampe = 0;
           fase_mov_zampe = 3;        
      }          
    }


/*
    //incliniamo il robot a sinistra, un grado alla volta fino che il piede e' bene schiacciato a terra 
    if (fsrReading1 >= 250 || target_x < 5) 
    {
     zampa1_no_equilibrio = 1; 
     target_x_appo = target_x_appo + 1;
     equilibrio_per_ricarica();
    }

    //La pressione del piede a terra e' sotto la soglia e quindi FINE FASE
    if (fsrReading1 < 250 && target_x >= 5) 
    {
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           fase_mov_zampe = 3;        
      }
    }
*/


/*  
    //********************************** NUOVA ROUTINE!
    //FINCHE' C'E' ABBASTANZA PRESSIONE NEL PIEDE, IL ROBOT SI INCLINA PER EQUILIBRARE.
    if (fsrReading1 > 250)
    {
      zampa1_no_equilibrio = 1;
      target_x_appo = 15; //10  20 10
      target_y_appo = 0; //  5  10  5
      //tempo_attesa_appo = 1; //serve per dire a che velocita' avverra' il raggiungimento di target_x_appo! piu' alta la variabile piu' lento il movimento
      equilibrio_per_ricarica();     
    }
    else
    {            
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
          
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           fase_mov_zampe = 3;              
      }        
    }//fine else
*/





    
} //fine if (fase_mov_zampe == 2)



// FASE 3 ----------------------------------------------------------------------------------------------------
//--------------------------------------------- RICARICA ZAMPA 1 ---------------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 3)
{

   zampa1_no_equilibrio = 1; // gia' settato a 1 nella fase precedente
 
   // -------------------------------------------------------- CODICE DI SICUREZZA ANTI-BUG ZAMPA 1 
   timer_generico = timer_generico + 1;
   if (timer_generico == 1)  //prova con timer_generico == 0
   {
    ok_zampa1 = 0; 
    moto_zampa1 = 1; 
   }    

   // -------------------------------------------------------- MOVIMENTO RICARICA ZAMPA 1    
   ricarica_zampa1();


   // -------------------------------------------------------- FINE RICARICA ZAMPA 1
   if(ok_zampa1 == 1)
   {
   
   
   
   // ----------------------------------- EQUILIBRIO RIABILITATO A ZAMPA 1 CHE HA FINITO DI CARICARE
       zampa1_no_equilibrio = 0; //la zampa 1, dopo la ricarica, riprende ad aiutare le altre zampe per equilibrio
       //zampa2_no_equilibrio = 0;
       //zampa3_no_equilibrio = 0;
       //zampa4_no_equilibrio = 0;
       
       target_x_appo = 0;// inclinaz a des -2
       target_y_appo = 0;
       //tempo_attesa_appo = 1; //serve per dire a che velocita' avverra' il raggiungimento di target_x_appo! piu' alta la variabile piu' lento il movimento
       equilibrio_per_ricarica();   
/*      
   // -------------------------------------------------------- RADRIZZAMENTO ZAMPE A FINE RICARICA ZAMPA 1   
       if(flag_fine_raddrizzamento2 == 0)
       {
          zampa2_no_equilibrio = 1;
          
          if(pos_avambraccio2_fin != flag_memorizz_coord_avambra2 && pos_braccio2_fin != flag_memorizz_coord_bra2)
          {
            movimento_zampa_new2(1 , 2, flag_memorizz_coord_avambra2);   //avambr 20
            movimento_zampa_new2(2 , 1, flag_memorizz_coord_bra2);  //bra 30          
          }
          else
          {
             flag_fine_raddrizzamento2 = 1;
             zampa2_no_equilibrio = 0;          
          }       
       } //fine if(flag_fine_raddrizzamento2 == 0)
*/

          
   // ----------------------------------- RAGGIUNTA L'INCLINAZIONE TARGET 0
       if(angolo_roll_attuale == target_x && angolo_pitch_attuale == target_y) // && pos_spalla1_fin == 70 && pos_spalla3_fin == 70 && pos_spalla2_fin == 50 && pos_spalla4_fin == 50) 
       {
         timer_generico2 = timer_generico2 + 1;
         if(timer_generico2 >= durata_timer)
         {               
     
          zampa1_no_equilibrio = 0; //EQUILIBRIO RIABILITATO PER ZAMPA 1
     
           
          timer_generico = 0;
          timer_generico2 = 0;
          //timer_generico3 = 0;
          timer_generico_equilibrio = 0;
          timer_raggiung_target_x = 0;
          timer_raggiung_target_y = 0;
          flag_fine_raddrizzamento1 = 0, flag_fine_raddrizzamento2 = 0, flag_fine_raddrizzamento3 = 0, flag_fine_raddrizzamento4 = 0;
          fase_mov_zampe = 4;
         }      
       } //fine if(target_x == angolo_roll_attuale && target_y == angolo_pitch_attuale)                   

   }//fine if(ok_zampa1 == 1)


   
} //fine if (fase_mov_zampe == 3)  

// FASE 4 ----------------------------------------------------------------------------------------------------
//-------------------------------------------- RESET ALTEZZA ROBOT -------------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 4)
{
  
  zampa1_no_equilibrio = 1;
  zampa2_no_equilibrio = 1;
  zampa3_no_equilibrio = 1;
  zampa4_no_equilibrio = 1; 


    altezza_zampe(); //220 e' l'altezza iniziale
  
    //SE ALTEZZA ROBOT INFERIORE ALLA MEDIA (211 / 229) ALLORA IL ROBOT SI ALZA
    if(altezza_robot_totale <= 230) //230
    {  
     modo_altezza1 = 1; //zampa si allunga
     modo_altezza2 = 1; //zampa si allunga
     modo_altezza3 = 1; //zampa si allunga
     modo_altezza4 = 1; //zampa si allunga
    }    
    //SE ALTEZZA ROBOT SUPERIORE ALLA MEDIA (211 / 229) ALLORA IL ROBOT SI ABBASSA
    if(altezza_robot_totale >= 240) //240
    {  
     modo_altezza1 = 2; //zampa si abbassa
     modo_altezza2 = 2; //zampa si abbassa
     modo_altezza3 = 2; //zampa si abbassa
     modo_altezza4 = 2; //zampa si abbassa
    }
    //SE ALTEZZA ROBOT RIENTRA NELL'AREA MEDIA (211 / 229) ALLORA IL ROBOT HA ESEGUITO IL RESET ALTEZZA!
    if(altezza_robot_totale > 230 && altezza_robot_totale < 240) 
    {

      modo_altezza1 = 0; //zampa si abbassa
      modo_altezza2 = 0; //zampa si abbassa
      modo_altezza3 = 0; //zampa si abbassa
      modo_altezza4 = 0; //zampa si abbassa
      
      zampa1_no_equilibrio = 0;
      zampa2_no_equilibrio = 0;
      zampa3_no_equilibrio = 0;
      zampa4_no_equilibrio = 0;  
      fase_mov_zampe = 5;
    } 
  
  
}// fine if (fase_mov_zampe == 4)



// FASE 5 ----------------------------------------------------------------------------------------------------
//-------------------------------------------- TUTTE LE ZAMPE SPINGONO! 1 --------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 5)
{

    lentezza_spinte = 3; 
    spinta_zampe = 1;

    timer_generico = timer_generico + 1;
    if(timer_generico >= durata_spinte) 
    {      
      timer_generico2 = timer_generico2 + 1;
      if (timer_generico2 == durata_timer)
      {
        timer_generico = 0; 
        timer_generico2 = 0;
        spinta_zampe = 0;      
        fase_mov_zampe = 6; 
      }        
    }     
  

  
/*  
    lentezza_spinte = 3; 
    spinta_zampe = 1;

    //timer_generico = timer_generico + 1;
    //if(timer_generico >= durata_spinte) 
    if(pos_braccio2_fin >= 127 && pos_avambraccio2_fin >= 76)     
    { 
      spinta_zampe = 0;
      
      timer_generico2 = timer_generico2 + 1;
      if (timer_generico2 == durata_timer)
      {
        timer_generico = 0; 
        timer_generico2 = 0;              
        fase_mov_zampe = 6; 
      }        
    }   
*/  
  

    
}



// FASE 6 ----------------------------------------------------------------------------------------------------
//---------------------------------- EQUILIBRA PER RICARICA ZAMPA 4 ------------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 6)
{
  
    if (flag_equilib_zampe == 0)
    {
      zampa1_no_equilibrio = 1;
      zampa2_no_equilibrio = 1;
      zampa3_no_equilibrio = 1;
      zampa4_no_equilibrio = 1;
      
      altezza_zampe();
      modo_altezza1 = 2; //zampa 1 si chiude
      modo_altezza3 = 2; //zampa 3 si chiude
      modo_altezza2 = 1; //zampa 2 si apre
    }

    if (fsrReading4 <= 250 && angolo_roll_attuale <= -5)
    {
      flag_equilib_zampe = 1;
     
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
           //zampa1_no_equilibrio = 0;
           //zampa2_no_equilibrio = 0;
           //zampa3_no_equilibrio = 0;
           //zampa4_no_equilibrio = 0;        
                
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           flag_equilib_zampe = 0;
           fase_mov_zampe = 7;        
      }          
    }
  
 /* 
    //incliniamo il robot a sinistra, un grado alla volta fino che il piede e' bene schiacciato a terra 
    if (fsrReading4 >= 250 || target_x > -5) 
    {
     zampa4_no_equilibrio = 1; 
     target_x_appo = target_x_appo - 1;
     equilibrio_per_ricarica();
    }

    //La pressione del piede a terra e' sotto la soglia e quindi FINE FASE
    if (fsrReading4 < 250 && target_x <= -5) 
    {
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           fase_mov_zampe = 7;        
      }
    }  
*/  
  
/*
    //********************************** NUOVA ROUTINE!
    //FINCHE' C'E' ABBASTANZA PRESSIONE NEL PIEDE, IL ROBOT SI INCLINA PER EQUILIBRARE.
    if (fsrReading4 > 250)
    {
      zampa4_no_equilibrio = 1;
      //era X=-5 e Y=0
      target_x_appo = -10; //10  -20 -5   -5
      target_y_appo = 2; //  5  -10 -5   0
      //tempo_attesa_appo = 2; //serve per dire a che velocita' avverra' il raggiungimento di target_x_appo! piu' alta la variabile piu' lento il movimento
      equilibrio_per_ricarica();     
    }
    else
    {       

      
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {         
           timer_generico = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           fase_mov_zampe = 7;              
      }        
    }//fine else
*/  


}//fine if (fase_mov_zampe == 6)






// FASE 7 ----------------------------------------------------------------------------------------------------
//--------------------------------------------- RICARICA ZAMPA 4 ---------------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 7)
{  
   //zampa4_no_equilibrio = 1; // gia' settato a 1 nella fase precedente
 
   // -------------------------------------------------------- CODICE DI SICUREZZA ANTI-BUG ZAMPA 4
   timer_generico = timer_generico + 1;
   if (timer_generico == 1)  //prova con timer_generico == 0
   {
    ok_zampa4 = 0; 
    moto_zampa4 = 1; 
   }    

   // -------------------------------------------------------- MOVIMENTO RICARICA ZAMPA 4   
   ricarica_zampa4();


   // -------------------------------------------------------- FINE RICARICA ZAMPA 4
   if(ok_zampa4 == 1)
   {
   
   // ----------------------------------- EQUILIBRIO RIABILITATO A ZAMPA 4 CHE HA FINITO DI CARICARE
       zampa4_no_equilibrio = 0; //la zampa 1, dopo la ricarica, riprende ad aiutare le altre zampe per equilibrio
       zampa1_no_equilibrio = 0;
       zampa2_no_equilibrio = 0;
       zampa3_no_equilibrio = 0;

       target_x_appo = 0; //2
       target_y_appo = 0;
       //tempo_attesa_appo = 2; //serve per dire a che velocita' avverra' il raggiungimento di target_x_appo! piu' alta la variabile piu' lento il movimento
       equilibrio_per_ricarica();   
/*      
   // -------------------------------------------------------- RADRIZZAMENTO ZAMPE A FINE RICARICA ZAMPA 1   
       if(flag_fine_raddrizzamento2 == 0)
       {
          zampa2_no_equilibrio = 1;
          
          if(pos_avambraccio2_fin != flag_memorizz_coord_avambra2 && pos_braccio2_fin != flag_memorizz_coord_bra2)
          {
            movimento_zampa_new2(1 , 2, flag_memorizz_coord_avambra2);   //avambr 20
            movimento_zampa_new2(2 , 1, flag_memorizz_coord_bra2);  //bra 30          
          }
          else
          {
             flag_fine_raddrizzamento2 = 1;
             zampa2_no_equilibrio = 0;          
          }       
       } //fine if(flag_fine_raddrizzamento2 == 0)
*/




          
   // ----------------------------------- RAGGIUNTA L'INCLINAZIONE TARGET 0
       if(angolo_roll_attuale == target_x && angolo_pitch_attuale == target_y) // && pos_spalla1_fin == 70 && pos_spalla3_fin == 70 && pos_spalla2_fin == 50 && pos_spalla4_fin == 50) 
       {
         timer_generico2 = timer_generico2 + 1;
         if(timer_generico2 >= durata_timer)
         {               
     
          zampa4_no_equilibrio = 0; //EQUILIBRIO RIABILITATO PER ZAMPA 4
     
           
          timer_generico = 0;
          timer_generico2 = 0;
          timer_generico3 = 0;
          timer_generico_equilibrio = 0;
          timer_raggiung_target_x = 0;
          timer_raggiung_target_y = 0;
          flag_fine_raddrizzamento1 = 0, flag_fine_raddrizzamento2 = 0, flag_fine_raddrizzamento3 = 0, flag_fine_raddrizzamento4 = 0;
          fase_mov_zampe = 8;
         }      
       } //fine if(target_x == angolo_roll_attuale && target_y == angolo_pitch_attuale)                   

   }//fine if(ok_zampa1 == 1)
} //fine if (fase_mov_zampe == 7) 

// FASE 8 ----------------------------------------------------------------------------------------------------
//-------------------------------------------- RESET ALTEZZA ROBOT --------------------------------------------
//-------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 8)
{
  
  zampa1_no_equilibrio = 1;
  zampa2_no_equilibrio = 1;
  zampa3_no_equilibrio = 1;
  zampa4_no_equilibrio = 1; 


    altezza_zampe(); //220 e' l'altezza iniziale
  
    //SE ALTEZZA ROBOT INFERIORE ALLA MEDIA (211 / 229) ALLORA IL ROBOT SI ALZA
    if(altezza_robot_totale <= 230) 
    {  
     modo_altezza1 = 1; //zampa si allunga
     modo_altezza2 = 1; //zampa si allunga
     modo_altezza3 = 1; //zampa si allunga
     modo_altezza4 = 1; //zampa si allunga
    }    
    //SE ALTEZZA ROBOT SUPERIORE ALLA MEDIA (211 / 229) ALLORA IL ROBOT SI ABBASSA
    if(altezza_robot_totale >= 240) 
    {  
     modo_altezza1 = 2; //zampa si abbassa
     modo_altezza2 = 2; //zampa si abbassa
     modo_altezza3 = 2; //zampa si abbassa
     modo_altezza4 = 2; //zampa si abbassa
    }
    //SE ALTEZZA ROBOT RIENTRA NELL'AREA MEDIA (211 / 229) ALLORA IL ROBOT HA ESEGUITO IL RESET ALTEZZA!
    if(altezza_robot_totale > 230 && altezza_robot_totale < 240) 
    {

      modo_altezza1 = 0; //zampa si abbassa
      modo_altezza2 = 0; //zampa si abbassa
      modo_altezza3 = 0; //zampa si abbassa
      modo_altezza4 = 0; //zampa si abbassa
      
      zampa1_no_equilibrio = 0;
      zampa2_no_equilibrio = 0;
      zampa3_no_equilibrio = 0;
      zampa4_no_equilibrio = 0;  
      fase_mov_zampe = 9;
    } 
}  
  
// FASE 9 ----------------------------------------------------------------------------------------------------
//-------------------------------------------- TUTTE LE ZAMPE SPINGONO! 2 --------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 9)
{
  
    //riflessi_massimi = 1;    
    //velocita_spinta = 1;  //piu alto = piu' lento

    lentezza_spinte = 3; 
    spinta_zampe = 1;

    timer_generico = timer_generico + 1;
    if(timer_generico >= durata_spinte) 
    {      
      timer_generico2 = timer_generico2 + 1;
      if (timer_generico2 == durata_timer)
      {
        timer_generico = 0; 
        timer_generico2 = 0;
        spinta_zampe = 0;      
        fase_mov_zampe = 10; 
      }        
    }     
}


// FASE 10 ---------------------------------------------------------------------------------------------------
//-------------------------------------------EQUILIBRA PER RICARICA ZAMPA 2 ----------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 10)
{
  
    if (flag_equilib_zampe == 0)
    {
      zampa1_no_equilibrio = 1;
      zampa2_no_equilibrio = 1;
      zampa3_no_equilibrio = 1;
      zampa4_no_equilibrio = 1;
      
      altezza_zampe();
      modo_altezza1 = 2; //zampa 1 si chiude
      modo_altezza3 = 2; //zampa 3 si chiude
      modo_altezza4 = 1; //zampa 4 si apre
    }

    if (fsrReading2 <= 250 && angolo_roll_attuale <= -5)
    {
      flag_equilib_zampe = 1;
     
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
           zampa1_no_equilibrio = 0;
           zampa2_no_equilibrio = 0;
           zampa3_no_equilibrio = 0;
           zampa4_no_equilibrio = 0;        
                
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           flag_equilib_zampe = 0;
           fase_mov_zampe = 11;        
      }          
    }  
  
 /* 
    //incliniamo il robot a destra, un grado alla volta fino che il piede e' bene schiacciato a terra 
    if (fsrReading2 >= 250 || target_x > -5) 
    {
     zampa2_no_equilibrio = 1; 
     target_x_appo = target_x_appo - 1;
     equilibrio_per_ricarica();
    }

    //La pressione del piede a terra e' sotto la soglia e quindi FINE FASE
    if (fsrReading2 < 250 && target_x <= -5) 
    {
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           fase_mov_zampe = 11;        
      }
    }    
 */ 
  
/*  
    //********************************** NUOVA ROUTINE!
    //FINCHE' C'E' ABBASTANZA PRESSIONE NEL PIEDE, IL ROBOT SI INCLINA PER EQUILIBRARE.
    if (fsrReading2 > 250)
    {
      zampa2_no_equilibrio = 1;
      //era X=-5 e Y=5
      target_x_appo = -20; //10  20 
      target_y_appo = 10; //  5  10
      //tempo_attesa_appo = 2; //serve per dire a che velocita' avverra' il raggiungimento di target_x_appo! piu' alta la variabile piu' lento il movimento
      equilibrio_per_ricarica();     
    }
    else
    {               
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {            
           timer_generico = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           fase_mov_zampe = 11;              
      }        
    }//fine else  
*/  

    
} //fine if (fase_mov_zampe == 10)




// FASE 11 ----------------------------------------------------------------------------------------------------
//--------------------------------------------- RICARICA ZAMPA 2 ---------------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 11)
{
   zampa2_no_equilibrio = 1; // gia' settato a 1 nella fase precedente
 
   // -------------------------------------------------------- CODICE DI SICUREZZA ANTI-BUG ZAMPA 2 
   timer_generico = timer_generico + 1;
   if (timer_generico == 1)  //prova con timer_generico == 0
   {
    ok_zampa2 = 0; 
    moto_zampa2 = 1; 
   }    

   // -------------------------------------------------------- MOVIMENTO RICARICA ZAMPA 2   
   ricarica_zampa2();


   // -------------------------------------------------------- FINE RICARICA ZAMPA 2
   if(ok_zampa2 == 1)
   {
   
   // ----------------------------------- EQUILIBRIO RIABILITATO A ZAMPA 1 CHE HA FINITO DI CARICARE
       zampa2_no_equilibrio = 0; //la zampa 1, dopo la ricarica, riprende ad aiutare le altre zampe per equilibrio
       //zampa1_no_equilibrio = 0;
       //zampa3_no_equilibrio = 0;
       //zampa4_no_equilibrio = 0;

       target_x_appo = 0;//2
       target_y_appo = 0;
       //tempo_attesa_appo = 2; //serve per dire a che velocita' avverra' il raggiungimento di target_x_appo! piu' alta la variabile piu' lento il movimento
       equilibrio_per_ricarica();   
/*      
   // -------------------------------------------------------- RADRIZZAMENTO ZAMPE A FINE RICARICA ZAMPA 2  
       if(flag_fine_raddrizzamento2 == 0)
       {
          zampa2_no_equilibrio = 1;
          
          if(pos_avambraccio2_fin != flag_memorizz_coord_avambra2 && pos_braccio2_fin != flag_memorizz_coord_bra2)
          {
            movimento_zampa_new2(1 , 2, flag_memorizz_coord_avambra2);   //avambr 20
            movimento_zampa_new2(2 , 1, flag_memorizz_coord_bra2);  //bra 30          
          }
          else
          {
             flag_fine_raddrizzamento2 = 1;
             zampa2_no_equilibrio = 0;          
          }       
       } //fine if(flag_fine_raddrizzamento2 == 0)
*/

          
   // ----------------------------------- RAGGIUNTA L'INCLINAZIONE TARGET 0
       if(angolo_roll_attuale == target_x && angolo_pitch_attuale == target_y) // && pos_spalla1_fin == 70 && pos_spalla3_fin == 70 && pos_spalla2_fin == 50 && pos_spalla4_fin == 50) 
       {
         timer_generico2 = timer_generico2 + 1;
         if(timer_generico2 >= durata_timer)
         {               
     
          zampa2_no_equilibrio = 0; //EQUILIBRIO RIABILITATO PER ZAMPA 2
     
           
          timer_generico = 0;
          timer_generico2 = 0;
          //timer_generico3 = 0;
          timer_generico_equilibrio = 0;
          timer_raggiung_target_x = 0;
          timer_raggiung_target_y = 0;
          flag_fine_raddrizzamento1 = 0, flag_fine_raddrizzamento2 = 0, flag_fine_raddrizzamento3 = 0, flag_fine_raddrizzamento4 = 0;
          fase_mov_zampe = 12;
         }      
       } //fine if(target_x == angolo_roll_attuale && target_y == angolo_pitch_attuale)                   

   }//fine if(ok_zampa2 == 1)
}  



// FASE 12 ----------------------------------------------------------------------------------------------------
//-------------------------------------------- RESET ALTEZZA ROBOT --------------------------------------------
//-------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 12)
{
  
  zampa1_no_equilibrio = 1;
  zampa2_no_equilibrio = 1;
  zampa3_no_equilibrio = 1;
  zampa4_no_equilibrio = 1; 


    altezza_zampe(); //220 e' l'altezza iniziale
  
    //SE ALTEZZA ROBOT INFERIORE ALLA MEDIA (211 / 229) ALLORA IL ROBOT SI ALZA
    if(altezza_robot_totale <= 230) 
    {  
     modo_altezza1 = 1; //zampa si allunga
     modo_altezza2 = 1; //zampa si allunga
     modo_altezza3 = 1; //zampa si allunga
     modo_altezza4 = 1; //zampa si allunga
    }    
    //SE ALTEZZA ROBOT SUPERIORE ALLA MEDIA (211 / 229) ALLORA IL ROBOT SI ABBASSA
    if(altezza_robot_totale >= 240) 
    {  
     modo_altezza1 = 2; //zampa si abbassa
     modo_altezza2 = 2; //zampa si abbassa
     modo_altezza3 = 2; //zampa si abbassa
     modo_altezza4 = 2; //zampa si abbassa
    }
    //SE ALTEZZA ROBOT RIENTRA NELL'AREA MEDIA (211 / 229) ALLORA IL ROBOT HA ESEGUITO IL RESET ALTEZZA!
    if(altezza_robot_totale > 230 && altezza_robot_totale < 240) 
    {

      modo_altezza1 = 0; //zampa si abbassa
      modo_altezza2 = 0; //zampa si abbassa
      modo_altezza3 = 0; //zampa si abbassa
      modo_altezza4 = 0; //zampa si abbassa
      
      zampa1_no_equilibrio = 0;
      zampa2_no_equilibrio = 0;
      zampa3_no_equilibrio = 0;
      zampa4_no_equilibrio = 0;  
      fase_mov_zampe = 13;
    } 
}  



// FASE 13 ----------------------------------------------------------------------------------------------------
//-------------------------------------------- TUTTE LE ZAMPE SPINGONO! 3 --------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 13)
{


    //durata_spinte = 25;
  
    lentezza_spinte = 3; 
    spinta_zampe = 1;

    timer_generico = timer_generico + 1;
    if(timer_generico >= durata_spinte) 
    {      
      timer_generico2 = timer_generico2 + 1;
      if (timer_generico2 == durata_timer)
      {
        timer_generico = 0; 
        timer_generico2 = 0;
        spinta_zampe = 0;      
        fase_mov_zampe = 14; 
        //durata_spinte = 15;
      }        
    }    
  
  /*
  //bra1 120
  //ava1 64

    lentezza_spinte = 3; 
    spinta_zampe = 1;

    //timer_generico = timer_generico + 1;
    //if(timer_generico >= durata_spinte) 
    
    if(pos_braccio1_fin >= 120 && pos_avambraccio1_fin >= 64)
    {    
      spinta_zampe = 0;
      
      timer_generico2 = timer_generico2 + 1;
      if (timer_generico2 == durata_timer)
      {
        timer_generico = 0; 
        timer_generico2 = 0;     
        fase_mov_zampe = 14; 
      }        
    }  
  */
  
  
    
}


//e verifica se per ric zampa 3 il robot si inclina a sinistra e picchiata


/*
fare routine che fa durare spinta a seconda di posizione zampe!

EQUILIBRIO RICARICA ZAMPA 1
RICARICA ZAMPA 1
RESET ALTEZZA
SPINTA 1 (dopo ricarica zampa 1) = La spinta finisce quando ZAMPA 2 raggiunge
               
EQUILIBRIO RICARICA ZAMPA 4
RICARICA ZAMPA 4
RESET ALTEZZA
SPINTA 2

EQUILIBRIO RICARICA ZAMPA 2
RICARICA ZAMPA 2
RESET ALTEZZA
SPINTA 3 (dopo ricarica zampa 2) = La spinta finisce quando ZAMPA 1 raggiunge

EQUILIBRIO RICARICA ZAMPA 3
RICARICA ZAMPA 3
RESET ALTEZZA
SPINTA 4
              
*/




// FASE 14 ---------------------------------------------------------------------------------------------------
//---------------------------------- EQUILIBRA PER RICARICA ZAMPA 3 ------------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 14)
{
  
    if (flag_equilib_zampe == 0)
    {
      zampa1_no_equilibrio = 1;
      zampa2_no_equilibrio = 1;
      zampa3_no_equilibrio = 1;
      zampa4_no_equilibrio = 1;
      
      altezza_zampe();
      modo_altezza2 = 2; //zampa 2 si chiude
      modo_altezza4 = 2; //zampa 4 si chiude
      modo_altezza1 = 1; //zampa 1 si apre
      
    }

    if (fsrReading3 <= 250 && angolo_roll_attuale >= 15)//15
    {
      flag_equilib_zampe = 1;
     
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
           //zampa1_no_equilibrio = 0;
           //zampa2_no_equilibrio = 0;
           //zampa3_no_equilibrio = 0;
           //zampa4_no_equilibrio = 0;        
                
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           flag_equilib_zampe = 0;
           fase_mov_zampe = 15;        
      }          
    }  


/*  
    if (flag_equilib_zampe == 0)
    {
      zampa1_no_equilibrio = 1;
      zampa2_no_equilibrio = 1;
      zampa3_no_equilibrio = 1;
      zampa4_no_equilibrio = 1;
      
      altezza_zampe();
      modo_altezza1 = 1; //zampa 1 si apre
      if(pos_avambraccio2_fin >= 40)//65
      {
       modo_altezza3 = 1; //zampa 1 si apre
      }
      else
      {
       modo_altezza3 = 0; //zampa 1 si ferma
      }
      
      modo_altezza2 = 2; //zampa 2 si chiude
      modo_altezza4 = 2; //zampa 4 si chiude 
            
      //timer_generico2 = timer_generico2 + 1;
      //if(pos_avambraccio1_fin >= 90)
      //{       
      // modo_altezza2 = 2; //zampa 2 si chiude
      // modo_altezza4 = 2; //zampa 4 si chiude       
      //}           
    }


    if (fsrReading3 <= 250 && angolo_roll_attuale >= 5)//15
    {
      flag_equilib_zampe = 1;
     
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
           zampa1_no_equilibrio = 0;
           zampa2_no_equilibrio = 0;
           zampa3_no_equilibrio = 0;
           zampa4_no_equilibrio = 0;        
                
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           flag_equilib_zampe = 0;
           fase_mov_zampe = 15;        
      }          
    }    
*/

  
  
  /*
    //incliniamo il robot a destra, un grado alla volta fino che il piede e' bene schiacciato a terra 
    if (fsrReading3 >= 250 || target_x < 5)  
    {
     zampa3_no_equilibrio = 1; 
     target_x_appo = target_x_appo + 1;
     target_y_appo = 2;
     equilibrio_per_ricarica();
    }

    //La pressione del piede a terra e' sotto la soglia e quindi FINE FASE
    if (fsrReading3 < 250 && target_x >= 5) 
    {
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           fase_mov_zampe = 15;        
      }
    }      
  */
  
  
  
  
  /*
  
    zampa3_no_equilibrio = 1; 
    target_x_appo = 10;
    target_y_appo = -5;
    equilibrio_per_ricarica();
  
    //incliniamo il robot a sinistra, un grado alla volta fino che il piede e' bene schiacciato a terra 
    if (fsrReading3 >= 250) 
    {
     //zampa3_no_equilibrio = 1; 
     //target_x_appo = target_x_appo + 1; //usiamo + 2 per far andare piu' rapida l'inclinazione per evitare che sia troppo poca considerando che il sensore tattile 3 e' da verificare se funge bene quando la zampa 3 e' troppo indietro!
     //target_y_appo = -2;
     //equilibrio_per_ricarica();
    }

    //La pressione del piede a terra e' sotto la soglia e quindi FINE FASE
    if (fsrReading3 < 250) 
    {
      timer_generico = timer_generico + 1;
      if(timer_generico >= durata_timer)
      {  
           timer_generico = 0;
           timer_generico2 = 0;
           timer_generico_equilibrio = 0;
           timer_raggiung_target_x = 0;
           timer_raggiung_target_y = 0;   
           flag_memorizz_coord_zampa = 0;
           fase_mov_zampe = 15;        
      }
    }    

*/





    
}//fine if (fase_mov_zampe == 14)



// FASE 15 ---------------------------------------------------------------------------------------------------
//--------------------------------------------- RICARICA ZAMPA 3 ---------------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 15)
{  
   
   //zampa3_no_equilibrio = 1; // gia' settato a 1 nella fase precedente
   
 
   // -------------------------------------------------------- CODICE DI SICUREZZA ANTI-BUG ZAMPA 3
   timer_generico = timer_generico + 1;
   if (timer_generico == 1)  //prova con timer_generico == 0
   {
    ok_zampa3 = 0; 
    moto_zampa3 = 1; 
   }    

   // -------------------------------------------------------- MOVIMENTO RICARICA ZAMPA 3   
   ricarica_zampa3();



   // -------------------------------------------------------- FINE RICARICA ZAMPA 3
   if(ok_zampa3 == 1)
   {     

   // ----------------------------------- EQUILIBRIO RIABILITATO A ZAMPA 3 CHE HA FINITO DI CARICARE
       zampa3_no_equilibrio = 0; //la zampa 1, dopo la ricarica, riprende ad aiutare le altre zampe per equilibrio
       zampa1_no_equilibrio = 0;
       zampa2_no_equilibrio = 0;
       zampa4_no_equilibrio = 0;

       target_x_appo = 0;//-2
       target_y_appo = 0;
       //tempo_attesa_appo = 2; //serve per dire a che velocita' avverra' il raggiungimento di target_x_appo! piu' alta la variabile piu' lento il movimento
       equilibrio_per_ricarica();   
/*      
   // -------------------------------------------------------- RADRIZZAMENTO ZAMPE A FINE RICARICA ZAMPA 3   
       if(flag_fine_raddrizzamento2 == 0)
       {
          zampa2_no_equilibrio = 1;
          
          if(pos_avambraccio2_fin != flag_memorizz_coord_avambra2 && pos_braccio2_fin != flag_memorizz_coord_bra2)
          {
            movimento_zampa_new2(1 , 2, flag_memorizz_coord_avambra2);   //avambr 20
            movimento_zampa_new2(2 , 1, flag_memorizz_coord_bra2);  //bra 30          
          }
          else
          {
             flag_fine_raddrizzamento2 = 1;
             zampa2_no_equilibrio = 0;          
          }       
       } //fine if(flag_fine_raddrizzamento2 == 0)
*/

          
   // ----------------------------------- RAGGIUNTA L'INCLINAZIONE TARGET 0
       if(angolo_roll_attuale == target_x && angolo_pitch_attuale == target_y) // && pos_spalla1_fin == 70 && pos_spalla3_fin == 70 && pos_spalla2_fin == 50 && pos_spalla4_fin == 50) 
       {
         timer_generico2 = timer_generico2 + 1;
         if(timer_generico2 >= durata_timer)
         {               
     
          zampa3_no_equilibrio = 0; //EQUILIBRIO RIABILITATO PER ZAMPA 3
     
           
          timer_generico = 0;
          timer_generico2 = 0;
          timer_generico3 = 0;
          timer_generico_equilibrio = 0;
          timer_raggiung_target_x = 0;
          timer_raggiung_target_y = 0;
          flag_fine_raddrizzamento1 = 0, flag_fine_raddrizzamento2 = 0, flag_fine_raddrizzamento3 = 0, flag_fine_raddrizzamento4 = 0;
          fase_mov_zampe = 16;
         }      
       } //fine if(target_x == angolo_roll_attuale && target_y == angolo_pitch_attuale)                   

   }//fine if(ok_zampa1 == 1)
} //fine if (fase_mov_zampe == 15) 


// FASE 16 ----------------------------------------------------------------------------------------------------
//-------------------------------------------- RESET ALTEZZA ROBOT --------------------------------------------
//-------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 16)
{
  
  zampa1_no_equilibrio = 1;
  zampa2_no_equilibrio = 1;
  zampa3_no_equilibrio = 1;
  zampa4_no_equilibrio = 1; 


    altezza_zampe(); //220 e' l'altezza iniziale
  
    //SE ALTEZZA ROBOT INFERIORE ALLA MEDIA (211 / 229) ALLORA IL ROBOT SI ALZA
    if(altezza_robot_totale <= 230) 
    {  
     modo_altezza1 = 1; //zampa si allunga
     modo_altezza2 = 1; //zampa si allunga
     modo_altezza3 = 1; //zampa si allunga
     modo_altezza4 = 1; //zampa si allunga
    }    
    //SE ALTEZZA ROBOT SUPERIORE ALLA MEDIA (211 / 229) ALLORA IL ROBOT SI ABBASSA
    if(altezza_robot_totale >= 240) 
    {  
     modo_altezza1 = 2; //zampa si abbassa
     modo_altezza2 = 2; //zampa si abbassa
     modo_altezza3 = 2; //zampa si abbassa
     modo_altezza4 = 2; //zampa si abbassa
    }
    //SE ALTEZZA ROBOT RIENTRA NELL'AREA MEDIA (211 / 229) ALLORA IL ROBOT HA ESEGUITO IL RESET ALTEZZA!
    if(altezza_robot_totale > 230 && altezza_robot_totale < 240) 
    {

      modo_altezza1 = 0; //zampa si abbassa
      modo_altezza2 = 0; //zampa si abbassa
      modo_altezza3 = 0; //zampa si abbassa
      modo_altezza4 = 0; //zampa si abbassa
      
      zampa1_no_equilibrio = 0;
      zampa2_no_equilibrio = 0;
      zampa3_no_equilibrio = 0;
      zampa4_no_equilibrio = 0;  
      fase_mov_zampe = 17;
    } 
}  



// FASE 17 ----------------------------------------------------------------------------------------------------
//-------------------------------------------- TUTTE LE ZAMPE SPINGONO! 4 --------------------------------------
//------------------------------------------------------------------------------------------------------------  
if (fase_mov_zampe == 17)
{
    //riflessi_massimi = 1;    
    //velocita_spinta = 1;  //piu alto = piu' lento

    lentezza_spinte = 3; 
    spinta_zampe = 1;

    timer_generico = timer_generico + 1;
    if(timer_generico >= durata_spinte) 
    {      
      timer_generico2 = timer_generico2 + 1;
      if (timer_generico2 == durata_timer)
      {
        timer_generico = 0; 
        timer_generico2 = 0;
        timer_generico3 = 0;        
        spinta_zampe = 0;      
        fase_mov_zampe = 0; 
      }        
    }      
}// fine if (fase_mov_zampe == 17)








      // QUESTO E' L'ULTIMO CODICE GESTIONE SERVI! 8/1/2017
      //************************************************************************************** ZAMPA 1 SETUP 
      //SETTAGGIO INIZIALE AVAMBRACCIO - 0
      new_pos_avambraccio1 = 180 - pos_avambraccio1_fin;
      pulselengthz = map(new_pos_avambraccio1, 0, 180, SERVOMIN_ava1, SERVOMAX_ava1);
      pwm.setPWM(0, 0, pulselengthz); 
      
      //SETTAGGIO INIZIALE BRACCIO - 1
      pulselengthzb = map(pos_braccio1_fin, 0, 180, SERVOMIN_bra1, SERVOMAX_bra1);
      pwm.setPWM(1, 0, pulselengthzb); 
           
      //SETTAGGIO INIZIALE SPALLA - 2
      new_pos_spalla1 = 180 - pos_spalla1_fin;
      pulselengthzc = map(new_pos_spalla1, 0, 180, SERVOMIN_spa1, SERVOMAX_spa1);
      pwm.setPWM(2, 0, pulselengthzc);  
   
      //************************************************************************************** ZAMPA 2 SETUP ---------- Con variabili fin!
      //SETTAGGIO INIZIALE AVAMBRACCIO - 4
      new_pos_avambraccio2 = 180 - pos_avambraccio2_fin;      
      pulselengthz = map(new_pos_avambraccio2, 0, 180, SERVOMIN_ava2, SERVOMAX_ava2);
      pwm.setPWM(4, 0, pulselengthz); 
     
      //SETTAGGIO INIZIALE BRACCIO - 6 
      new_pos_braccio2 = 180 - pos_braccio2_fin;
      pulselengthzb = map(new_pos_braccio2, 0, 180, SERVOMIN_bra2, SERVOMAX_bra2); 
      pwm.setPWM(6, 0, pulselengthzb); 

      //SETTAGGIO INIZIALE SPALLA - 7
      new_pos_spalla2 = 180 - pos_spalla2_fin;
      pulselengthzc = map(new_pos_spalla2, 0, 180, SERVOMIN_spa2, SERVOMAX_spa2);
      pwm.setPWM(7, 0, pulselengthzc);         

 
      //************************************************************************************** ZAMPA 3 SETUP
      //SETTAGGIO INIZIALE AVAMBRACCIO - 8
      pos_avambraccio3_fin = pos_avambraccio3_fin + 0;// e' mirror quindi + 10
      new_pos_avambraccio3 = 180 - pos_avambraccio3_fin;
      pulselengthz = map(new_pos_avambraccio3, 0, 180, SERVOMIN_ava3, SERVOMAX_ava3);
      pwm.setPWM(8, 0, pulselengthz); 
   
      //SETTAGGIO INIZIALE BRACCIO - 9
      pulselengthzb = map(pos_braccio3_fin, 0, 180, SERVOMIN_bra3, SERVOMAX_bra3);
      pwm.setPWM(9, 0, pulselengthzb);  

      //SETTAGGIO INIZIALE SPALLA - 10
      //new_pos_spalla3 = 180 - pos_spalla3_fin;
      //pulselengthzc = map(new_pos_spalla3, 0, 180, SERVOMIN_spa3, SERVOMAX_spa3);
      //pwm.setPWM(10, 0, pulselengthzc);      
      
 
      //************************************************************************************** ZAMPA 4 SETUP
      //SETTAGGIO INIZIALE AVAMBRACCIO - 12
      new_pos_avambraccio4 = pos_avambraccio4_fin; 
      pulselengthz = map(new_pos_avambraccio4, 0, 180, SERVOMIN_ava4, SERVOMAX_ava4);
      pwm.setPWM(12, 0, pulselengthz); 
    
      //SETTAGGIO INIZIALE BRACCIO - 13
      new_pos_braccio4 = 180 - pos_braccio4_fin;      
      pulselengthzb = map(new_pos_braccio4, 0, 180, SERVOMIN_bra4, SERVOMAX_bra4);
      pwm.setPWM(13, 0, pulselengthzb);  
     
      //SETTAGGIO INIZIALE SPALLA - 15
      //new_pos_spalla4 = pos_spalla4_fin;      
      //pulselengthzc = map(new_pos_spalla4, 0, 180, SERVOMIN_spa4, SERVOMAX_spa4);
      //pwm.setPWM(15, 0, pulselengthzc);  





} //fine else risposta_direzione = "GYRO OK";
}//movimento_on = 1  
}//fine if (start == 7)  
   
    //}//fine else if (mpuIntStatus & 0x02) {     
    //wdt_reset(); WATCHDOG
    //delay(10);
    //mpu.resetFIFO();
 }//fine LOOP
//______________________________________________________________________________________________________





void equilibrio_per_ricarica()
{
    //fase di inclinazione X ---------> TARGET_X 
    if(target_x != target_x_appo)//12
    {
       if(target_x < target_x_appo)
       {
         timer_raggiung_target_x = timer_raggiung_target_x + 1;
         if(timer_raggiung_target_x == tempo_attesa_appo) 
         {
          target_x = target_x + 1;  
          timer_raggiung_target_x = 0;
         }
       }       
       if(target_x > target_x_appo)
       {
         timer_raggiung_target_x = timer_raggiung_target_x + 1;
         if(timer_raggiung_target_x == tempo_attesa_appo) 
         {
          target_x = target_x - 1;  
          timer_raggiung_target_x = 0;
         }
       }             
    } //fine if(target_x != target_x_appo)



    //fase di inclinazione Y ---------> TARGET_Y 
    if(target_y != target_y_appo)
    {
       if(target_y < target_y_appo)
       {
         timer_raggiung_target_y = timer_raggiung_target_y + 1;
         if(timer_raggiung_target_y == tempo_attesa_appo) 
         {
          target_y = target_y + 1;  
          timer_raggiung_target_y = 0;
         }
       }       
       if(target_y > target_y_appo)
       {
         timer_raggiung_target_y = timer_raggiung_target_y + 1;
         if(timer_raggiung_target_y == tempo_attesa_appo) 
         {
          target_y = target_y - 1;  
          timer_raggiung_target_y = 0;
         }
       }             
    } //fine if(target_y != target_x_appo)

}




//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void ricarica_zampa1() //parte solo se c'e' ok_zampa2 = 1!
{
  
  //movimento_zampa_new1(int servo_num, int st_speed, int target)



    // ********************************************************************************RICARICA ZAMPA 1 
    // ------------------------------------ AVAMBRACCIO 1 SI CHIUDE
    if (moto_zampa1 == 1)
    {       
      movimento_zampa_new1(1,8,40);//40 sollevamento_avambr_ant_ricarica   vel8
      
      if (pos_avambraccio1_fin == 40)
      {             
        timer_ricarica1 = 0;          
        moto_zampa1 = 2;   
      }                 
    }  
      
    // ------------------------------------- BRACCIO 1 AVANZA      
    if (moto_zampa1 == 2)
    { 
      movimento_zampa_new1(2,8,65); //80 bra<ccio    vel 8

      //l' avambraccio si apre quando il braccio supera apertura 60
      if (pos_braccio1_fin <= 65)
      {
       movimento_zampa_new1(1,4,140);      //  pos avambraccio continua fino a 140! bisogna fermarlo! 
      }

      //CHECK MULTIPLO SENSORE TATTILE
      if (fsrReading1 >= 300 && pos_braccio1_fin <= 65) //300
      {
       fsrReading1_check = fsrReading1_check + 1;       
      }

    
      //FINE RICARICA ZAMPA 1
      if ((pos_braccio1_fin == 65 && pos_avambraccio1_fin == 140) || (fsrReading1_check == 2)) //era 3
      {
          //movimento_zampa_C(0,1,1,1,2,pos_avambraccio1_fin);        
    
          timer_ricarica1 = 0; 
          ok_zampa1 = 1; //finita ricarica           
          moto_zampa1 = 3;  
          fsrReading1_check = 0; 
      }         
    }        
}



void ricarica_zampa2() //parte solo se c'e' ok_zampa2 = 1!
{
  
  //void movimento_zampa_B(int flag_movimento_zampa, int zampa_num, int servo_num, int attesa_servo , int st_speed, int target) 

    // ********************************************************************************RICARICA ZAMPA 2 
    // ------------------------------------ BRACCIO SI ALZA e AVAMBRACCIO 2 SI CHIUDE 
    if (moto_zampa2 == 1)
    {       
      //pausa_spinta_zampe = 1;//STOP TIMER CAMMINATA!      
      
      //movimento_zampa_fast(0,2,1,40);//30 sollevamento_avambr_ant_ricarica
      //movimento_zampa(0,2,1,1,40);//40 sollevamento_avambr_ant_ricarica    

      movimento_zampa_new2(2,8,130);//
      movimento_zampa_new2(1,8,20);//40 sollevamento_avambr_ant_ricarica   vel8
      
      if (pos_avambraccio2_fin == 20 && pos_braccio2_fin == 130)
      {
       //timer_ricarica2 = timer_ricarica2 + 1;
       //if (timer_ricarica2 > 10) //10
       //{               
        timer_ricarica2 = 0;          
        moto_zampa2 = 2;
       //} 
      }                 
    }  
      
      //ricarica zampa 3 e 4 devono spingere con gli avambracci! (aprendoli!)
      
    // ------------------------------------- BRACCIO 2 AVANZA      
    if (moto_zampa2 == 2)
    { 
      //movimento_zampa_fast(0,2,2,65); //parte da 110 a 95
      //movimento_zampa(0,2,2,1,20); //50
      movimento_zampa_new2(2,8,65); //80 bra<ccio    vel 8

      //l' avambraccio si apre quando il braccio supera apertura 60
      if (pos_braccio2_fin <= 65)
      {
       movimento_zampa_new2(1,4,140);      // 85 pos avambraccio continua fino a 140! bisogna fermarlo! 
      }


      //CHECK MULTIPLO SENSORE TATTILE
      if (fsrReading2 >= 300)
      {
       fsrReading2_check = fsrReading2_check + 1;       
      }
      
      //FINE RICARICA ZAMPA 2
      if ((pos_braccio2_fin == 65 && pos_avambraccio2_fin == 140) || (fsrReading2_check == 2)) //era 3
      {
          //movimento_zampa_C(0,1,1,1,2,pos_avambraccio1_fin);        
    
          timer_ricarica2 = 0; 
          ok_zampa2 = 1; //finita ricarica           
          moto_zampa2 = 3; 
          fsrReading2_check = 0;
      }         
    }        
}




void ricarica_zampa3() //parte solo se c'e' ok_zampa3 = 1!
{
    // ********************************************************************************RICARICA ZAMPA 3   
    // --------------------------- AVAMBRACCIO 3 SI CHIUDE + BRACCIO 3 SI ALZA (AVANZA)
    if (moto_zampa3 == 1)
    {   
      
      movimento_zampa_new3(1,8,20);//20 avambraccio                                     vel8
      movimento_zampa_new3(2,4,10);// braccio              vel8
          
      if (pos_avambraccio3_fin == 20 && pos_braccio3_fin == 10)
      {             
        timer_ricarica3 = 0;          
        moto_zampa3 = 2;                      
      }                 
    } //fine if (moto_zampa3 == 1) 
  
  
    // ------------------------------------- AVAMBRACCIO 3 SI APRE    E BRACCIO SCENDE!!
    if (moto_zampa3 == 2)
    { 
      //timer_ricarica3 = timer_ricarica3 + 1;
      
       movimento_zampa_new3(1,8,130); //avambraccio 3 si apre fino a 55                      vel8 

      //il braccio si abbassa quando l'avambraccio supera apertura 35
      if (pos_avambraccio3_fin >= 50)
      {
       //140=MAX(braccio tutto dietro
       movimento_zampa_new3(2,4,55); //braccio   40  
      }

      //CHECK MULTIPLO SENSORE TATTILE
      if (fsrReading3 >= 300)
      {
       fsrReading3_check = fsrReading3_check + 1;       
      }

      //FINE RICARICA ZAMPA 3
      if ((pos_avambraccio3_fin == 130 && pos_braccio3_fin == 55) || (fsrReading3_check == 2)) //era 3
      {   
        //movimento_zampa_C(0,3,1,1,2,pos_avambraccio4_fin);           
        timer_ricarica3 = 0; 
        ok_zampa3 = 1; //finita ricarica           
        moto_zampa3 = 3;      
        fsrReading3_check = 0;    
      }        
    } //fine if (moto_zampa4 == 2)       
  
} //fine void ricarica_zampa4()






void ricarica_zampa4() //parte solo se c'e' ok_zampa4 = 1!
{
    // ********************************************************************************RICARICA ZAMPA 4   
    // --------------------------- AVAMBRACCIO 4 SI CHIUDE + BRACCIO 4 SI ALZA (AVANZA)
    if (moto_zampa4 == 1)
    {   
      
      movimento_zampa_new4(1,8,20);//20 avambraccio                                     vel4
      movimento_zampa_new4(2,4,10);// braccio              vel2
          
      if (pos_avambraccio4_fin == 20 && pos_braccio4_fin == 10)
      {             
        timer_ricarica4 = 0;          
        moto_zampa4 = 2;                      
      }                 
    } //fine if (moto_zampa4 == 1) 
  

  
    // ------------------------------------- AVAMBRACCIO 4 SI APRE    E BRACCIO SCENDE!!
    if (moto_zampa4 == 2)
    { 
      timer_ricarica4 = timer_ricarica4 + 1;
      

      movimento_zampa_new4(1,8,130); //avambraccio 4 si apre fino a 70                      vel2

      //il braccio si abbassa quando l'avambraccio supera apertura tot
      if (pos_avambraccio4_fin >= 50)                             //45
      {
        
       //  140=MAX(braccio tutto dietro
       movimento_zampa_new4(2,4,55); //braccio                   45  
      }

      //CHECK MULTIPLO SENSORE TATTILE
      if (fsrReading4 >= 300)
      {
       fsrReading4_check = fsrReading4_check + 1;       
      }

      //FINE RICARICA ZAMPA 4
      if ((pos_avambraccio4_fin == 130 && pos_braccio4_fin == 55) || (fsrReading4_check == 2)) // && timer_ricarica4 > 5)//timer_ricarica = 5
      {   
        //movimento_zampa_C(0,4,1,1,2,pos_avambraccio4_fin);           
        timer_ricarica4 = 0; 
        ok_zampa4 = 1; //finita ricarica           
        moto_zampa4 = 3;  
        fsrReading4_check = 0;        
      }        
    } //fine if (moto_zampa4 == 2)       
  
} //fine void ricarica_zampa4()




void movimento_zampa_new1(int servo_num, int st_speed, int target)
{

  /////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////// ZAMPA 1  
  /////////////////////////////////////////////////////////////////////////////////////////////
  if (servo_num == 1) //AVAMBRACCIO
  {
     //timer_ava2 = timer_ava2 + 1;     
     //----------------------------------------------------------------SE POSIZIONE ATTUALE < target 
     if (pos_avambraccio1_fin < target) 
     {
      //timer_ava2 = 0; 
      pos_avambraccio1_fin = pos_avambraccio1_fin + st_speed; 
              
        //CHECK POSIZIONE 
        if(pos_avambraccio1_fin >= target)
        {
          pos_avambraccio1_fin = target;
        }                 
     }      
     //----------------------------------------------------------------SE POSIZIONE ATTUALE > target      
     if (pos_avambraccio1_fin > target) 
     {
      //timer_ava2 = 0; 
      pos_avambraccio1_fin = pos_avambraccio1_fin - st_speed;
                    
        //CHECK POSIZIONE 
        if(pos_avambraccio1_fin <= target)
        {
          pos_avambraccio1_fin = target;          
        }       
     }           
     //----------------------------------------------------------------SE POSIZIONE ROOT == target      
     if (pos_avambraccio1_fin == target)
     {
      //timer_ava2 = 0; 
      pos_avambraccio1_fin = target;
     }      
  }

  if (servo_num == 2) //BRACCIO
  {
     //timer_ava2 = timer_ava2 + 1;
     
     //----------------------------------------------------------------SE POSIZIONE ATTUALE < target 
     if (pos_braccio1_fin < target) 
     {
      //timer_ava2 = 0; 
      pos_braccio1_fin = pos_braccio1_fin + st_speed; 
              
        //CHECK POSIZIONE 
        if(pos_braccio1_fin >= target)
        {
          pos_braccio1_fin = target;
        }                 
     }      
     //----------------------------------------------------------------SE POSIZIONE ATTUALE > target      
     if (pos_braccio1_fin > target) 
     {
      //timer_ava2 = 0; 
      pos_braccio1_fin = pos_braccio1_fin - st_speed;
                    
        //CHECK POSIZIONE 
        if(pos_braccio1_fin <= target)
        {
          pos_braccio1_fin = target;          
        }       
     }           
     //----------------------------------------------------------------SE POSIZIONE ROOT == target      
     if (pos_braccio1_fin == target)
     {
      //timer_ava2 = 0; 
      pos_braccio1_fin = target;
     }      
  } 
}


void movimento_zampa_new2(int servo_num, int st_speed, int target)
{
  /////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////// ZAMPA 2  
  /////////////////////////////////////////////////////////////////////////////////////////////
  if (servo_num == 1) //AVAMBRACCIO
  {
     //timer_ava2 = timer_ava2 + 1;     
     //----------------------------------------------------------------SE POSIZIONE ATTUALE < target 
     if (pos_avambraccio2_fin < target) 
     {
      //timer_ava2 = 0; 
      pos_avambraccio2_fin = pos_avambraccio2_fin + st_speed; 
              
        //CHECK POSIZIONE 
        if(pos_avambraccio2_fin >= target)
        {
          pos_avambraccio2_fin = target;
        }                 
     }      
     //----------------------------------------------------------------SE POSIZIONE ATTUALE > target      
     if (pos_avambraccio2_fin > target) 
     {
      //timer_ava2 = 0; 
      pos_avambraccio2_fin = pos_avambraccio2_fin - st_speed;
                    
        //CHECK POSIZIONE 
        if(pos_avambraccio2_fin <= target)
        {
          pos_avambraccio2_fin = target;          
        }       
     }           
     //----------------------------------------------------------------SE POSIZIONE ROOT == target      
     if (pos_avambraccio2_fin == target)
     {
      //timer_ava2 = 0; 
      pos_avambraccio2_fin = target;
     }      
  }

  if (servo_num == 2) //BRACCIO
  {
     //timer_ava2 = timer_ava2 + 1;
     
     //----------------------------------------------------------------SE POSIZIONE ATTUALE < target 
     if (pos_braccio2_fin < target) 
     {
      //timer_ava2 = 0; 
      pos_braccio2_fin = pos_braccio2_fin + st_speed; 
              
        //CHECK POSIZIONE 
        if(pos_braccio2_fin >= target)
        {
          pos_braccio2_fin = target;
        }                 
     }      
     //----------------------------------------------------------------SE POSIZIONE ATTUALE > target      
     if (pos_braccio2_fin > target) 
     {
      //timer_ava2 = 0; 
      pos_braccio2_fin = pos_braccio2_fin - st_speed;
                    
        //CHECK POSIZIONE 
        if(pos_braccio2_fin <= target)
        {
          pos_braccio2_fin = target;          
        }       
     }           
     //----------------------------------------------------------------SE POSIZIONE ROOT == target      
     if (pos_braccio2_fin == target)
     {
      //timer_ava2 = 0; 
      pos_braccio2_fin = target;
     }      
  } 
}



void movimento_zampa_new3(int servo_num, int st_speed, int target)
{
  /////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////// ZAMPA 3  
  /////////////////////////////////////////////////////////////////////////////////////////////
  if (servo_num == 1) //AVAMBRACCIO
  {
     //timer_ava2 = timer_ava2 + 1;     
     //----------------------------------------------------------------SE POSIZIONE ATTUALE < target 
     if (pos_avambraccio3_fin < target) 
     {
      //timer_ava2 = 0; 
      pos_avambraccio3_fin = pos_avambraccio3_fin + st_speed; 
              
        //CHECK POSIZIONE 
        if(pos_avambraccio3_fin >= target)
        {
          pos_avambraccio3_fin = target;
        }                 
     }      
     //----------------------------------------------------------------SE POSIZIONE ATTUALE > target      
     if (pos_avambraccio3_fin > target) 
     {
      //timer_ava2 = 0; 
      pos_avambraccio3_fin = pos_avambraccio3_fin - st_speed;
                    
        //CHECK POSIZIONE 
        if(pos_avambraccio3_fin <= target)
        {
          pos_avambraccio3_fin = target;          
        }       
     }           
     //----------------------------------------------------------------SE POSIZIONE ROOT == target      
     if (pos_avambraccio3_fin == target)
     {
      //timer_ava2 = 0; 
      pos_avambraccio3_fin = target;
     }      
  }

  if (servo_num == 2) //BRACCIO
  {
     //timer_ava2 = timer_ava2 + 1;
     
     //----------------------------------------------------------------SE POSIZIONE ATTUALE < target 
     if (pos_braccio3_fin < target) 
     {
      //timer_ava2 = 0; 
      pos_braccio3_fin = pos_braccio3_fin + st_speed; 
              
        //CHECK POSIZIONE 
        if(pos_braccio3_fin >= target)
        {
          pos_braccio3_fin = target;
        }                 
     }      
     //----------------------------------------------------------------SE POSIZIONE ATTUALE > target      
     if (pos_braccio3_fin > target) 
     {
      //timer_ava2 = 0; 
      pos_braccio3_fin = pos_braccio3_fin - st_speed;
                    
        //CHECK POSIZIONE 
        if(pos_braccio3_fin <= target)
        {
          pos_braccio3_fin = target;          
        }       
     }           
     //----------------------------------------------------------------SE POSIZIONE ROOT == target      
     if (pos_braccio3_fin == target)
     {
      //timer_ava2 = 0; 
      pos_braccio3_fin = target;
     }      
  } 
}

void movimento_zampa_new4(int servo_num, int st_speed, int target)
{
  /////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////// ZAMPA 4 
  /////////////////////////////////////////////////////////////////////////////////////////////
  if (servo_num == 1) //AVAMBRACCIO
  {
     //timer_ava2 = timer_ava2 + 1;
     
     //----------------------------------------------------------------SE POSIZIONE ATTUALE < target 
     if (pos_avambraccio4_fin < target) 
     {
      //timer_ava2 = 0; 
      pos_avambraccio4_fin = pos_avambraccio4_fin + st_speed; 
              
        //CHECK POSIZIONE 
        if(pos_avambraccio4_fin >= target)
        {
          pos_avambraccio4_fin = target;
        }                 
     }      
     //----------------------------------------------------------------SE POSIZIONE ATTUALE > target      
     if (pos_avambraccio4_fin > target) 
     {
      //timer_ava2 = 0; 
      pos_avambraccio4_fin = pos_avambraccio4_fin - st_speed;
                    
        //CHECK POSIZIONE 
        if(pos_avambraccio4_fin <= target)
        {
          pos_avambraccio4_fin = target;          
        }       
     }           
     //----------------------------------------------------------------SE POSIZIONE ROOT == target      
     if (pos_avambraccio4_fin == target)
     {
      //timer_ava2 = 0; 
      pos_avambraccio4_fin = target;
     }      
  }

  if (servo_num == 2) //BRACCIO
  {
     //timer_ava2 = timer_ava2 + 1;     
     //----------------------------------------------------------------SE POSIZIONE ATTUALE < target 
     if (pos_braccio4_fin < target) 
     {
      //timer_ava2 = 0; 
      pos_braccio4_fin = pos_braccio4_fin + st_speed; 
              
        //CHECK POSIZIONE 
        if(pos_braccio4_fin >= target)
        {
          pos_braccio4_fin = target;
        }                 
     }      
     //----------------------------------------------------------------SE POSIZIONE ATTUALE > target      
     if (pos_braccio4_fin > target) 
     {
      //timer_ava2 = 0; 
      pos_braccio4_fin = pos_braccio4_fin - st_speed;
                    
        //CHECK POSIZIONE 
        if(pos_braccio4_fin <= target)
        {
          pos_braccio4_fin = target;          
        }       
     }           
     //----------------------------------------------------------------SE POSIZIONE ROOT == target      
     if (pos_braccio4_fin == target)
     {
      //timer_ava2 = 0; 
      pos_braccio4_fin = target;
     }      
  } 
}









void equilibrio_spalle()
{

    //----------------------------------------------------------------------    
    //----------------------------------------------------- ASSE X
    //----------------------------------------------------------------------       
         //ROBOT DRITTO
         if (angolo_roll_attuale == 0)
         {          
          //fermi 2 e 4          
          valore_tar_sin2 = 0; 
          valore_tar_sin4 = 0;
          //fermi 1 e 3  
          valore_tar_des1 = 0; 
          valore_tar_des3 = 0;          
         }      
  
         //INCLINAZIONE A SINISTRA
         if (angolo_roll_attuale > 0)
         {  
          //Apertura 2 e 4  
          valore_tar_sin2 = 1; 
          valore_tar_sin4 = 1;
          //Chiusura 1 e 3  
          valore_tar_des1 = -1; 
          valore_tar_des3 = -1;  
          //fase_equilibrio = 1;         
         }
       
         //INCLINAZIONE A DESTRA
         if (angolo_roll_attuale < 0)
         {
          //Apertura 2 e 4  
          valore_tar_sin2 = -1; 
          valore_tar_sin4 = -1;
          //Chiusura 1 e 3  
          valore_tar_des1 = 1; 
          valore_tar_des3 = 1;  
          //fase_equilibrio = 1;          
         }   
        
         
 //movimento_zampa_C(0, 1, 3, 1 , 1, pos_spalla1_fin+valore_tar_sin2) ;
 //movimento_zampa_C(0, 2, 3, 1 , 1, pos_spalla2_fin+valore_tar_sin2) ;
 
 
 
/*
//SPALLA 2
int pos_spalla2 = 60; // 120 =MAX (spalla tutta chiusa)
                      //  0 =MIN (spalla tutta aperta)
                      //  60 = MED 
*/
 
 
 //---

 if (valore_tar_sin2 == 0)
 { 
  movimento_zampa_new1(3, 1, 60); //zampa 1
  movimento_zampa_new3(3, 1, 60); //zampa 3 
  
  movimento_zampa_new2(3, 1, 60); //zampa 2
  movimento_zampa_new4(3, 1, 60); //zampa 4 
 }

 if (valore_tar_sin2 == 1)
 { 
  movimento_zampa_new1(3 , 1, 50); //zampa 1 
  movimento_zampa_new3(3 , 1, 50); //zampa 3 
  
  movimento_zampa_new2(3 , 1, 70); //zampa 2
  movimento_zampa_new4(3 , 1, 70); //zampa 4 
 }
 
 if (valore_tar_sin2 == -1)
 { 
  movimento_zampa_new1(3, 1, 70); //zampa 1 
  movimento_zampa_new3(3, 1, 70); //zampa 3
  
  movimento_zampa_new2(3, 1, 50); //zampa 2
  movimento_zampa_new4(3, 1, 50); //zampa 4   
 } 
 //--- 
}







void spinta_zampe_codice()
{
  crono_spinte = crono_spinte + 1;
  if (crono_spinte == lentezza_spinte)
  {
      //------------------------------ ZAMPA 1
      //SPINTA BRACCI 1,2,3,4 
      //if(pos_braccio1_fin <= 130){
        movimento_zampa_new1(2,3,140); //SPINTA BRACCIO 1!         110
        if (pos_braccio1_fin >= 110) //100
        {
         movimento_zampa_new1(1,2,140); //SPINTA AVAMBRACCIO 1!          
        }  
      //}

      //------------------------------ ZAMPA 2
      //if(pos_braccio2_fin <= 130){        
        movimento_zampa_new2(2,3,140); //SPINTA BRACCIO 2!         110
        if (pos_braccio2_fin >= 110) //100
        {
         movimento_zampa_new2(1,2,140); //SPINTA AVAMBRACCIO 2! 
        }  
      //}  

 
      //------------------------------ ZAMPA 3
      //if(pos_braccio3_fin <= 100){         
        movimento_zampa_new3(2,2,100); //SPINTA BRACCIO 3!         100   
        //se il braccio e' <= di 90, il polpaccio si chiude lentamente

        //se il braccio e' > di 90, il polpaccio si apre velocemente        
        if (pos_braccio3_fin > 30) //90
        {
         movimento_zampa_new3(1,2,140);      
        }
        
        
      //------------------------------ ZAMPA 4       
      //if(pos_braccio4_fin <= 100){   
        movimento_zampa_new4(2,2,100); //SPINTA BRACCIO 4!         100     

        //se il braccio e' > di 90, il polpaccio si apre lentamente
        if (pos_braccio4_fin > 30) //90
        {
         movimento_zampa_new4(1,2,140);      
        }
 
      
   crono_spinte = 0;   
  }//fine if (crono_spinte == lentezza_spinte)   
}


/*
void spinta_zampe_codice()
{
  crono_spinte = crono_spinte + 1;
  if (crono_spinte == lentezza_spinte)
  {
    
      //------------------------------ ZAMPA 1
      //SPINTA BRACCI 1,2,3,4       
        movimento_zampa_new1(2,3,140); //SPINTA BRACCIO 1!         110
        if (pos_braccio1_fin >= 90) //100
        {
          
           //se il peso e' troppo nel piede
           if (fsrReading1 > 400) 
           {
            movimento_zampa_new1(1,2,20); //avambr chiude
           }
           //se il peso e' troppo POCO nel piede
           if (fsrReading1 < 400) 
           {
            movimento_zampa_new1(1,2,140); //avambr  
           }   
         
        }  
     


      //------------------------------ ZAMPA 2     
        movimento_zampa_new2(2,3,140); //SPINTA BRACCIO 2!         110
        if (pos_braccio2_fin >= 90) //100
        {
           
           //se il peso e' troppo nel piede
           if (fsrReading2 > 400) 
           {
            movimento_zampa_new2(1,2,20); //avambr chiude
           }
           //se il peso e' troppo POCO nel piede
           if (fsrReading2 < 400) 
           {
            movimento_zampa_new2(1,2,140); //avambr  
           }      
          
         
        }  
      

 
      //------------------------------ ZAMPA 3        
        movimento_zampa_new3(2,2,100); //SPINTA BRACCIO 3!         100          
        if (pos_braccio3_fin > 30) //90
        {
  
           //se il peso e' troppo nel piede
           if (fsrReading3 > 400) 
           {
            movimento_zampa_new3(1,2,50);   
           }          
           //se il peso e' troppo POCO nel piede
           if (fsrReading3 < 400) 
           {
            movimento_zampa_new3(1,2,140);   
           }           
                     
        }


        
      //------------------------------ ZAMPA 4       
        movimento_zampa_new4(2,2,100); //SPINTA BRACCIO 4!         100     
        if (pos_braccio4_fin > 30) //90
        {
           //se il peso e' troppo nel piede
           if (fsrReading4 > 400) 
           {
            movimento_zampa_new4(1,2,50);   
           }          
           //se il peso e' troppo POCO nel piede
           if (fsrReading4 < 400) 
           {
            movimento_zampa_new4(1,2,140);   
           }            
        }


 
      
   crono_spinte = 0;   
  }//fine if (crono_spinte == lentezza_spinte)   
}
*/



void altezza_zampe()
{  
  
  altezza_robot_zampa1 = pos_avambraccio1_fin;
  altezza_robot_zampa2 = pos_avambraccio2_fin;  
  altezza_robot_zampa3 = pos_avambraccio3_fin; 
  altezza_robot_zampa4 = pos_avambraccio4_fin;   
  altezza_robot_totale = altezza_robot_zampa1 + altezza_robot_zampa2 + altezza_robot_zampa3 + altezza_robot_zampa4;
  
  
  ///////////////////////////////////////////////////////////////////////////////////////////  
  ///////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////// ROUTINE ALTEZZA ROBOT /////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////
//flag_altezza_serial == 1
//modo_altezza4 = 1 (aumenta) 2 (diminuisce)
 
// ------------------------------------------ESECUZIONE ALTEZZA
//if (flag_altezza_serial == 1)
//{   
  /*
    // --------------------------------SETUP
    // pos_avambraccio1_alt_targ e' settato con pulsante piu' e meno
    //flag_altezza_serial = 1 quando do l'input da pulsante
    if (flag_altezza == 0)
    {    
      pos_avambraccio2_alt_targ = pos_avambraccio1_alt_targ;
      pos_avambraccio3_alt_targ = pos_avambraccio1_alt_targ -10;
      pos_avambraccio4_alt_targ = pos_avambraccio1_alt_targ -10; 
      flag_altezza = 1; //fine setup
    }      
  */

/*
    //---ZAMPA 1
    if (pos_avambraccio1_fin < pos_avambraccio1_alt_targ)
    {
     modo_altezza1 = 1; //AUMENTA ALTEZZA ZAMPA
    }
    if (pos_avambraccio1_fin > pos_avambraccio1_alt_targ)
    {
     modo_altezza1 = 2; //DIMINUISCE ALTEZZA ZAMPA
    } 
*/    

/*
    //---ZAMPA 2
    if (pos_avambraccio2_fin < pos_avambraccio2_alt_targ)
    {
     modo_altezza2 = 1; //AUMENTA ALTEZZA ZAMPA
    }
    if (pos_avambraccio2_fin > pos_avambraccio2_alt_targ)
    {
     modo_altezza2 = 2; //DIMINUISCE ALTEZZA ZAMPA
    } 
*/    
/*    
    //---ZAMPA 3
    if (pos_avambraccio3_fin < pos_avambraccio3_alt_targ)
    {
     modo_altezza3 = 1; //AUMENTA ALTEZZA ZAMPA
    }
    if (pos_avambraccio3_fin > pos_avambraccio3_alt_targ)
    {
     modo_altezza3 = 2; //DIMINUISCE ALTEZZA ZAMPA
    } 
    //---ZAMPA 4
    if (pos_avambraccio4_fin < pos_avambraccio4_alt_targ)
    {
     modo_altezza4 = 1; //AUMENTA ALTEZZA ZAMPA
    }
    if (pos_avambraccio4_fin > pos_avambraccio4_alt_targ)
    {
     modo_altezza4 = 2; //DIMINUISCE ALTEZZA ZAMPA
    } 
*/

/*
 pausa_reflex = 1; 
 speed_reflex = 4; //per il braccio sara' 1 mentre per l'avambr sara' speed_reflex * 2 
*/

//void movimento_zampa_C1(int servo_num, int st_speed, int target)
    //----------------------------------------------- ZAMPA 1 ------------------------------------------------


timerk = timerk + 1;
if(timerk == 2)
{

    if(modo_altezza1 == 0)
    {
      
          movimento_zampa_new1(2,1,pos_braccio1_fin);  //zampa1      vel 1  brac
          movimento_zampa_new1(1,2,pos_avambraccio1_fin); //zampa1   vel 2     avambr       
    }  

    if(modo_altezza1 == 1)
    {
          //AUMENTA ALTEZZA ZAMPA, il braccio avanza e L'AVAMBRACCIO SI APRE! E FUNZIONA
          //quindi se la pos_avambraccio1_fin < del targer
          movimento_zampa_new1(2,1,70);  //zampa1      vel 1  brac
          movimento_zampa_new1(1,2,120); //zampa1   vel 2     avambr                             
    } //fine if(modo_altezza == 1)
    
    if(modo_altezza1 == 2)
    {
          //DIMINUISCE ALTEZZA ZAMPA, il braccio indietreggia e L'AVAMBRACCIO SI CHIUDE! E FUNZIONA
          //quindi se la pos_avambraccio2_fin < del targer
          movimento_zampa_new1(2,1,130);  //zampa1      vel 1  brac
          movimento_zampa_new1(1,2,20); //zampa1   vel 2     avambr 
    } //fine if(modo_altezza == 2)




    //----------------------------------------------- ZAMPA 2 ------------------------------------------------
    if(modo_altezza2 == 0)
    {
          movimento_zampa_new2(2,1,pos_braccio2_fin);  //zampa2      vel 1  brac
          movimento_zampa_new2(1,2,pos_avambraccio2_fin); //zampa2   vel 2     avambr       
    }  

    if(modo_altezza2 == 1)
    {
          //AUMENTA ALTEZZA ZAMPA, il braccio avanza e L'AVAMBRACCIO SI APRE! E FUNZIONA
          //quindi se la pos_avambraccio2_fin < del targer
          movimento_zampa_new2(2,1,70);  //zampa2      vel 1  brac
          movimento_zampa_new2(1,2,120); //zampa2   vel 2     avambr                             
    } //fine if(modo_altezza == 1)
    
    if(modo_altezza2 == 2)
    {
          //DIMINUISCE ALTEZZA ZAMPA, il braccio indietreggia e L'AVAMBRACCIO SI CHIUDE! E FUNZIONA
          //quindi se la pos_avambraccio2_fin < del targer
          movimento_zampa_new2(2,1,130);  //zampa2      vel 1  brac
          movimento_zampa_new2(1,2,20); //zampa2   vel 2     avambr 
    } //fine if(modo_altezza == 2)


    //----------------------------------------------- ZAMPA 3 ------------------------------------------------
    if(modo_altezza3 == 0)
    {
          movimento_zampa_new3(2,1,pos_braccio3_fin);  //zampa3     vel 1  brac
          movimento_zampa_new3(1,2,pos_avambraccio3_fin); //zampa3   vel 2     avambr       
    }  

    if(modo_altezza3 == 1)//pos < di targ
    {
          //AUMENTA ALTEZZA ZAMPA, il braccio avanza e L'AVAMBRACCIO SI APRE! E FUNZIONA
          //quindi se la pos_avambraccio2_fin < del targer
          movimento_zampa_new3(2,1,75);  //zampa3      vel 1  brac
          movimento_zampa_new3(1,2,110); //zampa3   vel 2     avambr 
    } //fine if(modo_altezza == 1)
    
    if(modo_altezza3 == 2)
    {
          //DIMINUISCE ALTEZZA ZAMPA, il braccio indietreggia e L'AVAMBRACCIO SI CHIUDE! E FUNZIONA
          //quindi se la pos_avambraccio2_fin < del targer
          movimento_zampa_new3(2,1,20);  //zampa3      vel 1  brac
          movimento_zampa_new3(1,2,10); //zampa3   vel 2     avambr 
    } //fine if(modo_altezza == 2)


    //----------------------------------------------- ZAMPA 4 ------------------------------------------------
    if(modo_altezza4 == 0)
    {
          movimento_zampa_new4(2,1,pos_braccio4_fin);  //zampa4     vel 1  brac
          movimento_zampa_new4(1,2,pos_avambraccio4_fin); //zampa4   vel 2     avambr       
    }  

    if(modo_altezza4 == 1)//pos < di targ
    {
          //AUMENTA ALTEZZA ZAMPA, il braccio avanza e L'AVAMBRACCIO SI APRE! E FUNZIONA
          //quindi se la pos_avambraccio4_fin < del targer
          movimento_zampa_new4(2,1,75);  //zampa4      vel 1  brac
          movimento_zampa_new4(1,2,110); //zampa4   vel 2     avambr 
    } //fine if(modo_altezza == 1)
    
    if(modo_altezza4 == 2)
    {
          //DIMINUISCE ALTEZZA ZAMPA, il braccio indietreggia e L'AVAMBRACCIO SI CHIUDE! E FUNZIONA
          //quindi se la pos_avambraccio2_fin < del targer
          movimento_zampa_new4(2,1,20);  //zampa4      vel 1  brac
          movimento_zampa_new4(1,2,10); //zampa4   vel 2     avambr 
    } //fine if(modo_altezza == 2)

timerk = 0;
}




/*
    //FINE CAMBIO ALTEZZA
    if (fine_altezz_zampa2 == 1 ) //&& fine_altezz_zampa2 == 1 && fine_altezz_zampa3 == 1 && fine_altezz_zampa4 == 1)
    {     
     flag_altezza = 0; 
     flag_altezza_serial = 0; 
     fine_altezz_zampa1 = 0, fine_altezz_zampa2 = 0, fine_altezz_zampa3 = 0, fine_altezz_zampa4 = 0; 
    }
*/

    
//}// fine if (flag_altezza_serial == 1)   
}//fine void altezza_zampe()

void mantenimento_rotta()
{

  
  
  
  
}// fine void mantenimento_rotta()




int freeRam () {

extern int __heap_start, *__brkval;

int v;

return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);

}


