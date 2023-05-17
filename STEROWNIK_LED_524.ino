//LAMPA LED ZASILANA Z ZASILACZA 12V
//LED LAMP, POWERED BY A 12V POWER SUPPLY
// Martin Szmig  biglu@o2.pl 2023r.
#include <EEPROM.h>
#include <TimerOne.h>
//===  WERSJA, wpisz nazwe pliku:   =========================
String wersja = "STEROWNIK_LED_524";  //Wysyla sie przez Serial Monitor po uruchomieniu a nast. Serial.end()

//===  Piny  ===============================================
#define LightSensor A3          //analogowe wejscie fotorezystora nr 1
#define LightSensorZMIERZCH A0  //analogowe wejscie fotorezystora nr 2, CZUJNIK ŚWIATŁA ZEWNĘTRZNEGO
#define Potencjometr A1         //analogowe wejscie potencjometru
#define Sensor_PIR A2           //cyfrowe wejscie PIR
#define Led_TAPE 10             //PWM
#define LED_RED 5               //LED_INFO _0 - Proporcjonanie, MAX
#define LED_GREEN 6             //LED_INFO _1 - Balans, MIN
#define LED_YELLOW 9            //LED_INFO _2 - MANUAL
#define LED_INFO_pir 3          //LED_INFO _pir
#define LED_INFO_ogranicz 11    //LED_INFO _ogranicz
#define LED_BLUE 13             //LED_INFO _standby
#define LightSenPower 8         //cyfrowe wyjscie jako zasilanie fotorezystora + zasilanie potencjometru

#define Pin_Przycisk1 7  //Tryb_pracy
#define Pin_Przycisk2 4  //SetA, Menu Serial
//===   Nazwy stalych   ======================================
#define AUTOMAT 0
#define MANUAL 1
#define BALANS 1
#define PROPORCJA 0
#define MaxFOTOREZYSTOR 997
#define MaxPOTENCJOMETR 660
unsigned int PWM_PODSWIETL_PO_PIR_prim, PWM_PODSWIETL_PO_PIR = 150;  //183;  //dobrane empirycznie
//===   inne zmienne ===============================================
bool flag_start = true, flag_start_2, flag_PIR = false, flag_break_1, flag_kierunek, led_ogranicz_state;
bool flag_zmierzch, FUNKCJA_ZMIERZCH = false, FUNKCJA_PIR = false, AUTO_;
bool Tryb_pracy, flag_1, flag_2, flag_light_zero, flag_zerowanie_czasu = false;
bool flag_szybki_led1, flag_alarm_temp, flag_alarm_temp_70st;
byte licznik_przycisk_red, licznik_temp;
byte licznik_przycisk_1, licznik_05_sek;
byte licznik_pir, flag_czas_pir, flag_preset;
int lihgt_p_c, lihgt_sr, lihgt_wid, OdczytPotencjometr, licznik_minut, temp, swiatlo_zewnetrzne_raw, swiatlo_zewnetrzne_sr, swiatlo_zewnetrzne;
int prog_zmierzchu, procent_pwm, dzielnik;
float procent_pwm_rob, pwm_rob, potent_rob;
long PWM, PWM_MANUAL, PWM_MANUAL_prim, PWM_prim = 1;
long PWM_pomiar, PWM_led, PWM_led_2, PWM_led_3, PWM_led_4, PWM_led_RED, roznica_PWM;
unsigned long czas_1_min, licznik_05_sek_pir, time_poPIR;
unsigned long czas_przycisk2, czas_do_pir1, czas_do_pir2, czas_swiecenia, czas_calkowity;
unsigned long timeNow, krok_0_1sek, krok_0_5sek, krok_5sek, krok_0_led, czas_PIR, czas_menu;
unsigned long timeNow_prim, time_roboczy1, time_roboczy2 = millis(), time_roboczy3, Maximum_timeNow = 2592000000;  //30 dni //do Real_millis()
String Preset[2] = { "Fabryczne", "Manualne" };
unsigned int RZECZ[9]; // preset_roboczy[13];
//                           0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12
//byte preset_fabr[13] = { 150, 1, 0, 0, 2, 1, 3, 2, 0, 0, 0, 0, 0 };
//                       0, 1, 2,  3,  4, 5, 6, 7, 8
byte preset_fabr[13] = { 0, 1, 0, 150, 2, 3, 2, 0, 0 };
//========================================================================
//---------------- S E T U P ---------------------------------------------
//========================================================================
void setup() {
  //Serial.begin(9600);
  flag_start_2 = true;  //gdy true wyswietli podsumowanie po starcie i Serial.end()
  //      --321098
  DDRB = B00101111;
  PORTB = B00000000;
  //      76543210
  DDRD = B01101000;
  PORTD = B00000000;

  Timer1.initialize(100);  //10 -> 1kH; 100 -> 10kH; 1000 -> 100kH; analogWrite() -> 0.5kHz
  Timer1.pwm(Led_TAPE, 140);
  PWM_prim = 140;
  //pinMode(Led_TAPE, OUTPUT);
  //digitalWrite(Led_TAPE, LOW);
  //pinMode(LightSenPower, OUTPUT); // zasilanie fotorezystora
  //digitalWrite(LightSenPower, LOW);
  //pinMode(LightSensorZMIERZCH, INPUT);
  pinMode(Sensor_PIR, INPUT);  //Sensor_PIR
  pinMode(LightSensor, INPUT);
  pinMode(Potencjometr, INPUT);
  pinMode(Pin_Przycisk1, INPUT_PULLUP);  //przycisk1
  pinMode(Pin_Przycisk2, INPUT_PULLUP);  //przycisk2

  digitalWrite(Led_TAPE, LOW);
  Odczyt_z_EEPROM();
  LightMea();
  czas_PIR = millis() + czas_do_pir2;
  licznik_pir = 2;
  if (RZECZ[8]) pinMode(LightSensorZMIERZCH, INPUT);
}
//========================================================================= 4294967295
//==============   L O O P ================================================
//=========================================================================
void loop() {
  timeNow = Real_millis();
  if (flag_zerowanie_czasu) Zerowanie();
  Przyciski();
  Czujnik_Pir();
  Periodyki();
  Ledy();
  if (FUNKCJA_PIR == false && FUNKCJA_ZMIERZCH == false) {  //AUTO_ = BALANS lub PROPORCJA
    if (Tryb_pracy == AUTOMAT) Tryb_automatyczny();
    else if (Tryb_pracy == MANUAL) Tryb_manualny();
  }
  if (FUNKCJA_PIR) {
    flag_break_1 = false;
    if (RZECZ[3] < PWM_PODSWIETL_PO_PIR) PWM_PODSWIETL_PO_PIR_prim = RZECZ[3];
    else if (RZECZ[3] >= PWM_PODSWIETL_PO_PIR) PWM_PODSWIETL_PO_PIR_prim = PWM_PODSWIETL_PO_PIR;
    //wygaszenie
    while (PWM_prim > PWM_PODSWIETL_PO_PIR_prim + 1) {
      PWM_prim--;
      For_lighting(PWM_prim);
      delay(30);
      if (digitalRead(Sensor_PIR) == HIGH || digitalRead(Pin_Przycisk1) == LOW || digitalRead(Pin_Przycisk2) == LOW) {
        flag_break_1 = true;
        licznik_pir = 2;  //przedluzenie czasu
        break;            //Wybudzenie
      }
    }
    if (flag_break_1 == false) {  //flag_light_zero -> patrz periodyki
      //podswietlanie po pir
      if (lihgt_wid < prog_zmierzchu && flag_light_zero == false) {
        if (PWM_MANUAL_prim != PWM_PODSWIETL_PO_PIR_prim) {
          Timer1.pwm(Led_TAPE, PWM_PODSWIETL_PO_PIR_prim);
          PWM_MANUAL_prim = PWM_PODSWIETL_PO_PIR_prim;
        }
      }
      //wygaszenie podswietlania po pir
      else if (lihgt_wid >= prog_zmierzchu + 20 || flag_light_zero) {
        if (PWM_MANUAL_prim > 0) {
          PWM_MANUAL_prim = 0;
          for (int i = PWM_PODSWIETL_PO_PIR_prim; i > 40; i--) {
            Timer1.pwm(Led_TAPE, i);
            delay(40);
          }
          digitalWrite(Led_TAPE, LOW);
        }
      }
    }
  }
  if (FUNKCJA_ZMIERZCH && FUNKCJA_PIR == false) {
    if (swiatlo_zewnetrzne > prog_zmierzchu + 30) {  //jest widno, wygaszanie swiatla
      flag_zmierzch = false;
      while (PWM_prim > 0) {
        flag_szybki_led1 = true;
        timeNow = Real_millis();
        LightMea();
        Ledy();
        Periodyki();
        flag_break_1 = false;
        PWM_prim--;
        Timer1.pwm(Led_TAPE, PWM_prim);
        delay(30);
        if (digitalRead(Pin_Przycisk1) == LOW || digitalRead(Pin_Przycisk2) == LOW) {
          PWM_prim = PWM;
          break;
        }
      }
      if (flag_break_1 == false) {
        digitalWrite(Led_TAPE, LOW);
        flag_break_1 = true;
      }
      if (flag_szybki_led1) flag_szybki_led1 = false;
    } else if (swiatlo_zewnetrzne < prog_zmierzchu) {  //jest ciemno, zapalanie swiatla
      flag_zmierzch = true;
      if (Tryb_pracy == AUTOMAT) Tryb_automatyczny();
      else if (Tryb_pracy == MANUAL) Tryb_manualny();
    }
  }
}
//========================================================================
//===============   FUNKCJE   ============================================
//========================================================================
//========================================================================
void Zerowanie() {
  czas_1_min = 0;
  czas_przycisk2 = 0;
  krok_0_1sek = 0;
  krok_0_5sek = 0;
  krok_5sek = 0;
  krok_0_led = 0;
  czas_PIR = 0;
  czas_menu = 0;
  flag_zerowanie_czasu = false;
}
unsigned long Real_millis() {
  time_roboczy1 = millis();
  if (time_roboczy1 != time_roboczy2) {  //patrz notatki!
    if (time_roboczy1 >= time_roboczy2) time_roboczy3 = time_roboczy1 - time_roboczy2;
    else time_roboczy3 = time_roboczy1;
    if (timeNow + time_roboczy3 <= Maximum_timeNow) {
      timeNow_prim = timeNow + time_roboczy3;
    } else {
      timeNow_prim = 0;
      flag_zerowanie_czasu = true;
    }
    time_roboczy2 = time_roboczy1;
  }
  return timeNow_prim;
}
void Tryb_automatyczny() {  //(AUTO_ == BALANS lub PROPORCJA  
  LightMea();
  if (timeNow > krok_0_1sek) {
    krok_0_1sek = timeNow + 18;
    if (RZECZ[2]) {
      if (AUTO_ == PROPORCJA && PWM < RZECZ[3]) PWM = RZECZ[3];
      if (PWM > PWM_MANUAL || PWM_MANUAL < RZECZ[3]) PWM = PWM_MANUAL;
    }
    if (PWM > PWM_prim) PWM_prim++;
    else if (PWM < PWM_prim) PWM_prim--;
    For_lighting(PWM_prim);
  }
}
void Tryb_manualny() {
  LightMea();
  if (flag_alarm_temp_70st) Timer1.pwm(Led_TAPE, RZECZ[3]);
  else {
    if (PWM_prim > PWM_MANUAL) {
      for (unsigned int i = PWM_prim; i > PWM_MANUAL; i--) {
        Timer1.pwm(Led_TAPE, i);
        Periodyki();
      }
    } else if (PWM_prim < PWM_MANUAL) {
      for (unsigned int i = PWM_prim; i < PWM_MANUAL; i++) {
        Timer1.pwm(Led_TAPE, i);
        Periodyki();
      }
    }
    if (PWM_MANUAL != PWM_MANUAL_prim) {
      Timer1.pwm(Led_TAPE, PWM_MANUAL);
      PWM_MANUAL_prim = PWM_MANUAL;
    }
    PWM_prim = PWM_MANUAL;
  }
}
void Ledy() {
  if (flag_alarm_temp) {
    if (timeNow > krok_0_led) {
      krok_0_led = timeNow + 200;
      flag_kierunek = !flag_kierunek;
    }
    digitalWrite(LED_RED, flag_kierunek);
    digitalWrite(LED_GREEN, flag_kierunek);
    digitalWrite(LED_YELLOW, flag_kierunek);
    digitalWrite(LED_BLUE, flag_kierunek);
  } else {
    //UTALANIE JASNOSCI LED
    if (timeNow > krok_0_led && FUNKCJA_PIR) {
      krok_0_led = timeNow + 30;
      if (flag_kierunek) PWM_led_2--;
      else PWM_led_2++;
      if (PWM_led_2 >= 25) flag_kierunek = true;
      else if (PWM_led_2 <= 5) flag_kierunek = false;
      PWM_led = PWM_led_2;
      PWM_led_3 = PWM_led_2;
      PWM_led_4 = PWM_led_2;
      PWM_led_RED = PWM_led_2;
    } else if (FUNKCJA_PIR == false) {
      PWM_led = map(lihgt_wid, 0, MaxFOTOREZYSTOR, 0, 100);  //255max jasnosc led_info
      PWM_led_3 = 10;
      if (PWM_led > 100) PWM_led = 100;
      else if (PWM_led < 10) PWM_led = 10;
      if (PWM_led > 57) PWM_led_RED = PWM_led - 50;  // Kompensacja zbyt małego opornika
      else PWM_led_RED = 7;
      if (Tryb_pracy == AUTOMAT) {
        roznica_PWM = PWM_MANUAL - PWM_pomiar;
        if (PWM_prim > 0 && RZECZ[2] && timeNow > krok_0_led && roznica_PWM < 0) {
          krok_0_led = timeNow + 500;
          led_ogranicz_state = !led_ogranicz_state;
          if (led_ogranicz_state) PWM_led_4 = 10;
          else PWM_led_4 = 20;
        } else if (roznica_PWM > 0) PWM_led_4 = 10;
      }
      if (Tryb_pracy == MANUAL || RZECZ[2] == false) PWM_led_4 = 0;
    }
    //REALIZACJA JASNOSCI LED
    if (Tryb_pracy == AUTOMAT) {
      if (AUTO_ == PROPORCJA) {
        analogWrite(LED_RED, PWM_led_RED);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, LOW);
      } else if (AUTO_ == BALANS) {
        digitalWrite(LED_RED, LOW);
        analogWrite(LED_GREEN, PWM_led);
        digitalWrite(LED_YELLOW, LOW);
      }
    } else if (Tryb_pracy == MANUAL) {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, LOW);
      analogWrite(LED_YELLOW, PWM_led);
    }
    if (FUNKCJA_ZMIERZCH) {
      if (timeNow > krok_0_led && flag_zmierzch == false) {
        if (flag_szybki_led1) krok_0_led = timeNow + 200;
        else krok_0_led = timeNow + 1500;
        flag_kierunek = !flag_kierunek;
        digitalWrite(LED_BLUE, flag_kierunek);
      } else if (flag_zmierzch) digitalWrite(LED_BLUE, HIGH);
    }
    // info funkcje dodatkowe
    if (FUNKCJA_ZMIERZCH == false && RZECZ[1]) analogWrite(LED_INFO_pir, PWM_led_3);
    else if (FUNKCJA_ZMIERZCH == false && RZECZ[1] == false) digitalWrite(LED_INFO_pir, LOW);
    else if (FUNKCJA_ZMIERZCH && RZECZ[7]) analogWrite(LED_INFO_pir, PWM_led_3);
    else if (FUNKCJA_ZMIERZCH && RZECZ[7] == false) digitalWrite(LED_INFO_pir, LOW);
    analogWrite(LED_INFO_ogranicz, PWM_led_4);  //patrz ustalanie jasnosci
  }
}
void LightMea() {                     //Pomiar swiatla i potencjometru
  //----------------------------------------------------------------
  digitalWrite(LightSenPower, HIGH);  //zasilanie czujnikow ON
  delay(5);
  OdczytPotencjometr = analogRead(Potencjometr);  //potencjometr
  lihgt_p_c = analogRead(LightSensor);            //fotorezystor nr 1
  digitalWrite(LightSenPower, LOW);               //zasilanie czujnikow OFF
  //-----------------------------------------------------------------
  lihgt_sr = Sredni_lihgt_pomiar_czujnika(lihgt_p_c);
  lihgt_wid = FILTR_WIDWLKOWY_JEDEN(lihgt_sr);
  if (lihgt_wid > MaxFOTOREZYSTOR) lihgt_wid = MaxFOTOREZYSTOR;  //empirycznie zmierzona maksymalna wartosc
  PWM = lihgt_wid;
  //Balans/Proporcja
  if (RZECZ[0] == false && PWM > 500) PWM = 1023;  //Proporcja  //buło 650, patrz pomiary w notatkach
  else if (RZECZ[0]) {
    PWM = MaxFOTOREZYSTOR - PWM;  //odwrocenie wartosci do BALANS //było 997
    if (PWM > 650) PWM = 1023;
    if (PWM < 300) PWM = 0;  //było  PWM_PODSWIETL_PO_PIR=150
  }
  if (PWM >RZECZ[3]) PWM_pomiar = PWM; //do migania led ogranicznik
  else PWM_pomiar =RZECZ[3];
  OdczytPotencjometr = FILTR_WIDELKOWY_potencjometr(OdczytPotencjometr);
  PWM_MANUAL = map(OdczytPotencjometr, 0, 1023, 100, MaxPOTENCJOMETR);  //było 660
  if (PWM_MANUAL < 100) PWM_MANUAL = 100;
  else if (PWM_MANUAL > (MaxPOTENCJOMETR - 10)) PWM_MANUAL = 1023;  //patrz filtr widelkowy
}
void Pomiar_Zmierzchu() {
  if (RZECZ[8]) {
    digitalWrite(LightSenPower, HIGH);  //zasilanie czujnikow ON
    delay(5);
    swiatlo_zewnetrzne_raw = analogRead(LightSensorZMIERZCH);
    digitalWrite(LightSenPower, LOW);  //zasilanie czujnikow OFF
    swiatlo_zewnetrzne_sr = Srednie_swiatlo_zewnetrzne(swiatlo_zewnetrzne_raw);
    swiatlo_zewnetrzne = FILTR_WIDWLKOWY_swiatla_zewnetrznego(swiatlo_zewnetrzne_sr);
  } else swiatlo_zewnetrzne = lihgt_wid;
}
int Sredni_lihgt_pomiar_czujnika(int m) {  //swiatlo
  static int _buff[30], sum;
  static byte flag_firstFilter = 0;
  static const byte _buff_max = 30;
  int i;
  if (flag_firstFilter == 0) {
    flag_firstFilter = 1;
    for (i = 0, sum = 0; i < _buff_max; i++) {
      _buff[i] = m;
      sum += _buff[i];
    }
    return m;
  } else {
    sum -= _buff[0];
    for (i = 0; i < (_buff_max - 1); i++) {
      _buff[i] = _buff[i + 1];
    }
    _buff[29] = m;
    sum += _buff[29];
    i = sum / 30;
    return i;
  }
}
int FILTR_WIDWLKOWY_JEDEN(int a_pom) {  //swiatlo
  static int b_min, c_max, d_wid = 8;
  int e_wyj;
  if (a_pom < b_min) {
    b_min = a_pom;
    c_max = a_pom + d_wid;
  } else if (a_pom > c_max) {
    b_min = a_pom - d_wid;
    c_max = a_pom;
  }
  e_wyj = (b_min + c_max) / 2;
  return e_wyj;
}
int FILTR_WIDELKOWY_potencjometr(int pot) {  //potencjometr
  static int b_minP, c_maxP, d_widP = 10;
  int e_wyjP;
  if (pot < b_minP) {
    b_minP = pot;
    c_maxP = pot + d_widP;
  } else if (pot > c_maxP) {
    b_minP = pot - d_widP;
    c_maxP = pot;
  }
  e_wyjP = (b_minP + c_maxP) / 2;
  return e_wyjP;
}
int Srednie_swiatlo_zewnetrzne(int zew) {  //swiatlo nr 2
  static bool flag_firstFilter_nr2 = false;
  static const byte _buff_max_nr2 = 20;  //ilosc probek do sredniej
  static int buff_nr2[_buff_max_nr2], suma_nr2;
  int i2;
  if (flag_firstFilter_nr2 == false) {
    flag_firstFilter_nr2 = true;
    for (i2 = 0, suma_nr2 = 0; i2 < _buff_max_nr2; i2++) {
      buff_nr2[i2] = zew;
      suma_nr2 += buff_nr2[i2];
    }
    return zew;
  } else {
    suma_nr2 -= buff_nr2[0];
    for (i2 = 0; i2 < (_buff_max_nr2 - 1); i2++) {
      buff_nr2[i2] = buff_nr2[i2 + 1];
    }
    buff_nr2[(_buff_max_nr2 - 1)] = zew;
    suma_nr2 += zew;  //buff_nr2[(_buff_max_nr2 - 1)];
    i2 = suma_nr2 / _buff_max_nr2;
    return i2;
  }
}
int FILTR_WIDWLKOWY_swiatla_zewnetrznego(int a2_pom) {  //swiatlo nr 2
  static int b2_min, c2_max, d2_wid = 8;
  int e2_wyj;
  if (a2_pom < b2_min) {
    b2_min = a2_pom;
    c2_max = a2_pom + d2_wid;
  } else if (a2_pom > c2_max) {
    b2_min = a2_pom - d2_wid;
    c2_max = a2_pom;
  }
  e2_wyj = (b2_min + c2_max) / 2;
  return e2_wyj;
}

void Progowanie_zmierzchu() {
  if (RZECZ[6] < 3) prog_zmierzchu = 100 + (RZECZ[6] * 90);  // ciemno, lekko widno, troche widno, widno
  else prog_zmierzchu = 500;                                                   
}
void Wyznaczanie_time_poPIR() {
  if (RZECZ[5] > 7) RZECZ[5] = 1;
  if (RZECZ[5] == 0) time_poPIR = 0;         //rzeczywisty czas 0
  else if (RZECZ[5] == 1) time_poPIR = 60;   //rzeczywisty czas 30 sek
  else if (RZECZ[5] == 2) time_poPIR = 600;  //rzeczywisty czas 5 min
  else if (RZECZ[5] >= 3 && RZECZ[5] < 7) time_poPIR = (RZECZ[5] - 2) * 1800;
  else if (RZECZ[5] == 7) time_poPIR = 172800;
}
void Wyznaczenie_czas_do_pir() {
  if (RZECZ[4] > 5) RZECZ[4] = 5;
  if (RZECZ[4] == 0) {
    czas_do_pir2 = 60000;  // 1min  (czas_do_pir1 = 30sek)
    czas_do_pir1 = 30000;
  } else {
    czas_do_pir2 = RZECZ[4] * 300000;  //flag_czas_do_pir * 5min , czyli od 5min do 25min
    czas_do_pir1 = czas_do_pir2 / 10;
  }
}
double GetTemp(void) {
  unsigned int wADC;
  double t;
  int t_int;
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);
  delay(20);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;
  wADC = ADCW;
  //t = (wADC - 324.31) / 1.22;  //ATmega328p - UNO
  t = (wADC - 324.31) * 1.80;  //ATmega328 - Nano
  //return (t);
  t_int = t;
  t_int = Srednie_GetTemp(t_int);
  return (t_int);
}

int Srednie_GetTemp(int tC) {  //swiatlo nr 2
  static int buff_tC[10], suma_tC;
  static bool flag_firstFilter_tC = false;
  static const byte _buff_max_tC = 10;
  int i;
  if (flag_firstFilter_tC == false) {
    flag_firstFilter_tC = true;
    for (i = 0, suma_tC = 0; i < _buff_max_tC; i++) {
      buff_tC[i] = tC;
      suma_tC += buff_tC[i];
    }
    return tC;
  } else {
    suma_tC -= buff_tC[0];
    for (i = 0; i < (_buff_max_tC - 1); i++) {
      buff_tC[i] = buff_tC[i + 1];
    }
    buff_tC[_buff_max_tC - 1] = tC;
    suma_tC += buff_tC[_buff_max_tC - 1];
    i = suma_tC / _buff_max_tC;
    return i;
  }
}

void menu() {
  Serial.begin(9600);
  Serial.println(F("M  E  N  U"));
  while (licznik_przycisk_red > 6) {
    static unsigned long czas_przycisk1;
    static bool flag_ust_fabr = false;
    timeNow = Real_millis();
    LightMea();
    Ledy();
    if (timeNow > czas_menu) licznik_przycisk_red = 255;  // 255-zapis do eeprom i wyjscie z menu
    if (digitalRead(Pin_Przycisk1) == LOW && timeNow > czas_przycisk1) {
      czas_przycisk1 = timeNow + 500;
      licznik_przycisk_red++;
      if (flag_preset < 1 && licznik_przycisk_red >= 9 && licznik_przycisk_red < 19) licznik_przycisk_red = 19;
      if (licznik_przycisk_red > 21) licznik_przycisk_red = 8;
      if (licznik_przycisk_red == 9) licznik_przycisk_red++;
      if (licznik_przycisk_red < 20) czas_menu = timeNow + 20000;
      else if (licznik_przycisk_red >= 20) czas_menu = timeNow + 300000;  //5min info serwisowe
    }
    if (digitalRead(Pin_Przycisk2) == LOW && timeNow > czas_przycisk2) {
      czas_przycisk2 = timeNow + 500;
      czas_menu = timeNow + 20000;
      if (licznik_przycisk_red == 8) {  //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Ustawienia Fabryczne \\\\\\\\\\\\\\\\\\\\\\\\\\////
        flag_ust_fabr = true;
        byte flag_preset_prim = 0;
        while (flag_ust_fabr) {
          //timeNow = millis();
          timeNow = Real_millis();
          LightMea();
          if (timeNow > czas_menu) {
            if (flag_preset_prim > 0 && flag_preset_prim < 2) {
              flag_preset = flag_preset_prim - 1;
              EEPROM.update(18, flag_preset);
            }
            flag_ust_fabr = false;       //wyjscie z presetow
            licznik_przycisk_red = 255;  // 255-zapis do eeprom i wyjscie z menu
          }
          if (digitalRead(Pin_Przycisk1) == LOW && timeNow > czas_przycisk1) {
            czas_przycisk1 = timeNow + 500;
            czas_menu = timeNow + 20000;
            flag_preset_prim++;
            if (flag_preset_prim > 3) flag_preset_prim = 1;
          }
          if (digitalRead(Pin_Przycisk2) == LOW && timeNow > czas_przycisk2) {
            czas_przycisk2 = timeNow + 500;
            czas_menu = timeNow + 20000;
            if (flag_preset_prim == 1 || flag_preset_prim == 2) {
              flag_preset = flag_preset_prim - 1;
              EEPROM.update(18, flag_preset);
            }
            flag_ust_fabr = false;
            if (flag_preset == 0) licznik_przycisk_red = 9;
            else if (flag_preset == 1) licznik_przycisk_red++;
          }
          if (timeNow > czas_PIR) {
            czas_PIR = timeNow + 500;
            Ledy();
            if (flag_preset_prim == 0) {
              Serial.print(Preset[(flag_preset)]);
              Serial.println(F("  Wcisnij --->TRYB"));
            } else if (flag_preset_prim == 1) {
              Serial.print(Preset[0]);  //Ustawienia fabryczne
              Serial.println(F("  TAK?"));
            } else if (flag_preset_prim == 2) {
              Serial.print(Preset[1]);  //Ustawienia ręczne
              Serial.println(F("  TAK?"));
            } else if (flag_preset_prim == 3) {
              Serial.print(Preset[flag_preset]);
              Serial.println(F("  Wyjscie bez zmian"));
            }
          }
        }
        Odczyt_z_EEPROM();
        czas_przycisk1 = timeNow + 500;
        czas_przycisk2 = timeNow + 500;
      } else if (licznik_przycisk_red == 9) licznik_przycisk_red++;
      else if (licznik_przycisk_red == 10) RZECZ[0] = !RZECZ[0];  //rodzaj automatu, RZECZ[0] - true "Balans", false "Proporcjonalnie"
      else if (licznik_przycisk_red == 11) RZECZ[1] = !RZECZ[1];    //Czujnik ruchu dla światła
      else if (licznik_przycisk_red == 12) RZECZ[2] = !RZECZ[2];    //Ogranicz automat/full auto
      else if (licznik_przycisk_red == 13) {
        if (RZECZ[2]) {
         RZECZ[3] = OdczytPotencjometr;
          if (RZECZ[3] > 355) RZECZ[3] = 355;
          if (RZECZ[3] < 100) RZECZ[3] = 100;
          Timer1.pwm(Led_TAPE, RZECZ[3]);
        } else licznik_przycisk_red--;
      } else if (licznik_przycisk_red == 14) {  //-czas oczekiwanie na ruch
        RZECZ[4]++;
        if (RZECZ[4] > 5) RZECZ[4] = 0;
        Wyznaczenie_czas_do_pir();
      } else if (licznik_przycisk_red == 15) {  //czas świecenia po pir/off
        RZECZ[5]++;
        if (RZECZ[5] > 7) RZECZ[5] = 0;
        Wyznaczanie_time_poPIR();
      } else if (licznik_przycisk_red == 16) {
        RZECZ[6]++;
        if (RZECZ[6] > 3) RZECZ[6] = 0;
        Progowanie_zmierzchu();
      } else if (licznik_przycisk_red == 17) RZECZ[7] = !RZECZ[7];     //PIR w trybi zmierzchowym
      else if (licznik_przycisk_red == 18) RZECZ[8] = !RZECZ[8];       //CZUJNIK SWIATLA ZEWNETRZNEGO
      else if (licznik_przycisk_red >= 19) licznik_przycisk_red = 255; //wyjscie z menu i zapis ustawien
    }
    if (timeNow > czas_PIR) {
      czas_PIR = timeNow + 500;
      if (licznik_przycisk_red == 7) {
        Serial.print(F("MENU"));
        //Serial.println(F("  Wcisniej --->TRYB"));
        Serial.println(F("  Wcisniej --->MODE"));
      } else if (licznik_przycisk_red == 8) {
        Serial.print(F("Ustawienia?   "));
        Serial.println(Preset[(flag_preset)]);
      } else if (licznik_przycisk_red == 9) {
        Serial.print(F("Preset Zmieniony na:   "));
        Serial.println(Preset[(flag_preset)]);
      } else if (licznik_przycisk_red == 10) {
        Serial.print(F("Rodzaj automatu:   "));
        if (RZECZ[0]) Serial.println(F("Balans"));
        else Serial.println(F("Proporcjonalnie"));
      } else if (licznik_przycisk_red == 11) {
        Serial.print(F("Czujnik PIR w trybach auto/manual:   "));
        if (RZECZ[1]) Serial.println(F("TAK"));
        else Serial.println(F("NIE"));
      } else if (licznik_przycisk_red == 12) {
        Serial.print(F("Ograniczanie automatu potencjometrem: "));
        if (RZECZ[2]) {
          Serial.println(F("TAK"));
        } else {
          Serial.println(F("FULL AUTO"));
        }
      } else if (licznik_przycisk_red == 13) {
        Serial.print(F("MINIMALNE swiatlo w automacie "));
        if (RZECZ[2]) {
          pwm_rob = float(RZECZ[3]);
          potent_rob = float(MaxPOTENCJOMETR);
          procent_pwm_rob = (pwm_rob / potent_rob) * 100.00;  // 1023;
          procent_pwm = int(procent_pwm_rob);
          Serial.print(F("PWM minimalne(100 do 355) = "));
          Serial.print(RZECZ[3]);
          Serial.print(F(", "));
          Serial.print(procent_pwm);
          Serial.println(F("%"));
        } else {
          Serial.println(F("FULL AUTO"));
        }
      } else if (licznik_przycisk_red == 14) {
        Serial.print(F("Oczekiwanie na ruch: "));
        if (RZECZ[1]) {
          Serial.print((czas_do_pir2 / 60000));
          Serial.println(F("minut"));
        } else {
          Serial.println(F("PIR Wylaczony"));
        }
      } else if (licznik_przycisk_red == 15) {
        Serial.print(F("Podswietlanie po czasie bezruchu: "));
        if (RZECZ[5]) {
          if (time_poPIR < 120) {
            Serial.print((time_poPIR / 2));
            Serial.println(F("sek"));
          } else if (time_poPIR >= 120 && time_poPIR < 172800) {
            Serial.print((time_poPIR / 120));
            Serial.println(F("minut"));
          } else if (time_poPIR == 172800) Serial.println(F("24 godziny"));
        } else {
          Serial.println(F("NIE"));
        }
      } else if (licznik_przycisk_red == 16) {
        Serial.print(F("Prog zmierzchu: "));
        if (RZECZ[6] == 0) Serial.println(F("CIEMNO"));
        else if (RZECZ[6] == 1) Serial.println(F("Lekko ciemno"));
        else if (RZECZ[6] == 2) Serial.println(F("Troche widno"));
        else if (RZECZ[6] >= 3) Serial.println(F("WIDNO"));
      } else if (licznik_przycisk_red == 17) {
        Serial.print(F("PIR w trybie ZMIERZCH: "));
        if (RZECZ[7]) Serial.println(F("TAK"));
        else Serial.println(F("NIE"));
      } else if (licznik_przycisk_red == 18) {
        Serial.print(F("CZUJNIK SWIATLA ZEWNETRZNEGO: "));
        if (RZECZ[8]) Serial.println(F("TAK"));
        else Serial.println(F("NIE"));
      } else if (licznik_przycisk_red == 19) {
        Serial.println(F("Wyjscie z Menu i zapis ustawien."));
      } else if (licznik_przycisk_red == 20) {
        Serial.print(F("Temp CPU: "));
        Serial.print(temp);
        Serial.println(F(" st.C"));
        Serial.print(F("wersja kodu: "));
        Serial.println(wersja);
      } else if (licznik_przycisk_red == 21) {
        Serial.print(F("Czas swiecenia led[h]: "));
        Serial.println((czas_swiecenia / 60));
        Serial.print(F("Czas calkowity[h]:     "));
        Serial.println((czas_calkowity / 60));
      }
    }
    if ((licznik_przycisk_red != 13) || (licznik_przycisk_red == 13 && RZECZ[2] == false)) {
      krok_0_1sek = timeNow + 2;
      LightMea();
      if (RZECZ[2]) {
        if (PWM < RZECZ[3]) PWM = RZECZ[3];
        if (PWM > PWM_MANUAL || PWM_MANUAL < RZECZ[3]) PWM = PWM_MANUAL;
      }
      if (PWM > PWM_prim) PWM_prim++;
      else if (PWM < PWM_prim) PWM_prim--;
      For_lighting(PWM_prim);
    }
    //=== Periodyki w Menu=============================
    if (timeNow > krok_0_5sek) {  //krok co 0,5sek
      krok_0_5sek = timeNow + 500;
      licznik_05_sek++;
      if (licznik_05_sek > 11) {
        licznik_05_sek = 0;
        temp = GetTemp();
      }
    }
    if (timeNow > czas_1_min) {  //1min
      czas_1_min = timeNow + 60000;
      if (timeNow > 60000) czas_calkowity++;
      if (PWM_prim > 0) czas_swiecenia++;
    }
    //------WYJSCIE Z MENU--------------------------------------------------------------------------
    if (licznik_przycisk_red == 255) {
      if (flag_preset == 1) {
        RZECZ[3] = RZECZ[3] - 100;  //potrzebne bo trzeba ograniczyć do 255
        for (byte k = 0; k < 9; k++) {
          EEPROM.update(k + 1, RZECZ[k]);
        }
        RZECZ[3] = RZECZ[3] + 100;
      }
      EEPROM_Writelong(30, czas_calkowity);
      EEPROM_Writelong(40, czas_swiecenia);
      EEPROM.update(18, flag_preset);
      licznik_przycisk_red = 0;
      czas_PIR = timeNow + czas_do_pir2;  //oddaejemy czas_PIR i przedłużamy
      licznik_pir = 2;
      licznik_przycisk_red = 0;  //wyjscie z petli
      if (RZECZ[0]) AUTO_ = BALANS;
      else AUTO_ = PROPORCJA;
      Podsumowanie_Menu();
      for (byte i = 0; i < 4; i++) {
        Serial.print(wersja);
        Serial.println(F(" - KONIEC  M E N U"));
      }
      Serial.end();
    }
  }
}
void Przyciski() {
  while (digitalRead(Pin_Przycisk1) == LOW) {  //Tryb_pracy i FUNKCJE - FUNKCJA_ZMIERZCH, FUNKCJA_PIR, OGRANICZNIK AUTOMATU
    static unsigned long czas_przycisk1 = 0;
    timeNow = Real_millis();
    if (timeNow > czas_przycisk1) {
      if (licznik_przycisk_1 < 15) czas_przycisk1 = timeNow + 500;
      else if (licznik_przycisk_1 >= 15) czas_przycisk1 = timeNow + 1200;
      licznik_przycisk_1++;
      if (licznik_przycisk_1 == 2) FUNKCJA_ZMIERZCH = false;
      else if (licznik_przycisk_1 == 6) {
        FUNKCJA_ZMIERZCH = true;
      }
      if (licznik_przycisk_1 >= 15) {
        if (licznik_przycisk_1 > 16) licznik_przycisk_1 = 15;
        if (licznik_przycisk_1 == 15) {
          RZECZ[1] = !RZECZ[1];
        } else if (licznik_przycisk_1 == 16) {
          RZECZ[2] = !RZECZ[2];
        }
      }
    }
    digitalWrite(LED_INFO_ogranicz, RZECZ[2]);
    digitalWrite(LED_INFO_pir, RZECZ[1]);
    digitalWrite(LED_BLUE, FUNKCJA_ZMIERZCH);
  }
  if (licznik_przycisk_1 == 1) {
    Tryb_pracy = !Tryb_pracy;
    EEPROM.update(20, Tryb_pracy);
  } else if (licznik_przycisk_1 > 1 && licznik_przycisk_1 < 15) {
    EEPROM.update(14, FUNKCJA_ZMIERZCH);
  } else if (licznik_przycisk_1 >= 15) {
    RZECZ[7] = RZECZ[1];  //czujnik ruchu w auto/manual i zmierzchu
    EEPROM.update(2, RZECZ[1]);
    EEPROM.update(13, RZECZ[7]);
    EEPROM.update(3, RZECZ[2]);
  }
  if (licznik_przycisk_1) licznik_przycisk_1 = 0;
  while (digitalRead(Pin_Przycisk2) == LOW) {  //PROPORCJA/BALANS i menu
    timeNow = Real_millis();
    if (timeNow - czas_przycisk2 >= 500) {
      czas_przycisk2 = timeNow;
      licznik_przycisk_red++;
    }
    digitalWrite(LED_BLUE, FUNKCJA_ZMIERZCH);
    if (licznik_przycisk_red == 6) {
      RZECZ[0] = !RZECZ[0];
      if (RZECZ[0]) AUTO_ = BALANS;
      else AUTO_ = PROPORCJA;
      digitalWrite(LED_RED, (!RZECZ[0]));  // Proporc
      digitalWrite(LED_GREEN, RZECZ[0]);   //Balans
      digitalWrite(LED_YELLOW, LOW);       //MANUAL
      EEPROM.update(12, RZECZ[0]);
      licznik_przycisk_red++;
    }
    if (licznik_przycisk_red > 30) {  //przedłużam czas
      czas_PIR = timeNow;             //pozyczamy czas_PIR
      czas_menu = timeNow + 10000;
      czas_przycisk2 = timeNow + 1000;
      while (digitalRead(Pin_Przycisk2) == LOW) {
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, LOW);
        delay(100);
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_YELLOW, HIGH);
        delay(100);
      }
      //   ======================================================================== M E N U =======
      licznik_przycisk_red = 7;  //potrzebne do menu
      menu();
    }
  }
  if (licznik_przycisk_red) licznik_przycisk_red = 0;
}
void Czujnik_Pir() {
  if (digitalRead(Sensor_PIR) == HIGH) {  //ustalenie czasu aktywnosci
    flag_1 = true;
  } else if (digitalRead(Sensor_PIR) == LOW) {
    flag_1 = false;
    flag_2 = true;
  }
  if (flag_1 && flag_2) {
    licznik_pir++;
    if (licznik_pir > 2) licznik_pir = 2;
    flag_1 = false;
    flag_2 = false;
  }
  if (licznik_pir && licznik_pir < 2) {
    flag_czas_pir = 1;
  } else if (licznik_pir >= 2) {
    flag_czas_pir = 2;
  }
  //wybudzenie i aktywnosc
  if (digitalRead(Sensor_PIR) == HIGH && flag_czas_pir) {
    if (flag_czas_pir == 1) {
      czas_PIR = timeNow + czas_do_pir1;
    } else if (flag_czas_pir == 2) {
      czas_PIR = timeNow + czas_do_pir2;
    }
    flag_PIR = true;
    FUNKCJA_PIR = false;
    PWM_MANUAL_prim = 0;
  }
  //zasypianie
  if (((FUNKCJA_ZMIERZCH && RZECZ[7]) || (FUNKCJA_ZMIERZCH == false && RZECZ[1])) && flag_PIR == true && timeNow > czas_PIR) {
    flag_PIR = false;
    licznik_pir = 0;
    flag_czas_pir = 0;
    FUNKCJA_PIR = true;
    licznik_05_sek_pir = 0;
    flag_light_zero = false;
  }
}
void Periodyki() {
  if (timeNow > krok_0_5sek) {  //krok co 0,5sek
    krok_0_5sek = timeNow + 500;
    Pomiar_Zmierzchu();
    LightMea();
    licznik_05_sek_pir++;
    if (licznik_05_sek_pir >= time_poPIR) flag_light_zero = true;  //patrz f.Wyznaczanie_time_poPIR()
    if (RZECZ[0]) AUTO_ = BALANS;                        //BALANS
    else if (RZECZ[0] == false) AUTO_ = PROPORCJA;       //Proporcja
  }
  if (timeNow > krok_5sek) {  //krok co 5 sek
    krok_5sek = timeNow + 5000;
    licznik_temp++;
    if (licznik_temp > 3) {  //15 sek
      licznik_temp = 0;
      temp = GetTemp();
      if (temp >= 55 && temp <= 70) flag_alarm_temp = true;
      else if (temp > 70) {
        flag_alarm_temp = true;
        flag_alarm_temp_70st = true;
      } else if (temp < 50) {
        flag_alarm_temp_70st = false;
        flag_alarm_temp = false;
      }
    }
    if (flag_start_2 && licznik_temp >= 2) {  //wyświetlanie wersji przez Serial
      temp = GetTemp();
      Serial.begin(9600);
      if (Serial) {
        Serial.print(F("Witaj tu: "));
        Podsumowanie_Menu();
        delay(100);
        Serial.end();
        flag_start_2 = false;
      }
    }
    if (flag_czas_pir == 1) dzielnik = czas_do_pir1 / 1000;
    else if (flag_czas_pir >= 2) dzielnik = czas_do_pir2 / 1000;
  }
  if (timeNow > 60000 && timeNow > czas_1_min) {  //1min
    czas_1_min = timeNow + 60000;
    licznik_minut++;
    czas_calkowity++;
    if (PWM_prim > 0) czas_swiecenia++;
    if (licznik_minut >= 30) {  //30 min
      licznik_minut = 0;
      EEPROM_Writelong(30, czas_calkowity);
      EEPROM_Writelong(40, czas_swiecenia);
    }
  }
}
void EEPROM_Writelong(int address, unsigned long values) {
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (values & 0xFF);
  byte three = ((values >> 8) & 0xFF);
  byte two = ((values >> 16) & 0xFF);
  byte one = ((values >> 24) & 0xFF);
  //Write the 4 bytes into the eeprom memory.
  EEPROM.update(address, four);
  EEPROM.update(address + 1, three);
  EEPROM.update(address + 2, two);
  EEPROM.update(address + 3, one);
}
long EEPROM_Readlong(unsigned long address) {
  //Read the 4 bytes from the eeprom memory.
  unsigned long four = EEPROM.read(address);
  unsigned long three = EEPROM.read(address + 1);
  unsigned long two = EEPROM.read(address + 2);
  unsigned long one = EEPROM.read(address + 3);
  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}
void Preset_przypisanie() {
  //if (flag_preset < 4) {
  if (flag_preset == 0) {
    for (byte i = 0; i < 9; i++) {
      RZECZ[i] = preset_fabr[i];
    }
    RZECZ[3] = RZECZ[3] + 100;
  }
  Progowanie_zmierzchu();
  Wyznaczanie_time_poPIR();   // czas swiecenia min po pir
  Wyznaczenie_czas_do_pir();  //czas oczekiwania na bezruch
}
void For_lighting(unsigned int ifromfor) {
  if (flag_alarm_temp_70st) {
    Timer1.pwm(Led_TAPE, RZECZ[3]);
  } else Timer1.pwm(Led_TAPE, ifromfor);
}
void Odczyt_z_EEPROM() {  //odczytanie z pamięci EEPROM
  flag_preset = EEPROM.read(18);
  if (flag_preset > 1) flag_preset = 1;
  if (flag_preset == 1) {
    for (byte j = 0; j < 9; j++) {
      RZECZ[j] = EEPROM.read(j + 1);
    }
    RZECZ[3] = RZECZ[3] + 100;
  }
  Preset_przypisanie();
  if (flag_start) {
    czas_calkowity = EEPROM_Readlong(30);
    czas_swiecenia = EEPROM_Readlong(40);
    flag_start = false;
  }
  byte roboczy_odczyt_eeprom20 = EEPROM.read(20);
  if (roboczy_odczyt_eeprom20 > 1) Tryb_pracy = true;
  else Tryb_pracy = roboczy_odczyt_eeprom20;
  FUNKCJA_ZMIERZCH = EEPROM.read(14);
  //if (RZECZ[8]) pinMode(LightSensorZMIERZCH, INPUT);
}
void Podsumowanie_Menu() {
  Serial.println(wersja);
  Serial.println(F("Ustawienia:"));
  Serial.print(F("Rodzaj automatu:   "));
  if (RZECZ[0]) Serial.println(F("Balans"));
  else Serial.println(F("Proporcjonalnie"));
  Serial.print(F("Czujnik PIR w trybach auto/manual:   "));
  if (RZECZ[1]) Serial.println(F("TAK"));
  else Serial.println(F("NIE"));
  Serial.print(F("Ograniczanie automatu potencjometrem: "));
  if (RZECZ[2]) {
    Serial.println(F("TAK"));
  } else Serial.println(F("FULL AUTO"));
  Serial.println(F("MINIMALNE swiatlo w automacie:"));
  if (RZECZ[2]) {
    pwm_rob = float(RZECZ[3]);
    potent_rob = float(MaxPOTENCJOMETR);
    procent_pwm_rob = (pwm_rob / potent_rob) * 100.00;
    procent_pwm = int(procent_pwm_rob);
    Serial.print(F("PWM minimalne(100 do 355) = "));
    Serial.print(RZECZ[3]);
    Serial.print(F(", "));
    Serial.print(procent_pwm);
    Serial.println(F("%"));
  } else Serial.println(F("FULL AUTO"));
  Serial.print(F("Oczekiwanie na ruch: "));
  if (RZECZ[1]) {
    Serial.print((czas_do_pir2 / 60000));
    Serial.println(F("minut"));
  } else Serial.println(F("PIR Wylaczony"));
  Serial.print(F("Podswietlanie po czasie bezruchu: "));
  if (RZECZ[5]) {
    if (time_poPIR < 120) {
      Serial.print((time_poPIR / 2));
      Serial.println(F("sek"));
    } else if (time_poPIR >= 120 && time_poPIR < 172800) {
      Serial.print((time_poPIR / 120));
      Serial.println(F("minut"));
    } else if (time_poPIR == 172800) Serial.println(F("24 godziny"));
  } else Serial.println(F("NIE"));
  Serial.print(F("Prog zmierzchu: "));
  if (RZECZ[6] == 0) Serial.println(F("CIEMNO"));
  else if (RZECZ[6] == 1) Serial.println(F("Lekko ciemno"));
  else if (RZECZ[6] == 2) Serial.println(F("Troche widno"));
  else if (RZECZ[6] >= 3) Serial.println(F("WIDNO"));
  Serial.print(F("PIR w trybie ZMIERZCH: "));
  if (RZECZ[7]) Serial.println(F("TAK"));
  else Serial.println(F("NIE"));
  Serial.print(F("CZUJNIK SWIATLA ZEWNETRZNEGO: "));
  if (RZECZ[8]) Serial.println(F("TAK"));
  else Serial.println(F("NIE"));
  Serial.print(F("Czas swiecenia led: "));
  Serial.print((czas_swiecenia / 60));
  Serial.println(F(" godzin"));
  Serial.print(F("Czas calkowity:     "));
  Serial.print((czas_calkowity / 60));
  Serial.println(F(" godzin"));
  Serial.print(F("Temp CPU: "));
  Serial.print(temp);
  Serial.println(F(" st.C"));
  unsigned int address2 = 0, adress2MAX = 44;  //tyle jest wykorzystane  z adress2MAX = EEPROM.length();
  byte value;
  Serial.print(F("Odczyt calego EEPROM, adressMAX= "));
  Serial.println(adress2MAX);
  while (address2 <= adress2MAX) {
    value = EEPROM.read(address2);
    Serial.print(address2);
    Serial.print("\t");
    Serial.println(value, DEC);
    if (address2 == adress2MAX) Serial.println("koniec");
    address2++;
    delay(10);
  }
}
