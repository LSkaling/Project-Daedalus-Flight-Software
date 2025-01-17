#ifndef BUZZER_H
#define BUZZER_H

class Buzzer {
  public:
    enum BeepType {
      SLOW,
      FAST,
      LONG,
      NONE
    };
    Buzzer(int pin);
    void slowBeep();
    void fastBeep();
    void longBeep();
    void noBeep();
    void update();

  private:
    int pin;
    BeepType beepType;
    int lastBeepTime;
    bool buzzerOn;

};

#endif