#ifndef DigitalIO_h
#define DigitalIO_h

class DigitalIO
{
public:
    void startSetup();
    void setIO_On(int gpio);
    void setIO_Off(int gpio);
    int SCHWALLWASSERPUMPE = 16;
    int UWS = 17;
    int FILTERPUMPE = 18;
    int WP = 19;
};

#endif