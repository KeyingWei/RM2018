#ifndef _INFRARED_H
#define _INFRARED_H

#define Infrared_1  PDin(12)
#define Infrared_2  PDin(13)
#define Infrared_3  PDin(14)
#define Infrared_5  PHin(10)
#define Infrared_6  PHin(11)



#define SPACE_LIMIT1  PAin(0)
#define SPACE_LIMIT2  PAin(1)
#define SPACE_LIMIT3  PAin(2)
#define SPACE_LIMIT4  PAin(3)
#define SPACE_LIMIT5  PIin(5)
#define SPACE_LIMIT6  PDin(15)
#define SPACE_LIMIT7  PHin(10)


void Infrared_init(void);
void Spaing_Limit_Init(void);

#endif

