
typedef struct reg_1508pl10   //flags //Init_pins_for_DDS   pins_DDS;  // объявляю структуру 
{
           char            Name[5];  //имя структуры
  unsigned char       T_Amp_pres:1; //Тестовый режим "0"
  unsigned char          T_Fop_o:1; //Разряд управления Обход усилителя кварцевого осциллятора 0 – стандартный режим 1 – обход

  unsigned char             T_vr:1;   //Тестовый режим
  unsigned char             T_Pd:1;   //Тестовый режим
  unsigned char          T_Del_m:1;   //Тестовый режим

  unsigned char               Kz:1; //Разряды управления Разряд управления «Z–состоянием» фазового детектора. 0 – режим выключен 1 – режим включен
  unsigned char               FD:2; //Соответствие кода FD1, FD0 и ширины импульса «мертвой зоны»: 00 - 17нс|01 - 29 | 10 - 52| 11 - 100 нс
 
  unsigned char               Kb:1; //Разряд управления функцией вывода Fdiv2(14) 0 – вывод работает как выход 1 – вывод работает как вход
  unsigned char               Kp:1; // Разряд управления полярностью выводов ФД. 0 – прямая полярность 1 – обратная полярность

  unsigned char             Kok:1; // Разряд управления типом выводов. 0 – открытый сток (см. назначение выводов) 1 – КМОП выход
  unsigned char             Kpo:1; // Разряд управления функцией вывода Fdiv3(9). 0 – вывод работает как выход 1 – вывод работает как вход
  unsigned char             Klt:1; // Разряд управления источником тока. 0 – источник тока на выводах ФД выключен 1 – источник тока на выводах ФД включен.
                                   // В этом режиме ФД имеет один выход, а к выходу Fd2 (Down) должен быть подключен внешний токозадающий резистор

  unsigned char            KREF4:1; //коэффициента деления опорного канала
  unsigned char            KREF3:1; //коэффициента деления опорного канала

  unsigned char            KREF2:1; //коэффициента деления опорного канала
  unsigned char            KREF1:1; //коэффициента деления опорного канала
  unsigned char            KREF0:1; //коэффициента деления опорного канала

  unsigned int             Kosn16_19:4; //коэффициента деления основного канала старшие биты
  unsigned int             Kosn00_15:16; //коэффициента деления основного канала младшие биты


} reg_1508pl10;