#include "Uart2.h"

void Uart2::begin(HardwareSerial* port, uint32_t baud, int8_t rx, int8_t tx){
  _ser=port;
  _ser->begin(baud, SERIAL_8N1, rx, tx);
}
bool Uart2::available(){return _ser->available();}
Data Uart2::read(){
  Data d = {false, 0, 0, 0, 0};
  String s = _ser->readStringUntil('\n');
  int c1 = s.indexOf(','), c2 = s.indexOf(',',c1+1), c3 = s.indexOf(',',c2+1), c4 = s.indexOf(',',c3+1);  
  if (c1 == -1 || c2 == -1 || c3 == -1 || c4 == -1) {return d;}
  String valid = s.substring(0,c1);
  if (valid != "raspicam_send") {return d;}
  else d.valid = true;
  d.id  = s.substring(c1+1,c2).toInt();
  d.x  = s.substring(c2+1,c3).toInt();
  d.y = s.substring(c3+1,c4).toInt();
  d.angle = s.substring(c4+1).toFloat();
  return d;
}