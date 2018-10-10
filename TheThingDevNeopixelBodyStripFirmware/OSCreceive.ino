

void OSCMsgReceive()
{
  OSCMessage msg;
  int size = UDP.parsePacket();
  //Serial.println(size);

  if (size>0){
//    if (size == 16){
//      while (size--){
//        msg.fill(UDP.read());
//      }
//      if (!msg.hasError()){
//        msg.route("/onoff",onoff);
//      }
//    }
//    else
//    {
      while (size--){
        msg.fill(UDP.read());
      }
      if (!msg.hasError()){
        msg.route("/aknow",glove);
      }
    }
//  }
}

void glove(OSCMessage &msg, int addrOffset) {
  int L = msg.getDataLength(0);
  //if (DEBUG) {Serial.print("Size of msg: "); Serial.println(L);}
  for (int i=0; i<8; i++) 
  {
    bufferFromGloveFloat[i] = msg.getFloat(i);
    //Serial.println(bufferFromGloveFloat[i]);
  }
}

//void onoff(OSCMessage &msg, int addrOffset) {
//  int L = msg.getDataLength(0);
//  //if (DEBUG) {Serial.print("Size of msg: "); Serial.println(L);}
//  for (int i=0; i<1; i++) 
//  {
//    bufferFromPiezo[i] = msg.getFloat(i);
//    Serial.print("From Piezo: "); Serial.println(bufferFromPiezo[i]);
//  }
//}

