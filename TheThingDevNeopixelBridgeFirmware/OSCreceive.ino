// Function to receive data from WemosD1Mini board on the cellist bowing hand.
// This data activates de light patter based on the bowing gestures.

void OSCMsgReceive()
{
  OSCMessage msg;
  int size = UDP.parsePacket();
  //Serial.println(size);

  if (size>0){
    while (size--){
      msg.fill(UDP.read());
    }
    if (!msg.hasError()){
      msg.route("/aknow",glove);
    }
  }
}

void glove(OSCMessage &msg, int addrOffset) {
  int L = msg.getDataLength(0);
  //if (DEBUG) {Serial.print("Size of msg: "); Serial.println(L);}
  for (int i=0; i<8; i++) 
  {
    bufferFromGloveFloat[i] = msg.getFloat(i);
  }
}
