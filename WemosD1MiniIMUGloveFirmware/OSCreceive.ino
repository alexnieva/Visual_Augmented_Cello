// Function to receive data from TheThingDev board on the bridge of the cello.
// This board has a piezoelectric sensor that detects vibrations from the cello top. 
// This activates the overall functioning of the system.

void OSCMsgReceive()
{
  OSCMessage msg;
  int size = UDP.parsePacket();
  //Serial.println(size);

  if (size>0){
    //Serial.printf("Received %d bytes from %s, port %d\n", size, UDP.remoteIP().toString().c_str(), UDP.remotePort());

    while (size--){
      msg.fill(UDP.read());
    }
    if (!msg.hasError()){
      msg.route("/reply",piezo);
    }
  }
}

void piezo(OSCMessage &msg, int addrOffset) {
  int L = msg.getDataLength(0);
  //if (DEBUG) {Serial.print("Size of msg: "); Serial.println(L);}
  for (int i=0; i<2; i++) 
  {
    bufferFromPiezo[i] = msg.getFloat(i);
    Serial.println(bufferFromPiezo[i]);
//    OSCMessage msg1("/onoff");
//    msg1.add(bufferFromPiezo[i]);
//    UDP.beginPacket(outIp2,portRemote);
//    msg1.send(UDP);
//    UDP.endPacket();
//    msg1.empty();
  }
}
