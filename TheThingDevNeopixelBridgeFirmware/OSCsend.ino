// Function to send data to WemosD1Mini sending piezoelectric sensor information

void OSCMsgSend(float countToSend, float audioON)
{
 OSCMessage msgOut("/reply");
 msgOut.add(countToSend);
 msgOut.add(audioON);
 UDP.beginPacket(ipServer,portRemote);
 msgOut.send(UDP);
 UDP.endPacket();
 msgOut.empty();
  
}

