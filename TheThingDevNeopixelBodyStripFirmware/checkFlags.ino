
void checkFlags() {
    
    
//    if ((bufferFromGloveFloat[4] > 2.0) && (bufferFromGloveFloat[4] < 4.0)&& (FLAG1 == false) && (bufferFromGloveFloat[4] - oldBufferFromGloveFloat[4]) < 0) {
//      FLAG1 = true; //startTimer1 = millis();
//      Serial.print("First Thres: "); Serial.println(FLAG1);
//    }

//    if ((bufferFromGloveFloat[4] > 4.0) && (FLAG1 == false) && (bufferFromGloveFloat[4] - oldBufferFromGloveFloat[4]) < 0) {
//      FLAG1 = true; //startTimer1 = millis();
//      Serial.print("FLAG1 Thres: "); Serial.println(FLAG1);
//    }

    // Acceleration Y axis 
    if ((bufferFromGloveFloat[1] > 1.1) && (FLAG1 == false) && (bufferFromGloveFloat[1] - oldBufferFromGloveFloat[1])<0) {
      FLAG1 = true;
      Serial.print("FLAG1 Thres: "); Serial.println(FLAG1);
    }

    // Acceleration X axis
    else if ((bufferFromGloveFloat[0] > 1.2) && (FLAG2 == false) && (bufferFromGloveFloat[0] - oldBufferFromGloveFloat[0])<0) {
      FLAG2 = true;
      Serial.print("FLAG2 Thres: "); Serial.println(FLAG2);
    }

    // Acceleration Z axis
    else if ((bufferFromGloveFloat[2] > 1.0) && (FLAG3 == false) && (bufferFromGloveFloat[2] - oldBufferFromGloveFloat[2])<0) {
      FLAG3 = true;
      Serial.print("FLAG3 Thres: "); Serial.println(FLAG3);
    }

}
    
//    else if ((bufferFromGloveFloat[4] > 2) && (flag1 == 1)){
//      if ((bufferFromGloveFloat[4] - oldBufferFromGloveFloat[4]) > 0) {
//      timer1 = millis() - startTimer1;
//      if ((bufferFromGloveFloat[4] > 4) && (flag2 == 0)) {
//        flag2 = 1; output2 = 1;
//        Serial.print("Threshold 2:"); Serial.println(bufferFromGloveFloat[4]);
//        if (bufferFromGloveFloat[7]){
//          Stick.Random();
//        }
//        //bufferFromPiezo[0] = 0.0;     
//      }
//      else if ((bufferFromGloveFloat[4] > 4) && (flag2 == 1)){
//        
//      }
//      else if ((bufferFromGloveFloat[4] < 4) && (flag2 == 1)){
//        flag2 = 0; output2 = 0;
//        //flag1 = 0; output1 = 0; 
//        Serial.print("Out-flag2: "); Serial.println(flag2);
//      }
////      if ((bufferFromGloveFloat[4] < 1) && (flag2 == 1)) {
////        flag2 = 0; output2 = 0;
////        Serial.print("Out-flag2: "); Serial.println(flag2);
////      }     
//    }
//    
//    else if ((bufferFromGloveFloat[4] < 2) && (flag1 == 1)){
//      
//    }
//    
//    if ((bufferFromGloveFloat[4] < 1) && (flag1 == 1)){
//      flag1 = 0; output1 = 0; Serial.print("Out-flag1: "); Serial.println(flag1);
//      flag2 = 0; output2 = 0; Serial.print("Out-flag2: "); Serial.println(flag2);
//    }
