void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.

  dof.readAccel();
  outAccel[0] = dof.calcAccel(dof.ax) - abias[0];
  outAccel[1] = dof.calcAccel(dof.ay) - abias[1];
  outAccel[2] = dof.calcAccel(dof.az) - abias[2];
  
//  //LPF
//  outAccel[0] = filter(outAccel[0],0.7,oldAccel[0]);
//  outAccel[1] = filter(outAccel[1],0.7,oldAccel[1]);
//  outAccel[2] = filter(outAccel[2],0.7,oldAccel[2]);

  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  //Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(dof.calcAccel(dof.ax), 2);
  Serial.print(", ");
  Serial.print(dof.calcAccel(dof.ay), 2);
  Serial.print(", ");
  Serial.println(dof.calcAccel(dof.az), 2);
  Serial.print("Afilt: "); Serial.print(outAccel[0]); Serial.print(", ");
  Serial.print(outAccel[1]); Serial.print(", ");
  Serial.println(outAccel[2]);
  Serial.print("NormAccel: "); Serial.println(normAccel);
  
#elif defined PRINT_RAW 
  Serial.print(dof.ax);
  Serial.print(", ");
  Serial.print(dof.ay);
  Serial.print(", ");
  Serial.println(dof.az);
#endif

}


