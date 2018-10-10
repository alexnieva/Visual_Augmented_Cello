void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  dof.readGyro();
  outGyro[0] = dof.calcGyro(dof.gx) - gbias[0];
  outGyro[1] = dof.calcGyro(dof.gy) - gbias[1];
  outGyro[2] = dof.calcGyro(dof.gz) - gbias[2];
  
//  //LPF
//  outGyro[0] = filter(dof.calcGyro(dof.gx),0.7,oldGyro[0]);
//  outGyro[1] = filter(dof.calcGyro(dof.gy),0.7,oldGyro[1]);
//  outGyro[2] = filter(dof.calcGyro(dof.gz),0.7,oldGyro[2]);

  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  //Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(outGyro[0], 2);
  Serial.print(", ");
  Serial.print(outGyro[1], 2);
  Serial.print(", ");
  Serial.println(outGyro[2], 2);
#elif defined PRINT_RAW
  Serial.print(dof.gx);
  Serial.print(", ");
  Serial.print(dof.gy);
  Serial.print(", ");
  Serial.println(dof.gz);
#endif
}

