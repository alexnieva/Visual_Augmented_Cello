//
//
//double runningAverage (uint16_t M){
//  #define LM_SIZE 128
//  static uint32_t LM[LM_SIZE];
//  static byte index = 0;
//  static uint32_t sum = 0;
//  static byte count = 0;
//
//  //update
//  sum -= LM[index];
//  LM[index] = M*M;
//  sum += LM[index];
//  index++;
//  index = index%LM_SIZE;
//  if (count<LM_SIZE) count++;
//
//  return sqrt(sum/count);
//}
