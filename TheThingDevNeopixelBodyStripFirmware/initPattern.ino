void initPattern() {
//  if (FLAG1) {
//    Stick.Scanner(white,15);
//    Stick.Pix = 63;
//    Stick.TotalSteps = Stick.Pix*2;
//    return;
//  }

  if (FLAG1) {
    Stick.Random(1);
    return;
  }

  else if (FLAG3) {
    Stick.Fade(white,black,32,60);
    return;
  }
  else if (FLAG2) {
    Stick.Scanner(white,5);
    Stick.Pix = 134;
    Stick.TotalSteps = Stick.Pix*2;
//    Stick.TheaterChase(white,black,1);
    return;
  }
}

