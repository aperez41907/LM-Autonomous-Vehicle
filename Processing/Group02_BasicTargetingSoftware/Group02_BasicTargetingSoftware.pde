 /*
 --------- UCF ECE Group 2: Basic Targeting Software  ----------
 ===============================================================
 --- Based on Open-Source Material, initiated by Bob Rudolph ---
 
 A few keyboard shortcuts:
 press 'b' to set background image
 hold 'r' and click+drag to form a rectangle  "fire-restricted" zone
 press SPACEBAR to toggle manual/autonomous modes
 press SHIFT to toggle arrow-key aiming in manual mode
 */


 //   <===============================================================================================>
 //   Begin custom values - change these camera dimensions to work with your turret
 //   <===============================================================================================>

// changed to 640x480
// normally 320x240
public int camWidth = 320*2;                   //   camera width (pixels),   usually 160*n
public int camHeight = 240*2;                  //   camera height (pixels),  usually 120*n

 //   <===============================================================================================>
 //   End custom values
 //   <===============================================================================================>

boolean PRINT_FRAMERATE = false;     // set to true to print the framerate at the bottom of the IDE window

int[] diffPixelsColor = {
  255, 255, 0
};  // Red, green, blue values (0-255)  to show pixel as marked as target
public int effect = 0;                // Effect

public boolean mirrorCam = false;            //   set true to mirror camera image

public float xMin = 0.0;      //  Actual calibration values are loaded from "settings.txt".
public float xMax = 180.0;    //  If "settings.txt" is borken / unavailable, these defaults are used instead - 
public float yMin = 0.0;      //  otherwise, changing these lines will have no effect on your gun's calibration.
public float yMax = 180.0;    //

import JMyron.*;
import blobDetection.*;
import processing.serial.*;
import java.awt.Frame;
import processing.opengl.*;                     //  see note on OpenGL in void setup() 
import procontroll.*;
import net.java.games.input.*;
import processing.video.*;
import java.awt.Rectangle;

public int minBlobArea = 30;                    //   minimum target size (pixels)
public int tolerance = 100;                     //   sensitivity to motion

public boolean runWithoutArduino = false;
public boolean connecting = false;


public Serial arduinoPort;
JMyron camInput;
BlobDetection target;
Blob blob;
Blob biggestBlob;


int[] Background;
int[] rawImage;
int[] rawBackground;
int[] currFrame;
int[] screenPixels;
public int range = 0;
public int targetX = camWidth/2;
public int targetY = camHeight/2;

int fire = 0;

int[] prevFire = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

float xRatio;
float yRatio;

int possibleX = camWidth/2;
int possibleY = camHeight/2;

int displayX = camWidth/2;
int displayY = camHeight/2;

int oldX = camWidth/2;                // smoothing
int oldY = camHeight/2;               // smoothing
int xdiff;                            // smoothing
int ydiff;                            // smoothing

public float smoothingFactor = 0.8;    // smoothing
public float floatRange = 0;

public boolean activeSmoothing = true;

String strRange;  //LIDAR
String strTargetx;
String strTargety;
String fireSelector;
String scanSelector;


public boolean showDifferentPixels = false;
public boolean showTargetBox = true;
public boolean showCameraView = true;
public boolean firingMode = true;             // true = semi,        false = auto
public boolean safety = true;
public boolean controlMode = false;           // true = autonomous,  false = manual
public boolean scanWhenIdle = true;
public boolean trackingMotion = true;

int idleTime = 10000;          // how many milliseconds to wait until scanning (when in scan mode)
int idleBeginTime = 0;
boolean scan = false;

public String serPortUsed;

int[][] fireRestrictedZones = new int[30][4];
int restrictedZone = 1;
boolean showRestrictedZones = false;

boolean selectingColor = false;
boolean trackingColor = false;
int trackColorTolerance = 100;
int trackColorRed = 255;
int trackColorGreen = 255;
int trackColorBlue = 255;

boolean selectingSafeColor = false;
boolean safeColor = false;
int safeColorMinSize = 500;
int safeColorTolerance = 100;
int safeColorRed = 0;
int safeColorGreen = 255;
int safeColorBlue = 0;

boolean useArrowKeys = false;   // use arrow keys to finely adjust the aiming (in manual mode)

public boolean useInputDevice = false;  // use a joystick or game controller as input (in manual mode)
public boolean inputDeviceIsSetup = false;

public ControllIO controlIO;         // more stuff for using a joystick or game controller for input
public ControllDevice inputDevice;

public ControllButton[] buttons = new ControllButton[30];
public ControllSlider[] sliders = new ControllSlider[10];

public ControllButton[] fire_buttons = new ControllButton[0];
public ControllButton[] preciseAim_buttons = new ControllButton[0];
public ControllButton[] centerGun_buttons = new ControllButton[0];
public ControllButton[] autoOn_buttons = new ControllButton[0];
public ControllButton[] autoOff_buttons = new ControllButton[0];
public ControllButton[] inputToggle_buttons = new ControllButton[0];
public ControllButton[] randomSound_buttons = new ControllButton[0];

public ControllSlider[] pan_sliders = new ControllSlider[0];
public ControllSlider[] tilt_sliders = new ControllSlider[0];
public ControllSlider[] panInvert_sliders = new ControllSlider[0];
public ControllSlider[] tiltInvert_sliders = new ControllSlider[0];


public float xPosition = camWidth/2;
public float yPosition = camHeight/2;


String[] inStringSplit;  // buffer for backup
int controlMode_i, safety_i, firingMode_i, scanWhenIdle_i, trackingMotion_i, trackingColor_i, leadTarget_i, safeColor_i, 
showRestrictedZones_i, showDifferentPixels_i, showTargetBox_i, showCameraView_i, mirrorCam_i, soundEffects_i;



void setup() {
  
  loadSettings();
 
  size(camWidth, camHeight);                  // some users have reported a faster framerate when the code utilizes OpenGL. To try this, comment out this line and uncomment the line below.
  //size(camWidth, camHeight, OPENGL);
  camInput = new JMyron();
  camInput.start(camWidth, camHeight);
  camInput.findGlobs(0);
  camInput.adaptivity(1.01);
  camInput.update();
  currFrame = camInput.image();
  rawImage = camInput.image();
  Background = camInput.image();
  rawBackground = camInput.image();
  screenPixels = camInput.image();
  target = new BlobDetection(camWidth, camHeight);
  target.setThreshold(0.9);
  target.setPosDiscrimination(true);

  retryArduinoConnect();
  
  
  
  xRatio = (camWidth / (xMax - xMin));               // used to allign sights with crosshairs on PC
  yRatio = (camHeight/ (yMax - yMin));
  drawControlPanel();
}

void draw() {

  if (controlMode) {              // autonomous mode
    autonomousMode();
  }
  else if (!controlMode) {        // manual mode
    manualMode();
  }

  if (fire == 1) {
    idleBeginTime = millis();
    scan = false;
  }
  else {
    if (millis() > idleBeginTime + idleTime && controlMode && scanWhenIdle) {
      scan = true;
    }
    else {
      scan = false;
    }
  }

  if (!safety) {
    fire = 0;
  }
  
  strTargetx = "000" + str(targetX);   // make into 3-digit numbers
  strTargetx = strTargetx.substring(strTargetx.length()-3);
  strTargety = "000" + str(targetY);
  strTargety = strTargety.substring(strTargety.length()-3);
  fireSelector = str(0);
  if (firingMode) {
    fireSelector = str(1);
  }
  else {
    fireSelector = str(3);
  }
  if (scan) {
    scanSelector = str(1);
  }
  else {
    scanSelector = str(0);
  }
  //println('a' + strTargetx + strTargety  + str(fire) + fireSelector + scanSelector);
  if (!runWithoutArduino && !connecting) {
    arduinoPort.write('a' + strTargetx + strTargety + str(fire) + fireSelector + scanSelector);   // send to arduino
  }

  if ((keyPressed && key == 't') || showRestrictedZones) {
    for (int col = 0; col <= restrictedZone; col++) {
      noStroke();
      fill(0, 255, 0, 100);
      rect(fireRestrictedZones[col][0], fireRestrictedZones[col][2], fireRestrictedZones[col][1]-fireRestrictedZones[col][0], fireRestrictedZones[col][3]-fireRestrictedZones[col][2]);
    }
  }
  if (selectingColor) {
    stroke(190, 0, 190);
    strokeWeight(2);
    fill(red(currFrame[(mouseY*width)+mouseX]), green(currFrame[(mouseY*width)+mouseX]), blue(currFrame[(mouseY*width)+mouseX]));
    rect(mouseX+2, mouseY+2, 30, 30);
  }

  if (selectingSafeColor) {
    stroke(0, 255, 0);
    strokeWeight(2);
    fill(red(currFrame[(mouseY*width)+mouseX]), green(currFrame[(mouseY*width)+mouseX]), blue(currFrame[(mouseY*width)+mouseX]));
    rect(mouseX+2, mouseY+2, 30, 30);
  }

  for (int i = 9; i >= 1; i--) {
    prevFire[i] = prevFire[i-1];
  }
  prevFire[0] = fire;
  int sumNewFire = prevFire[0] + prevFire[1] + prevFire[2] + prevFire[3] + prevFire[4];
  int sumPrevFire = prevFire[5] + prevFire[6] + prevFire[7] + prevFire[8] + prevFire[9];

  if (fire == 1)
    strokeWeight(3);
  if (fire == 0)
    strokeWeight(1);
  stroke(0, 255, 0);                     //draw [green] crosshairs
  noFill();
  line(displayX, 0, displayX, camHeight);
  line(0, displayY, camWidth, displayY);

  updateControlPanels();
  prevTargetX = targetX;
  prevTargetY = targetY;

}

void autonomousMode() {
  
  if (selectingColor || selectingSafeColor) {
    cursor(1);
  }
  else {
    cursor(0);
  }
  camInput.update();
  rawBackground = camInput.retinaImage();
  rawImage = camInput.image();
  if (mirrorCam) {
    for (int i = 0; i < camWidth*camHeight; i++) {
      int y = floor(i/camWidth);
      int x = i - (y*camWidth);
      x = camWidth-x;
      currFrame[i] = rawImage[(y*camWidth) + x-1];
      Background[i] = rawBackground[(y*camWidth) + x-1];
    }
  }
  else {
    currFrame = rawImage;
    Background = rawBackground;
  }

  loadPixels();
  int safeColorPixelsCounter = 0;

  for (int i = 0; i < camWidth*camHeight; i++) {
    if (showCameraView) {
      pixels[i] = currFrame[i];
    }
    else {
      pixels[i] = color(0, 0, 0);
    }        

    boolean motion = (((abs(red(currFrame[i])-red(Background[i])) + abs(green(currFrame[i])-green(Background[i])) + abs(blue(currFrame[i])-blue(Background[i]))) > (200-tolerance)) && trackingMotion);
    boolean isTrackedColor = (((abs(red(currFrame[i])-trackColorRed) + abs(green(currFrame[i])-trackColorGreen) + abs(blue(currFrame[i])-trackColorBlue)) < trackColorTolerance) && trackingColor);

    boolean isSafeColor = (((abs(red(currFrame[i])-safeColorRed) + abs(green(currFrame[i])-safeColorGreen) + abs(blue(currFrame[i])-safeColorBlue)) < safeColorTolerance) && safeColor);

    if (motion || isTrackedColor) {
      screenPixels[i] = color(255, 255, 255);
      if (showDifferentPixels) {
        if (effect == 0) {
          pixels[i] = color(diffPixelsColor[0], diffPixelsColor[1], diffPixelsColor[2]);
        }
        else if (effect == 1) {
          pixels[i] = color((diffPixelsColor[0] + red(currFrame[i]))/2, (diffPixelsColor[1] + green(currFrame[i]))/2, (diffPixelsColor[2] + blue(currFrame[i]))/2);
        }
        else if (effect == 2) {
          pixels[i] = color(255-red(currFrame[i]), 255-green(currFrame[i]), 255-blue(currFrame[i]));
        }
        else if (effect == 3) {
          pixels[i] = color((diffPixelsColor[0] + (255-red(currFrame[i])))/2, (diffPixelsColor[1] + (255-green(currFrame[i])))/2, (diffPixelsColor[2] + (255-blue(currFrame[i])))/2);
        }
      }
    }
    else {
      screenPixels[i] = color(0, 0, 0);
    }

    if (isSafeColor) {
      safeColorPixelsCounter++;
      pixels[i] = color(0, 255, 0);
      screenPixels[i] = color(0, 0, 0);
    }
  }

  updatePixels();

  int biggestBlobArea = 0;
  target.computeBlobs(screenPixels);
  for (int i = 0; i < target.getBlobNb()-1; i++) {
    blob = target.getBlob(i);
    int blobWidth = int(blob.w*camWidth);
    int blobHeight = int(blob.h*camHeight);
    if (blobWidth*blobHeight >= biggestBlobArea) {
      biggestBlob = target.getBlob(i);
      biggestBlobArea = int(biggestBlob.w*camWidth)*int(biggestBlob.h*camHeight);
    }
  }
  possibleX = 0;
  possibleY = 0;

  if (biggestBlobArea >= minBlobArea) {
    possibleX = int(biggestBlob.x * camWidth);
    possibleY = int(biggestBlob.y * camHeight);
  }

  if (strRange != null) {
    floatRange = parseFloat(strRange);
  }
  if ((biggestBlobArea >= minBlobArea) ) {
    /*----- Select how far target will be (in feet) -----*/
    if(floatRange >= 3 && floatRange < 40) {
    /*----- Select how far target will be (in feet) -----*/
      fire = 1;
    } 
    else {
      fire = 0;
    }
    if (showTargetBox) {
      stroke(255, 50, 50);
      strokeWeight(3);
      fill(255, 50, 50, 150);
      rect(int(biggestBlob.xMin*camWidth), int(biggestBlob.yMin*camHeight), int((biggestBlob.xMax-biggestBlob.xMin)*camWidth), int((biggestBlob.yMax-biggestBlob.yMin)*camHeight));
    }

    anticipation();

    if (activeSmoothing) {
      xdiff = possibleX - oldX; // smoothing
      ydiff = possibleY - oldY; // smoothing
      possibleX = int(oldX + xdiff*(1.0-smoothingFactor)); // smoothing
      possibleY = int(oldY + ydiff*(1.0-smoothingFactor)); // smoothing
    }

    displayX = possibleX;
    displayY = possibleY;
    if (displayX < 0)
      displayX = 0;
    if (displayX > camWidth)
      displayX = camWidth;
    if (displayY < 0)
      displayY = 0;
    if (displayY > camHeight)
      displayY = 0;  
    targetX = int((possibleX/xRatio)+xMin);         
    targetY = int(((camHeight-possibleY)/yRatio)+yMin);
    oldX = possibleX; // smoothing
    oldY = possibleY; // smoothing
  }
  else {
    fire = 0;
  }

  boolean clearOfZones = true;
  for (int col = 0; col <= restrictedZone; col++) {
    if (possibleX > fireRestrictedZones[col][0] && possibleX < fireRestrictedZones[col][1] && possibleY > fireRestrictedZones[col][2] && possibleY < fireRestrictedZones[col][3]) {
      clearOfZones = false;
      fire = 0;
    }
  }


  if (safeColorPixelsCounter > safeColorMinSize && safeColor) {
    noStroke();
    fill(0, 255, 0, 150);
    rect(0, 0, width, height);
    fire = 0;
    targetX = int((xMin+xMax)/2.0);
    targetY = int(yMin);
    displayX = camWidth/2;
    displayY = camHeight;
  }
}

void manualMode() {
//  cursor(1);
  camInput.update();
  rawBackground = camInput.retinaImage();
  rawImage = camInput.image();
  if (mirrorCam) {
    for (int i = 0; i < camWidth*camHeight; i++) {
      int y = floor(i/camWidth);
      int x = i - (y*camWidth);
      x = camWidth-x;
      currFrame[i] = rawImage[(y*camWidth) + x-1];
      Background[i] = rawBackground[(y*camWidth) + x-1];
    }
  }
  else {
    currFrame = rawImage;
    Background = rawBackground;
  }

  loadPixels();                                 //draw camera view to screen
  for (int i = 0; i < camWidth*camHeight; i++) {  //
    pixels[i] = currFrame[i];                   //
  }                                             //
  updatePixels();                               //

  if(inputDeviceIsSetup) {
  }
  if(useInputDevice) {
    if(useArrowKeys) {   // use the arrow keys to aim one pixel at a time
      // use arrow keys to aim - see keyReleased() below
      if(keyPressed) {
        if (keyCode == 37) {                       // left arrow
          xPosition -= 1;
        }
        if (keyCode == 38) {                       // up arrow
          yPosition -= 1;
        }
        if (keyCode == 39) {                       // right arrow
          xPosition += 1;
        }
        if (keyCode == 40) {                       // down arrow
          yPosition += 1;
        }
        fire = 0;
        
      }
    }
  }else{  
    if(useArrowKeys) {   // use the arrow keys to aim one pixel at a time
      // use arrow keys to aim - see keyReleased() below
      if(keyPressed) {
        if (keyCode == 37) {                       // left arrow
          displayX -= 1;
        }
        if (keyCode == 38) {                       // up arrow
         displayY -= 1;
        }
        if (keyCode == 39) {                       // right arrow
          displayX += 1;
        }
        if (keyCode == 40) {                       // down arrow
          displayY += 1;
        }
        fire = 0;
      }
    }else{    
      displayX = mouseX;
      displayY = mouseY;
      // mar 10 added conditional items
      if (strRange != null) {
        floatRange = parseFloat(strRange);
      }
      /*-------- Select how far target will be (in feet) --------*/
      if (mousePressed && (floatRange >= 0 && floatRange < 30)) {
      /*-------- Select how far target will be (in feet) --------*/
        fire = 1;
      }
      else {
        fire = 0;
      }
    }
    
    // calculate position to go to based on mouse position
    targetX = constrain(int((displayX/xRatio)+xMin), 0, 180);
    targetY = constrain(int(((camHeight-displayY)/yRatio)+yMin), 0, 180);
  }
}


void mousePressed() {
  if (keyPressed && key == 'r') {
    print("constraints:" + mouseX + ", " + mouseY);
    fireRestrictedZones[restrictedZone][0] = mouseX;
    fireRestrictedZones[restrictedZone][2] = mouseY;
  }
  if (selectingColor) {
    trackColorRed = int(red(currFrame[(mouseY*width)+mouseX]));
    trackColorBlue = int(blue(currFrame[(mouseY*width)+mouseX]));
    trackColorGreen = int(green(currFrame[(mouseY*width)+mouseX]));
    selectingColor = false;
  }

  if (selectingSafeColor) {
    safeColorRed = int(red(currFrame[(mouseY*width)+mouseX]));
    safeColorBlue = int(blue(currFrame[(mouseY*width)+mouseX]));
    safeColorGreen = int(green(currFrame[(mouseY*width)+mouseX]));
    selectingSafeColor = false;
  }
}

void mouseReleased() {
  if (keyPressed && key == 'r') {
    println(" ... " + mouseX + ", " + mouseY);
    fireRestrictedZones[restrictedZone][1] = mouseX;
    fireRestrictedZones[restrictedZone][3] = mouseY;
    if (fireRestrictedZones[restrictedZone][1]>fireRestrictedZones[restrictedZone][0] && fireRestrictedZones[restrictedZone][1]>fireRestrictedZones[restrictedZone][2]) {
      restrictedZone++;
    }
  }
}

void keyReleased() {

  if (key == ' ') {
    controlMode = !controlMode;
  }

  if (key == 'b') {
    camInput.adapt();
  }
  if (key == 'a') {
    xMin = float(targetX);
    xRatio = (camWidth / (xMax - xMin));                         // used to allign sights with crosshairs on PC
  }
  if (key == 'd') {
    xMax = float(targetX);
    xRatio = (camWidth / (xMax - xMin));                         // used to allign sights with crosshairs on PC
  }
  if (key == 's') {
    yMin = float(targetY);
    yRatio = (camHeight/ (yMax - yMin));                         //
  }
  if (key == 'w') {
    yMax = float(targetY);
    yRatio = (camHeight/ (yMax - yMin));                         //
  }
  if(key == CODED && keyCode == 16) {      // shift key was pressed, toggle aim with arrow keys
    useArrowKeys = !useArrowKeys; 
  }
  
}

public void viewCameraSettings() {
  camInput.settings();
}
public void setBackground() {
  camInput.adapt();
}
public void selectColor() {
  selectingColor = true;
}
public void selectSafeColor() {
  selectingSafeColor = true;
}
public void radioEffect(int ID) {
  effect = ID + 1;
}
public void stop() {
  if (!runWithoutArduino) {
    arduinoPort.write("z0000000");
    delay(500);
    arduinoPort.stop();
  }
  camInput.stop();
  super.stop();
}

/*--------------- UCF Group02 (LM Autonomous Vehicle) Edit ---------------*/
void serialEvent( Serial arduinoPort) {
  //put the incoming data into a String - 
  //the '\n' is our end delimiter indicating the end of a complete packet
  //strRange = arduinoPort.readStringUntil('\n');
  
  String temp = arduinoPort.readStringUntil('\n');
  if (temp != null)
    strRange = temp;  
  
  //make sure our data isn't empty before continuing
  if (strRange != null) {
    //trim whitespace and formatting characters (like carriage return)
    strRange = trim(strRange);
    }
}
/*--------------- UCF Group02 (LM Autonomous Vehicle) Edit ---------------*/
