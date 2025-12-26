
#include <HardwareSerial.h>
#include <Arduino.h>




HardwareSerial FPSerial(2);




/*** PIN Definitions ***/
const int BTN_UP_PIN = 26;    // GPIO 26
const int BTN_DOWN_PIN = 25;  // GPIO 25
const int UART_TX_PIN = 17;   // GPIO 17
const int SPEAKER_PIN = 18;




/*** Screen Resolution ***/
const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;




/*** Player / Ball State ***/
// Initially centered
int16_t p1_y = SCREEN_HEIGHT / 2;
int16_t p2_y = SCREEN_HEIGHT / 2;
int16_t ball_x = SCREEN_WIDTH / 2;
int16_t ball_y = SCREEN_HEIGHT / 2;
// Score
int p1_score = 0;
int p2_score = 0;
bool game_over = false;




int16_t ball_vx = 4;  // x-velocity (initially 4)
int16_t ball_vy = 3;  // y-velocity (initially 3)




// Rendering Parameters
const int16_t PADDLE_SPEED = 6;
const int16_t PADDLE_HEIGHT = 60;
const int16_t PADDLE_WIDTH = 10;
const int16_t BALL_SIZE = 8;




// Paddle X-coord for collision
const int16_t P1_X = 20;
const int16_t P2_X = SCREEN_WIDTH - 20 - PADDLE_WIDTH;




//////////////// Game restart
unsigned long bothPressedTime = 0;
const unsigned long RESTART_TIME = 500;






void setup() {
 Serial.begin(115200);  // ESP32 setup (115200 is the frequency that ESP32 communicates)
 FPSerial.begin(115200, SERIAL_8N1, -1, UART_TX_PIN);




 pinMode(BTN_UP_PIN, INPUT_PULLUP);    // Set the pinmode to "input", defalut status to up (not pressed)
 pinMode(BTN_DOWN_PIN, INPUT_PULLUP);  // Same


 //ledcAttach(SPEAKER_PIN, SPEAKER_FREQ, 8);
 pinMode(SPEAKER_PIN, OUTPUT);
 digitalWrite(SPEAKER_PIN, LOW);  // no sound
  // Test beep
 playTone(1000, 200);
 delay(100);
 playTone(1500, 200);
}




/*** Functions! ***/
void readButtons() {
 // Store status for buttons if they are pressed
 bool upPressed = (digitalRead(BTN_UP_PIN) == LOW);
 bool downPressed = (digitalRead(BTN_DOWN_PIN) == LOW);




 // Update coordinates based on button press, speed of predefined velocity
 if (game_over) {
   if (upPressed && downPressed) {
     if (bothPressedTime == 0) bothPressedTime = millis();
   } else {
     bothPressedTime = 0;
   }
   return;
 }




 if (upPressed) {
   // move up (coordinate system is opposite!)
   p1_y -= PADDLE_SPEED;
 }
 if (downPressed) {
   // move down
   p1_y += PADDLE_SPEED;
 }




 // Bound check for player character via y-coordinate
 if (p1_y < 0) {
   // limit the top boundary
   p1_y = 0;
 }
 if (p1_y > SCREEN_HEIGHT - PADDLE_HEIGHT) {
   // limit the bottom boundary
   p1_y = SCREEN_HEIGHT - PADDLE_HEIGHT;
 }
}




// Reset, initial movement of the ball with two directions
void resetBall(int direction) {
 // Center the ball
 ball_x = SCREEN_WIDTH / 2;
 ball_y = SCREEN_HEIGHT / 2;




 // Set direction, keep the speed
 if (direction > 0) {
   ball_vx = abs(ball_vx);
 } else {
   ball_vx = -abs(ball_vx);
 }
}




void updateGamePhysics() {
 if (game_over) return;
 // Motion of the ball (coordinate updating)
 ball_x += ball_vx;
 ball_y += ball_vy;




 // Boundary collision
 bool bounced = false;
 if (ball_y <= 0 || ball_y >= SCREEN_HEIGHT - 1) {
   // Invert the motion when the ball hits the boundary
   ball_vy = -ball_vy;
   playTone(900, 35);
 }


 // Check for ball's x-coordinate overlap with paddle
 bool isCollideP1_X = (ball_x <= P1_X + PADDLE_WIDTH) && (ball_x + BALL_SIZE >= P1_X);
 // Check for ball's y-coordinate overlap with paddle
 bool isCollideP1_Y = (ball_y <= p1_y + PADDLE_HEIGHT) && (ball_y + BALL_SIZE >= p1_y);




 // If the ball was moving left & collided with P1
 if (ball_vx < 0 && isCollideP1_X && isCollideP1_Y) {
   // Relocate the ball slightly to not go inside the paddle
   ball_x = P1_X + PADDLE_WIDTH;
   // Bounce right
   ball_vx = abs(ball_vx);




  playTone(1200, 40);
 }




 // Check for ball's x-coordinate overlap with paddle 2
 bool isCollideP2_X = (ball_x + BALL_SIZE >= P2_X) && (ball_x <= P2_X + PADDLE_WIDTH);
 // Check for ball's y-coordinate overlap with paddle 2
 bool isCollideP2_Y = (ball_y <= p2_y + PADDLE_HEIGHT) && (ball_y + BALL_SIZE >= p2_y);




 // If the ball was moving right & collided with P2
 if (ball_vx > 0 && isCollideP2_X && isCollideP2_Y) {
   // Relocate the ball slightly to not go inside the paddle
   ball_x = P2_X - PADDLE_WIDTH;
   // Bounce left
   ball_vx = -abs(ball_vx);
   playTone(1200, 40);
 }


 // TODO: Scoring Logic!!!
 if (ball_x + BALL_SIZE < 0) {
   p2_score++;
   playTone(1800, 140);
   if (p2_score >= 5) {
     game_over = true;
   }
   resetBall(+1);
 } else if (ball_x > SCREEN_WIDTH) {
   p1_score++;
   playTone(1800, 140);
   if (p1_score >= 5) {
     game_over = true;
   }
   resetBall(-1);
 }


 // Simple AI: player2 follows the ball's y-coordinate
 if (ball_y > p2_y + PADDLE_HEIGHT / 2) {
   // Homing slower than player character
   // p2_y += (2 * PADDLE_SPEED / 3);
   p2_y += (PADDLE_SPEED / 3);
 } else if (ball_y < p2_y + PADDLE_HEIGHT / 2) {
   p2_y -= (2 * PADDLE_SPEED / 3);
 }




 // Player 2 bound check
 if (p2_y < 0) {
   // limit top boundary
   p2_y = 0;
 }
 // Bound check for Player 2
 if (p2_y > SCREEN_HEIGHT - PADDLE_HEIGHT) {
   // limit bottom boundary
   p2_y = SCREEN_HEIGHT - PADDLE_HEIGHT;
 }
}




void playTone(unsigned int freq, unsigned int duration_ms) {
 if (freq == 0) {
   digitalWrite(SPEAKER_PIN, LOW);
   return;
 }
 unsigned long period_us = 1000000UL / freq;  // Period in microseconds
 unsigned long half_period = period_us / 2;
  // Number of cycles
 unsigned long cycles = (freq * duration_ms) / 1000;
  // Generate the tone
 for (unsigned long i = 0; i < cycles; i++) {
   digitalWrite(SPEAKER_PIN, HIGH);
   delayMicroseconds(half_period);
   digitalWrite(SPEAKER_PIN, LOW);
   delayMicroseconds(half_period);
 }
}




// Debugging!!!
void printData() {
 // Printing out the values...
 Serial.print("\n******************************\n");
 Serial.print("P1_Y: ");
 Serial.print(p1_y);
 Serial.print('\n');
 Serial.print("P2_Y: ");
 Serial.print(p2_y);
 Serial.print('\n');
 Serial.print("Ball_X: ");
 Serial.print(ball_x);
 Serial.print('\n');
 Serial.print("Ball_Y: ");
 Serial.print(ball_y);
 Serial.print('\n');
 Serial.print("p1_score: ");
 Serial.print(p1_score);
 Serial.print('\n');
 Serial.print("p2_score: ");
 Serial.print(p2_score);
 Serial.print('\n');
}




// Send data to FPGA for VGA Output
/*
* We are using "UART" communication to simplify the wire connection between the ESP32 and FPGA.
* UART sends 1 byte (8bit), so we have to divide our 16bit data into two (divide by 8)
* Bit operator >> will shift the bits to the right, to send the first 8 bits (later be combined at FPGA)
* & with 0xFF (00000000 11111111) will "AND" the bits, which clears any unwanted bits [masking]
* Since UART cannot differentiate the data, we mark the initial point and end point with two opposite bits.
* One initial ~ end cycle will be 1 frame.
*/
void sendDataP(int16_t p1y, int16_t p2y, int16_t bx, int16_t by, int8_t p1_s, int8_t p2_s) {
 Serial.write(0xA5);  // Start (indicates beginning)




 // Sending Player Character's Coordinate
 Serial.write((p1y >> 8) & 0xFF);  // Shift the bits to the right 8 times to leave only the first 8 bits
 Serial.write(p1y & 0xFF);         // Send the rest 8 bits
 // Sending Player 2's Coordinate
 Serial.write((p2y >> 8) & 0xFF);
 Serial.write(p2y & 0xFF);
 // Sending Ball's X Coordinate
 Serial.write((bx >> 8) & 0xFF);
 Serial.write(bx & 0xFF);
 // Sending Ball's Y Coordinate
 Serial.write((by >> 8) & 0xFF);
 Serial.write(by & 0xFF);
 // Score Data
 Serial.write((uint8_t)p1_s);
 Serial.write((uint8_t)p2_s);




 Serial.write(0x5A);  // End (indicates end)
}




void sendData(int16_t p1y, int16_t p2y, int16_t bx, int16_t by, int8_t p1_s, int8_t p2_s) {
 FPSerial.write(0xA5);  // Start (indicates beginning)




 // Sending Player Character's Coordinate
 FPSerial.write((p1y >> 8) & 0xFF);  // Shift the bits to the right 8 times to leave only the first 8 bits
 FPSerial.write(p1y & 0xFF);         // Send the rest 8 bits
 // Sending Player 2's Coordinate
 FPSerial.write((p2y >> 8) & 0xFF);
 FPSerial.write(p2y & 0xFF);
 // Sending Ball's X Coordinate
 FPSerial.write((bx >> 8) & 0xFF);
 FPSerial.write(bx & 0xFF);
 // Sending Ball's Y Coordinate
 FPSerial.write((by >> 8) & 0xFF);
 FPSerial.write(by & 0xFF);


 FPSerial.write((uint8_t)p1_s);
 FPSerial.write((uint8_t)p2_s);


 FPSerial.write(0x5A);  // End (indicates end)
}




void loop() {
 readButtons();  // Read input for player character


 if (game_over && bothPressedTime != 0 && (millis() - bothPressedTime >= RESTART_TIME)) {
   p1_score = 0;
   p2_score = 0;
   game_over = false;
   bothPressedTime = 0;
   resetBall(1);
 }


 updateGamePhysics();                    // Calculating position, collision, <score>
 printData();                            // Debuggg
 sendData(p1_y, p2_y, ball_x, ball_y, p1_score, p2_score);   // Send data to FPGA
 sendDataP(p1_y, p2_y, ball_x, ball_y, p1_score, p2_score);  // send data to TX pin...




 delay(16);  // 16: about 60fps
}

