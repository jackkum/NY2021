#include <avr/io.h>
#include <avr/interrupt.h>

#include <SD.h>             // библиотека для работы с SD картой
#define SD_ChipSelectPin 10  //using digital pin 4 on arduino nano 328, can use other pins
//#include <TMRpcm.h>         // библиотека для работы с аудио
#include "font.h"

//Пин подключен к ST_CP входу 74HC595
const int latchPin = 6;
//Пин подключен к SH_CP входу 74HC595
const int clockPin = 7;
//Пин подключен к DS входу 74HC595
const int dataPin = 8;

//TMRpcm tmrpcm;

typedef enum {
  READY,
  PLAING,
  GAME_OVER
} STATUS_GAME;

typedef enum {
  VERTICAL,
  HORIZONTAL
} TEXT_ORIENTATION;

typedef enum {
  NONE,
  UP,
  DOWN,
  LEFT,
  RIGHT,
  CENTER
} DIRECTION;

typedef enum {
  NONE_GAME,
  SNAKE,
  TETRIS
} GAMES;

typedef struct {
  GAMES game;
  const char text[30];
} MENU;

typedef struct {
  uint8_t x;
  uint16_t y;
  DIRECTION d;
} FIFO;

volatile uint16_t buff[8] = {0,0,0,0,0,0,0,0};
volatile uint8_t col = 0;
volatile uint8_t cPos = 4;
volatile uint8_t game = NONE;
MENU menu[2] = {{SNAKE,"Snake"}, {TETRIS, "Tetris"}};
volatile uint8_t menuPos = 0;

uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

DIRECTION getDirection(void) {
  uint16_t adc = analogRead(A0);
  // center = 960
  // up     = 557
  // donw   = 845
  // right  = 704
  // left   = 326

  if(adc >= 960){ // center
    return NONE;
  }
  
  if(adc >= 940){ // center
    return CENTER;
  }
  
  if(adc >= 830){ // down
    return DOWN;
  }
  
  if(adc >= 690){ // right
    return RIGHT;
  } 
  
  if(adc >= 540){ // up
    return UP;
  } 
  
  if(adc >= 310){ // left
    return LEFT;
  }

  return NONE;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  cli(); // отключить глобальные прерывания
  TCCR2A = 0; // установить TCCR1A регистр в 0
  TCCR2B = 0;// (1<<CS20);
  //TCNT2  = 0x1F;

  // включить прерывание Timer1 overflow:
  TIMSK2 = (1 << TOIE2);
  // Установить CS10 бит так, чтобы таймер работал при тактовой частоте:
  TCCR2B |= (1 << CS20)|(1 << CS22);

  sei();

  // инициализация и настройка воспроизведения с карты
  //tmrpcm.speakerPin = 9;
  //tmrpcm.setVolume(5);
  //tmrpcm.quality(0);

  SD.begin(SD_ChipSelectPin);

  delay(300);

  //tmrpcm.play("music");
}

ISR(TIMER2_OVF_vect)
{
  // устанавливаем синхронизацию "защелки" на LOW
  digitalWrite(latchPin, LOW);

  // передаем отсчет для вывода на зеленые светодиоды
  shiftOut(dataPin, clockPin, MSBFIRST, (1<<col));

  uint16_t leds = buff[col];
  
  // передаем обратный отсчет  для вывода на красные светодиоды
  shiftOut(dataPin, clockPin, MSBFIRST, 0xFF & (leds>>8));
  // передаем обратный отсчет  для вывода на красные светодиоды
  shiftOut(dataPin, clockPin, MSBFIRST, 0xFF & (leds));
  
  //"защелкиваем" регистр, тем самым устанавливая значения на выходах
  digitalWrite(latchPin, HIGH);

  col++;
  if(col >= 8){
    col = 0;
  }

  //TCNT2  = 0x1F;
}

void clearScreen(void){
  cli();
  for(uint8_t i = 0; i < 8; i++){
    buff[i] = 0;
  }
  sei();
}

void printChar(uint8_t b, int offset, TEXT_ORIENTATION orientation){
  unsigned int index = ((unsigned int)b*8);
  for(uint8_t i = 0; i < 8; i++){
    uint8_t v = pgm_read_byte(&font_8x8[index + i]);

    if(orientation == VERTICAL){
      buff[i] = ((uint16_t)reverse(v)<<8) >> offset;
    } else {
      for(uint8_t x = 0; x < 8; x++){
        int o = i + offset;
        if(o >= 0 && o < 16){
          if(v & (1<<x)){
            buff[x] |= (1<<o);
          } else {
            buff[x] &= ~(1<<o);
          }
        }
      }
    }
  }
}

void printText(const char *str, TEXT_ORIENTATION orientation, uint8_t reset){
  static int offset = 0;

  if(reset){
    offset  = 0;
  }

  clearScreen();
  cli();
  
  int l = 0;
  for(l = 0; l < strlen(str); l++){
    int o = (l*8) - offset;
    printChar(str[l], 16+o, orientation);
  }
  sei();
  
  offset++;
  if(offset > (strlen(str) * 8) + 16){
    offset = 0;
  }

  delay(100);
}

void matrix(){
  cli();
  for (byte x = 0; x < 8; x++) {
    uint8_t r = random(0, 255);

    if(r > 200){
      buff[x] |= (1<<15);
    } else {
      buff[x] &= ~(1<<15);
    }
  }
  sei();

  delay(50);

  cli();
  for (byte x = 0; x < 8; x++) {
    buff[x] = (uint16_t)buff[x]>>1;
  }
  sei();

  delay(50);
}

void pac_man(){
  cli();
  buff[1] = 0x3C0<<3;
  buff[2] = 0x7C0<<3;
  buff[3] = 0x780<<3;
  buff[4] = 0x780<<3;
  buff[5] = 0x7C0<<3;
  buff[6] = 0x3C0<<3;

  buff[4] |= (1<<0) | (1<<2) | (1<<4) | (1<<6) | (1<<8);
  sei();

  delay(500);

  cli();
  buff[1] = 0x3C0<<3;
  buff[2] = 0x7E0<<3;
  buff[3] = 0x7E0<<3;
  buff[4] = 0x7E0<<3;
  buff[5] = 0x7E0<<3;
  buff[6] = 0x3C0<<3;

  buff[4] |= (1<<1) | (1<<3) | (1<<5) | (1<<7);
  sei();

  delay(500); 
}

void snakeGame(void) {
  static FIFO *snake;
  static FIFO food;
  static uint8_t len = 0, status = READY;
  static uint8_t speed = 1;
  static long int ms = millis();

  if(status == GAME_OVER){
    game = NONE;
    return;
  }

  // init snake 
  if(snake == NULL){
    // alloc memory
    snake = (FIFO *) malloc(sizeof(FIFO));

    // 
    snake[len].x = random(0, 4);
    snake[len].y = random(0, 8);
    snake[len].d = (DIRECTION) random(1, 5);
    len++;

    food.x = random(0, 4);
    food.y = random(0, 8);
  }

  clearScreen();

  cli();
  // draw foot
  buff[food.x] |= (1<<food.y);

  // draw snake
  for(uint16_t i = 0; i < len; i++) {
    FIFO p = snake[i];
    buff[p.x] |= (1<<p.y);
  }
  sei();

  bool update = (millis() - ms) > (1000-(speed*10));
  if(update){
    ms = millis();

    FIFO p = snake[len-1];

    switch(p.d){
      case DOWN:
        if(p.y == 0){
          p.y = 15;
        } else {
          p.y--;
        }
      break;
      case UP:
        p.y++;
        if(p.y >= 16){
          p.y = 0;
        }
      break;
      
      case LEFT:
        if(p.x == 0){
          p.x = 7;
        } else {
          p.x--;
        }
      break;
      case RIGHT:
        p.x++;
        if(p.x >= 8){
          p.x = 0;
        }
      break;
    }

    bool collision = food.x == p.x && food.y == p.y;
    if(collision) {
      //tmrpcm.disable();
      //tmrpcm.play("eat.wav");
      // next level
      speed++;
      // resize
      snake = (FIFO *) realloc(snake, sizeof(FIFO) * (len+1));
  
      snake[len].x = p.x;
      snake[len].y = p.y;
      snake[len].d = p.d;
      len++;

      bool nextFood = true;
      do {
        // generate food position
        food.x = random(0, 4);
        food.y = random(0, 8);

        // continue this loop?
        nextFood = false;
        // each cell of snake
        for(uint16_t i = 0; i < len; i++){
          // food inside a snake
          if(snake[i].x == food.x && snake[i].y == food.y){
            // continue
            nextFood = true;
            // break this loop
            break;
          }
        }
      } while(nextFood);
      
    } else {

      bool damage = false;
      for(uint8_t i = 0; i < (len-1); i++){
        // head inside it self
        if(snake[i].x == p.x && snake[i].y == p.y){
          // continue
          damage = true;
          // break this loop
          break;
        }
      }

      if(damage){
        // game is over
        status = GAME_OVER;
        // free memory
        free(snake);
        // reset vars
        snake = NULL;
        len = 0;
        speed = 1;
        // clear screen
        clearScreen();
        return;
      }
      
      for(uint8_t i = 0; i < (len-1); i++){
        snake[i].x = snake[i+1].x;
        snake[i].y = snake[i+1].y;
        snake[i].d = snake[i+1].d;
      }

      snake[len-1] = p;
    }
  }

  switch(getDirection()){
    case NONE: break;
    case CENTER:break;

    case DOWN:
      if(snake[len-1].d != UP){
        snake[len-1].d = DOWN;
      }
      break;
    case UP:
      if(snake[len-1].d != DOWN){
        snake[len-1].d = UP;
      }
      break;
    case LEFT:
      if(snake[len-1].d != RIGHT){
        snake[len-1].d = LEFT;
      }
      break;
    case RIGHT:
      if(snake[len-1].d != LEFT){
        snake[len-1].d = RIGHT;
      }
      break;
  }

  delay(100);
}

void tetrisGame(void){
  static uint8_t reset = 1;
  printText("Not implement yet \x01", HORIZONTAL, reset);
  reset = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  // matrix();
  // pac_man();
  static uint8_t reset = 0;
  uint8_t maxPos = (sizeof(menu)/sizeof(MENU))-1;

  switch(game){
    case NONE_GAME:
      
      switch(getDirection()){
        case DOWN:
          if(menuPos > 0){
            menuPos--;
          } else {
            menuPos = maxPos;
          }
          reset = 1;
          break;
        case UP:
          if(menuPos < maxPos){
            menuPos++;
          } else {
            menuPos = 0;
          }
          reset = 1;
          break;
         case CENTER:
          game = menu[menuPos].game;
         break;
      }
      
      printText(menu[menuPos].text, HORIZONTAL, reset);
      
      if(reset){
        delay(500);
      }
      
      reset = 0;
    break;

    case SNAKE:
      snakeGame();
    break;

    case TETRIS:
      tetrisGame();
    break;
  }
}
