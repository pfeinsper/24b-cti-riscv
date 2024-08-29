#define EN1 10
#define EN2 11
#define EN3 12

#define IN1 8
#define IN2 9
#define IN3 2

uint8_t in_seq[6][3] = {{1,0,0},{0,1,0},{0,1,0},{0,0,1},{0,0,1},{1,0,0}};
uint8_t en_seq[6][3] = {{1,0,1},{0,1,1},{1,1,0},{1,0,1},{0,1,1},{1,1,0}};

void setup() {
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);

  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
  digitalWrite(EN3, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
}

void loop() {
  // girar horário
  for(uint8_t i=0; i<6; i++){
    digitalWrite(IN1, in_seq[i][0]);
    digitalWrite(IN2, in_seq[i][1]);
    digitalWrite(IN3, in_seq[i][2]);

    digitalWrite(EN1, en_seq[i][0]);
    digitalWrite(EN2, en_seq[i][1]);
    digitalWrite(EN3, en_seq[i][2]);

    delay(5);
  }
}

// girar antihoario
// void loop() {
//   for(int8_t i=5; i>=0; i--){
//     digitalWrite(IN1, in_seq[i][0]);
//     digitalWrite(IN2, in_seq[i][1]);
//     digitalWrite(IN3, in_seq[i][2]);

//     digitalWrite(EN1, en_seq[i][0]);
//     digitalWrite(EN2, en_seq[i][1]);
//     digitalWrite(EN3, en_seq[i][2]);

//     delay(5);
//   }
// }
