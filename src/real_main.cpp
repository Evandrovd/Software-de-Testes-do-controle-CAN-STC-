#include <Arduino.h> 
#include <ESP32-TWAI-CAN.hpp>
#include <can_queue.hpp>

#define vel_min -5500 // Velovcidade minima que estaremos aceitando para o motor neste momento;
#define vel_max 5500 // Velocidade maxima que estaremos aceitando para o motor neste momento;

#define CAN_TX 5 // Pino padrao de envio can na esp32;
#define CAN_RX 4 // Pino padrao de recepcao can na esp32;

#define max_current_phase 300


// diferenca de velocidade na qual o algoritimo de rampa comeca a funcionar, a partir daqui um degrau sera criado para cada diferenca de tamanho.
#define ramping_threshhold 500

#define delay_inversao 700 // tempo para esperar quando existe e pedida uma inversao do motor, em ms, 1000ms = 1s

#define tamanho_max_comando 5 // Definicao de tamanho maximo de comando para o parsing, 6 porque o maior possivel e -3000 que tem 5 casas;

#define controller_id 1801D0F0 // id do controlador do motor a ser enviado no barramento can. Muda conforme o endereco do motor que recebe

char buffer[tamanho_max_comando + 1] = {'\0'};// Buffer para guardar o comando sendo digitado na porta serial; O mais um e devido ao fato de todo buffer ter que terminar em NULL para ser convertido devidamente a string.

// variavel de contagem do buffer para entrada de comandos; Utilizada somente na funcao handle_incoming_messages().5
unsigned short int buffer_count;

// Espaco para guardar o frame a ser enviado;
CanFrame txFrame; 

// guarda o horario da ultima mensagem enviada para comparamos com o horario atual
// assim garantimos que nao vamos sobrecarregar o CAN bus com milhares de mensagens.
unsigned long last_sent_time; 

// !!!NAO EDITE ESSA VARIAVEL SEM USAR A FUNCAO set_if_valid_speed(), RISCO DE QUEIMA DO ESC!!!
// Velocidade que e enviada para o motor a cada frame; 
uint16_t speed = 32000; 

// Velocidade a ser guardada pois deve ser enviada com o delay de inversao;
uint16_t delayed_speed;

// O valor da corrente das fases do motor; Esse valor tem que ser alterado caso se queira alterar o sentido do motor.
// Se o valor nao for alterado o motor se recusa a inverter o sentido.
// A corrente de fase e o a velocidade(speed) devem ter o mesmo sentido, isto e, devem possuir o mesmo sinal.
uint16_t current_phase = 32000;

// guarda a contagem do sinal de vida que tem que ser enviado para o motor.
// caso a contagem nao esteja correta ou nao chegue ao motor por 10x seguidas, que se traduzem para 500ms, o motor desliga e emite os 21 beeps de erro no can.
byte life_signal;

// funcao que lida com o envio de mensagens no barramento can;
bool send_can_message () { 
  // Se der tudo certo no envio;
  if (ESP32Can.writeFrame(txFrame)) 
  {
    // retorta true;
    return true;
  } 
  // se nao der tudo certo;
  else 
  {
    // imprime mensagem de erro;
    Serial.println("Houve uma falha no envio de uma mensagem CAN."); 
    // Retorna false;
    return false; 
  }
}


// funcao para alinhar a corrente de fase com a direcao de movimento desejada;
// Recebe a velocidade;
void align_current_fase(int peed) {
    // Se o sentido novo do movimento for positivo;
  if (speed > 32000) {
    //Serial.println("Invertendo Corrente de Fase.");
    // seta a corrente de fase para o positiva;
    current_phase = 32000 + max_current_phase;
    // seta a corrente de fase para o negativa;
  } else if (speed < 32000) {
    //Serial.println("Invertendo Corrente de Fase.");
    current_phase = 32000 - max_current_phase;
  }
}

// funcao que envia a mensagem de handshake via can;
void send_can_handshake() {
  Serial.println("Respondendo o pedido de handshake do motor!");
  life_signal = 0; // Zera o sinal de vida pra que a proxima mensagem seja o numero 1; 1801D0F0
  txFrame.identifier  = 0x0C01EFD0;// ID do vcu, vulgo este controlador.
  txFrame.extd = TWAI_MSG_FLAG_EXTD; // Informa o controlador can da esp32 que esse e um frame do tipo longo, de 29 bits e nao o normal de 11 bits;
  txFrame.data_length_code = 8;     // tamanho do pacote de dados, 8 byes;
  txFrame.data[0] = 0xAA; // byte 0 == targetPhaseCurrent == torque;
  txFrame.data[1] = 0xAA; // byte 1 == targetPhaseCurrent == torque;
  txFrame.data[2] = 0xAA;// byte 2 == TargetSpeed == Velocidade; ULTIMOS 4 bits. o proximo serao os primeiro 4 bits.
  txFrame.data[3] = 0xAA; // byte 3 == TargetSpeed == Velocidade; Lembrando que e no formato little-endian, que quer dizer que o algarismo mais signficativo vem primeiro
  txFrame.data[4] = 0xAA; // byte 4;  O primeiro bit controla se o motor esta ligado ou nao, o segundo se o controle por torque ou velocidade, o resto n serve p nada;\_que se  traduz para o numero e escrito de traz para frente;
  txFrame.data[5] = 0xAA; // byte 5; // nao serve para nada aqui;
  txFrame.data[6] = 0xAA; // byte 6; // nao serve para nada aqui;
  txFrame.data[7] = 0xAA; // byte 7; Sinal de vida, vulgo o numero que tem que ser somado +1 toda vez;
  send_can_message(); // envia a mensagem
  last_sent_time = millis(); // Seta o horario do envio da ultima mensagem;
}

// funcao que retorna a mensagem padrao can;
CanFrame can_default_message() {
  CanFrame txFrame;
  txFrame.identifier  = 0x0C01EFD0;// ID do vcu, vulgo este controlador.
  txFrame.extd = TWAI_MSG_FLAG_EXTD; // Informa o controlador can da esp32 que esse e um frame do tipo longo, de 29 bits e nao o normal de 11 bits;
  txFrame.data_length_code = 8; // tamanho do pacote de dados, 8 byes;
  txFrame.data[0] = (uint8_t)current_phase; // byte 0 == targetPhaseCurrent == torque;
  txFrame.data[1] = (uint8_t)(current_phase >> 8); // byte 1 == targetPhaseCurrent == torque;
  txFrame.data[2] = (uint8_t)speed;// byte 2 == TargetSpeed == Velocidade; ULTIMOS 4 bits. o proximo serao os primeiro 4 bits.
  txFrame.data[3] = ((uint8_t)(speed >> 8)); // byte 3 == TargetSpeed == Velocidade; Lembrando que e no formato little-endian, que quer dizer que o algarismo mais signficativo vem primeiro
  txFrame.data[4] = 0b00000011; // byte 4;  O primeiro bit controla se o motor esta ligado ou nao, o segundo se o controle por torque ou velocidade, o resto n serve p nada;\_que se  traduz para o numero e escrito de traz para frente;
  txFrame.data[5] = 0x00; // byte 5; // nao serve para nada aqui;
  txFrame.data[6] = 0x00; // byte 6; // nao serve para nada aqui;
  txFrame.data[7] = life_signal++; // byte 7; Sinal de vida, vulgo o numero que tem que ser somado +1 toda vez;
  align_current_fase(speed);
  return txFrame;
}

// fucao que monta um frame can com uma velocidade diferente.
CanFrame can_speed_change_message(int speed) {
  CanFrame txFrame;
  txFrame.identifier  = 0x0C01EFD0;// ID do vcu, vulgo este controlador.
  txFrame.extd = TWAI_MSG_FLAG_EXTD; // Informa o controlador can da esp32 que esse e um frame do tipo longo, de 29 bits e nao o normal de 11 bits;
  txFrame.data_length_code = 8; // tamanho do pacote de dados, 8 byes;
  txFrame.data[0] = (uint8_t)current_phase; // byte 0 == targetPhaseCurrent == torque;
  txFrame.data[1] = (uint8_t)(current_phase >> 8); // byte 1 == targetPhaseCurrent == torque;
  txFrame.data[2] = (uint8_t)speed;// byte 2 == TargetSpeed == Velocidade; ULTIMOS 4 bits. o proximo serao os primeiro 4 bits.
  txFrame.data[3] = ((uint8_t)(speed >> 8)); // byte 3 == TargetSpeed == Velocidade; Lembrando que e no formato little-endian, que quer dizer que o algarismo mais signficativo vem primeiro
  txFrame.data[4] = 0b00000011; // byte 4;  O primeiro bit controla se o motor esta ligado ou nao, o segundo se o controle por torque ou velocidade, o resto n serve p nada;\_que se  traduz para o numero e escrito de traz para frente;
  txFrame.data[5] = 0x00; // byte 5; // nao serve para nada aqui;
  txFrame.data[6] = 0x00; // byte 6; // nao serve para nada aqui;
  txFrame.data[7] = life_signal++; // byte 7; Sinal de vida, vulgo o numero que tem que ser somado +1 toda vez;
  align_current_fase(speed);
  return txFrame;
}
 
// Calcula a divisao de teto de a por b, ou a/b. Exemplo 3/2 = 2 porque 1.5 arredondado para cima e 2.
int ceil_division(int a, int b) {
  return (1+((a-1)/b));
}

// lida com as mensagens do motor;
void handle_incoming_messages() {
  // Espaco para guardar o frame recebido;
  CanFrame rxFrame;   
  // Se houver alguma mensagem no buffer CAN.
  if (ESP32Can.readFrame(rxFrame, 0)) { 
    // lida com a mensagem da anuncio do motor.
    // Se for uma mensagem de anuncio do motor.1801D0F0
    if (rxFrame.data[0] == 0x55 && rxFrame.data[3] == 0x55 && rxFrame.data[7] == 0x55) 
      { // se o frame 0,3 e 7 contem 0x55 assumimos que tudo e 0x55 e logo e uma mensagem de anuncio do motor;
        send_can_handshake();
      }
    // lida com manter o motor apos a primeira que nao seja de anuncio e recebida;
    // Se nao for uma mensagem de anuncio de motor.
    else if ((millis() - last_sent_time) >= 50) // Se for qualquer outra mensagem que nao tudo 0x55, quer dizer que o motor esta enviado mensagens de funcionamento;
    { 
      c_queue(&txFrame, can_default_message); // Checa se tem algo na fila se tiver escreve na mensagem de envio, se nao escreve a mensagem padrao;
      if (send_can_message() == false) Serial.println("erro no envido da mensagem can");
      last_sent_time = millis(); // Seta o horario do envio da ultima mensagem;
    }
  } 
}
// *Atencao!!! setar a variavel speed sem usar essa funcao pode queimar o ESC pois nao existe protecao via CAN para inversao de corrente instantanea;*
// Lida como envio das mensages ja interpretada pela funcao de lidar com entrada serial;
void set_if_valid_speed(int l_speed) {
  // checando se a velocidade pedida esta dentro dos limites que establecemos na secao #define.
  if (l_speed >= vel_min && l_speed <= vel_max ) { 
    // tem que colocar o offset de +32000 como diz no datasheet;
    // 32000 e igual a velocidade 0.
    int new_speed;
    new_speed = 32000 + l_speed;

    int speed_dif = new_speed - speed;

    Serial.print("Speed = ");
    Serial.print(speed);
    Serial.print(", new speed = ");
    Serial.print(new_speed);
    Serial.print(", Speed_dif = ");
    Serial.println(speed_dif);
    
    // se sped_dif for maior que o max, implementa o algoritimo de rampa
    if ((abs(speed_dif)) > ramping_threshhold) {
      Serial.println("Implementando rampa.");

      // calcula numero de passos ate chegar na velocidade alvo;
      unsigned short int number_of_steps = ceil_division(abs(speed_dif),ramping_threshhold);
      // escreve o numero de "degrais" necessarios;
      // i<2 pq nao precisamos implementar o ultimo degrau ja que ele ja vai ser a speed final;
      for(unsigned short int i = number_of_steps; i > 1; i--) {
        if (speed_dif > 0) {
          // atualiza o valor da velocidade;
          speed += ramping_threshhold;
          Serial.print("Velocidade de rampa = ");
          Serial.println(speed);
          // adicionando degrau de rampa na fila can;
          can_enqueue(can_speed_change_message(speed));
        } else {
          speed -= ramping_threshhold;
          // adicionando degrau de rampa na fila can;
          can_enqueue(can_speed_change_message(speed));
        }
      }

      // apos inserir todos o degrais de rampa na fila, inserimos a velocidade final na velocidade a ser enviada nas mensagens padrao a partir de entao;
      speed = new_speed;

    } else {
      Serial.println("Nao precisa de rampa, inserindo diretamente a velocidade.");
      speed = new_speed;
    }
    //imprime na tela as informacoes;
    Serial.print("Nova velocidade escolhida com sucesso, ");
    Serial.print(l_speed);
    Serial.print("rpm");
    Serial.print(". Valor exato sendo enviado: ");
    Serial.print(new_speed);
    Serial.println(".");
  } else {
    Serial.print("Valor inserido invalido. Por favor insira um valor entre ");
    Serial.print(vel_min);
    Serial.print(" e ");
    Serial.print(vel_max);
    Serial.println("rpm.");
  }
}

// Lida com as entradas via uart(terminal);
void handle_serial_input() { 
  if (Serial.available()) {
    char incoming_char = Serial.read();
    if (incoming_char ==  '\n') { // caso om proximo comando seja enter;
      if (buffer[0] == '\n') {
        Serial.println("Erro, tentou incluir buffer vazio!");
        buffer_count = 0;
      }
      else {
        if (buffer_count < 6) {
          buffer[buffer_count] = '\0';
        }
        if (buffer[0] == '\n') {
          Serial.println("Erro, tentou incluir buffer vazio!");
          buffer_count = 0;
        }
        else {
          if (buffer_count > 0) {
            set_if_valid_speed(atoi(buffer));
            //Serial.print("Setando a velocidade para: ");
            //Serial.println((char*)buffer);
            incoming_char = 0;
            buffer_count = 0;
          } else {
            Serial.println("Erro, tentou incluir buffer vazio!");
          }
        }
      }
    } 
    else {
      if (buffer_count < 6 && incoming_char != '\r') {
        Serial.print("Escrevendo valor no bufer:");
        Serial.println(incoming_char);
        buffer[buffer_count] = incoming_char;
        buffer_count+= 1;
      } 
    }
  }
}

void setup() {
  // setar os pinos do can;
  ESP32Can.setPins(CAN_TX, CAN_RX);
  // Setando tamanho da fila de frames can; esses sao os padroes;
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);
  // setando a velocidade do barramento can; Tem que usar o esp32can.convertspeed() pq a entrada dessa funcao e um enum interno da espressif.
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
  // Tentando iniciar a comunicacao can;
  // se o can for inicializado com sucesso
  if(ESP32Can.begin()) {
    // imprime que deu tudo certo
    Serial.println("INFO - CAN bus inicializado!");
  } else {
    // imprime que teve algum problema;
    Serial.println("ERRO - Falha na inicializacao do CAN bus");
  }
  // iniciar contagem do buffer comom 0;
  buffer_count = 0;
  // Comecar serial para debugging; baud rate 115200;
  Serial.begin(115200);
  // printar linha de inicializacao completa.
  Serial.println("Esp32 can inicializado com sucesso! Pronto para trasmitir.");
}

// todo implementar fila can, que envia as mensagens se der o tempo de delay de envio delas, ou envia a mensagem padrao keepalive se nao houver mensagens
// todo, reescrever codigo para aguardar ate que a velocidade do motor chege a zero, pela leitura de sensores, e entao comecar a acelar, ao inves de usar 1s como fazemos atualmente.

void loop() {
handle_serial_input();
handle_incoming_messages();
}
  
