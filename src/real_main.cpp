#include <Arduino.h> 
#include <ESP32-TWAI-CAN.hpp>
#include <can_queue.hpp>

#define vel_min -5000 // Velovcidade minima que estaremos aceitando para o motor neste momento;
#define vel_max 5000 // Velocidade maxima que estaremos aceitando para o motor neste momento;

#define CAN_TX 5 // Pino padrao de envio can na esp32;
#define CAN_RX 4 // Pino padrao de recepcao can na esp32;

#define max_current_phase 300;

#define delay_inversao 700 // tempo para esperar quando existe e pedida uma inversao do motor, em ms, 1000ms = 1s

#define tamanho_max_comando 5 // Definicao de tamanho maximo de comando para o parsing, 6 porque o maior possivel e -3000 que tem 5 casas;

#define controller_id 1801D0F0 // id do controlador do motor a ser enviado no barramento can. Muda conforme o endereco do motor que recebe

char buffer[tamanho_max_comando + 1] = {'\0'};// Buffer para guardar o comando sendo digitado na porta serial; O mais um e devido ao fato de todo buffer ter que terminar em NULL para ser convertido devidamente a string.

// variavel de contagem do buffer para entrada de comandos; Utilizada somente na funcao handle_incoming_messages().5
unsigned short int buffer_count;

// Variavel para gravar o tempo que houve a ultima inversao, para garantir que o motor nao sera invertido sem antes aguardar o tempo necessario para ele parar
// o que pode resultar na queima do mesmo ou do esc.
unsigned long last_inversion_time;


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

bool toggle_delay;

// O valor da corrente das fases do motor; Esse valor tem que ser alterado caso se queira alterar o sentido do motor.
// Se o valor nao for alterado o motor se recusa a inverter o sentido.
// A corrente de fase e o a velocidade(speed) devem ter o mesmo sentido, isto e, devem possuir o mesmo sinal.
uint16_t current_phase = 32000;

// guarda a contagem do sinal de vida que tem que ser enviado para o motor.
// caso a contagem nao esteja correta ou nao chegue ao motor por 10x seguidas, que se traduzem para 500ms, o motor desliga e emite os 21 beeps de erro no can.
byte life_signal;

void send_can_handshake() {
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

CanFrame send_can_default() {
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
  return txFrame;
}
 



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
void align_current_fase() {
    // Se o sentido novo do movimento for positivo;
  if (speed > 32000) {
    Serial.println("Invertendo Corrente de Fase.");
    // seta a corrente de fase para o positiva;
    current_phase = 32000 + max_current_phase;
    // seta a corrente de fase para o negativa;
  } else if (speed < 32000) {
    Serial.println("Invertendo Corrente de Fase.");
    current_phase = 32000 - max_current_phase;
  }
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
          CanFrame * mesage_to_send;
          can_queue(&txFrame, send_can_default);
        }
  }
  else {
    Serial.println("Mensagem nao reconhecida recebida via CAN.");
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
    new_speed = 32000;
    new_speed += l_speed;

    Serial.print("Speed = ");
    Serial.print(speed);
    Serial.print(", new speed = ");
    Serial.println(new_speed);
    
    // se houver inversao de sentido;
    if (((speed > 32000) && (new_speed < 32000)) || ((speed  < 32000) && (new_speed > 32000))) {
      Serial.println("Adicionando delay de 1s para adicionar a proxima velocidade;");
      // Liga o delay.
      toggle_delay = 1;
      // seta velocidade para 0, para aguadar o delay
      speed = 32000;
      // seta a velocidade do motor para a nova velocidade, so que com delay.
      delayed_speed = new_speed;
      // seta a tempo desde ultima inversao para agora;
      last_inversion_time = millis();
    } else {
      Serial.println("Nao precisa de delay, inserindo diretamente a velocidade.");
      // calcula o tempo passado desde o ultimo pedido de inversao;
      unsigned long int t_ult_ped_inversao = (millis() - last_inversion_time);
      // checa se nao existe tempo a ser aguardado devido a um pedido de inversao;
      if ( t_ult_ped_inversao >= delay_inversao) {
        // seta a proxima velocidade a ser enviada.
        speed = new_speed;
      } 
      //caso exista tempo a ser aguardado, informa o tempo no terminal;
      else {
        Serial.print("Erro - voce nao pode alterar a velocidade durante um periodo de inversao, aguarde ");
        Serial.print(t_ult_ped_inversao);
        Serial.println("ms.");
        return; // para a execucao da funcao aqui mesmo, para nao desperdicar recurcos;
      }
    }
    //seta o current phase de acordo com o maximo e o com o sentido do movimento pedido;
    align_current_fase();
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

// lida com possiveis delays necessarios na alteracao da velocidade, devido a inversao;
// delay --> tempo em millisegundos para aguardar antes de trocar a velocidade;
void handle_set_speed_delays() {
  if (((millis() - last_inversion_time) >= delay_inversao) && toggle_delay) {
    speed = delayed_speed;
    //seta o current phase de acordo com o maximo e o com o sentido do movimento pedido;
    align_current_fase();
    Serial.print("Setando Velocidade depois do delay ");
    Serial.println(speed);
    toggle_delay = 0;
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
  //Iniciar o ultimo tempo de inversao como 0 para nao dar problemas nas contas que temos que fazer com isso;
  last_inversion_time = 0;
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
  
