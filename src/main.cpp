#include <Arduino.h> 
#include <ESP32-TWAI-CAN.hpp>

#define vel_min -6000// Velovcidade minima que estaremos aceitando para o motor neste momento;
#define vel_max 6000 // Velocidade maxima que estaremos aceitando para o motor neste momento;

#define CAN_TX 5 // Pino padrao de envio can na esp32;
#define CAN_RX 4 // Pino padrao de recepcao can na esp32;

#define max_current_phase 300 // diferenca de velocidade na qual o algoritimo de rampa comeca a funcionar, a partir daqui um degrau sera criado para cada diferenca de tamanho.

#define ramping_threshhold 500 // Variavel que define o valor de cada degrau da rampa.

#define tamanho_max_comando 5 // Definicao de tamanho maximo de comando para o parsing, 6 porque o maior possivel e -3000 que tem 5 casas;

#define controller_id 1801D0F0 // id do controlador do motor a ser enviado no barramento can. Muda conforme o endereco do motor que recebe

#define ramping_smoothing_counter_target 2 // se usar um valor menor que 2 a tensao fica muito alta no barramento quando existe reversao

// variavel que guarda a velocidade alvo. Valor inicial 32000 que quer dizer 0rpm.
uint16_t target_speed = 32000;

// variavel que conta a quantidade de smoothers de rampam ja implementados
unsigned short int ramping_smoothing_counter = 0;

// variavel para dizer se o primeiro loop da mudanca de aceleracao;
bool first_loop = true;

// variavel para salvar o valor do ramping threshold com o devido sinal;
short int signed_ramping_threshhold;

// Buffer para guardar o comando sendo digitado na porta serial; O mais um e devido ao fato de todo buffer ter que terminar em NULL para ser convertido devidamente a string.
char buffer[tamanho_max_comando + 1] = {'\0'};

// variavel de contagem do buffer para entrada de comandos; Utilizada somente na funcao handle_incoming_messages().5
unsigned short int buffer_count;

// Espaco para guardar o frame a ser enviado;
CanFrame frame_buffer; 

// !!!NAO EDITE ESSA VARIAVEL SEM USAR A FUNCAO set_if_valid_speed(), RISCO DE QUEIMA DO ESC!!!
// Velocidade que e enviada para o motor a cada frame; 
uint16_t speed = 32000; 

// O valor da corrente das fases do motor; Esse valor tem que ser alterado caso se queira alterar o sentido do motor.
// Se o valor nao for alterado o motor se recusa a inverter o sentido. Te ignorando e ficando parado.
// A corrente de fase e o a velocidade(speed) devem ter o mesmo sentido, isto e, devem possuir o mesmo sinal. Se nao o motor nao se move.
uint16_t current_phase = 32000;

// guarda a contagem do sinal de vida que tem que ser enviado para o motor.
// caso a contagem nao esteja correta ou nao chegue ao motor por 10x seguidas, que se traduzem para 500ms, o motor desliga e emite os 21 beeps de erro no can.
byte life_signal;

// funcao que lida com o envio de mensagens no barramento can;
bool send_can_message () { 
  // Se der tudo certo no envio;
  if (ESP32Can.writeFrame(&frame_buffer)) 
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
void align_current_fase() {
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
  frame_buffer.identifier  = 0x0C01EFD0;// ID do vcu, vulgo este controlador.
  frame_buffer.extd = TWAI_MSG_FLAG_EXTD; // Informa o controlador can da esp32 que esse e um frame do tipo longo, de 29 bits e nao o normal de 11 bits;
  frame_buffer.data_length_code = 8;     // tamanho do pacote de dados, 8 byes;
  frame_buffer.data[0] = 0xAA; // byte 0 == targetPhaseCurrent == torque;
  frame_buffer.data[1] = 0xAA; // byte 1 == targetPhaseCurrent == torque;
  frame_buffer.data[2] = 0xAA;// byte 2 == TargetSpeed == Velocidade; ULTIMOS 4 bits. o proximo serao os primeiro 4 bits.
  frame_buffer.data[3] = 0xAA; // byte 3 == TargetSpeed == Velocidade; Lembrando que e no formato little-endian, que quer dizer que o algarismo mais signficativo vem primeiro
  frame_buffer.data[4] = 0xAA; // byte 4;  O primeiro bit controla se o motor esta ligado ou nao, o segundo se o controle por torque ou velocidade, o resto n serve p nada;\_que se  traduz para o numero e escrito de traz para frente;
  frame_buffer.data[5] = 0xAA; // byte 5; // nao serve para nada aqui;
  frame_buffer.data[6] = 0xAA; // byte 6; // nao serve para nada aqui;
  frame_buffer.data[7] = 0xAA; // byte 7; Sinal de vida, vulgo o numero que tem que ser somado +1 toda vez;
}

// funcao que retorna a mensagem padrao can;
void can_normal_message() {
  frame_buffer.identifier  = 0x0C01EFD0;// ID do vcu, vulgo este controlador.
  frame_buffer.extd = TWAI_MSG_FLAG_EXTD; // Informa o controlador can da esp32 que esse e um frame do tipo longo, de 29 bits e nao o normal de 11 bits;
  frame_buffer.data_length_code = 8; // tamanho do pacote de dados, 8 byes;
  frame_buffer.data[0] = (uint8_t)current_phase; // byte 0 == targetPhaseCurrent == torque;
  frame_buffer.data[1] = (uint8_t)(current_phase >> 8); // byte 1 == targetPhaseCurrent == torque;
  frame_buffer.data[2] = (uint8_t)speed;// byte 2 == TargetSpeed == Velocidade; ULTIMOS 4 bits. o proximo serao os primeiro 4 bits.
  frame_buffer.data[3] = ((uint8_t)(speed >> 8)); // byte 3 == TargetSpeed == Velocidade; Lembrando que e no formato little-endian, que quer dizer que o algarismo mais signficativo vem primeiro
  frame_buffer.data[4] = 0b00000011; // byte 4;  O primeiro bit controla se o motor esta ligado ou nao, o segundo se o controle por torque ou velocidade, o resto n serve p nada;\_que se  traduz para o numero e escrito de traz para frente;
  frame_buffer.data[5] = 0x00; // byte 5; // nao serve para nada aqui;
  frame_buffer.data[6] = 0x00; // byte 6; // nao serve para nada aqui;
  frame_buffer.data[7] = life_signal++; // byte 7; Sinal de vida, vulgo o numero que tem que ser somado +1 toda vez;
  align_current_fase();
}

// lida com a rampa de aceleracao/desaceleracao
void handle_accelaration() {
  // calcula a diferenca de velocidade e salva numa variavel;
  int speed_change = (int)target_speed - (int)speed;
  // se a diferenca de velocidade for maior que o valor para comecar a rampa
  if (abs(speed_change) > ramping_threshhold || (ramping_smoothing_counter != 0)) {
    // se a aceleracao for positia
    if (speed_change > 0) {
      // o threshold de rampa com sinal e positivo
      signed_ramping_threshhold = ramping_threshhold;
    } else {
      signed_ramping_threshhold = -ramping_threshhold;
    }
    // caso o contrario, threshold de rampa com sinal e negativo
    // se o contador de smoother de rampa for menor que o numero que queremos chegar
    if ((ramping_smoothing_counter) < ramping_smoothing_counter_target) {
      // se for a primeira iteracao do contador, colocamos ja o primeiro valor da velocidade
      if (first_loop == true) {
        speed += signed_ramping_threshhold;
        first_loop = false;
      }
      //incrementados o contador
      ramping_smoothing_counter++;
      // e nao fazemos mais nada, logo retornamos.
      return;
    } else {
      // se nao tivermos que fazer smoothing, zeramos o contador de smoothing para uma possivel proxima rodada
      // nesse caso seria umas das rodadas entre o primeiro e ultimo
      ramping_smoothing_counter = 0;
      // adicionamos a velocidade;
      speed+= signed_ramping_threshhold;
      // retornamos da funcao por aqui.
      return;
    }
  }
  // se nao for, zera o contador do smoother the rampa, caso seja o ultimo da rampa
  ramping_smoothing_counter = 0;
  // e seta a velocidade para ser igual a velocidade alvo;
  speed = target_speed;
  first_loop = true;
}

// lida com as mensagens do motor;
void handle_incoming_messages() { 
  // Se houver alguma mensagem no buffer CANa.
  if (ESP32Can.readFrame(&frame_buffer, 0)) { 
    // lida com a mensagem da anuncio do motor.
    // Se for uma mensagem de anuncio do motor.1801D0F0
    if (frame_buffer.identifier == 0x1801D0EF) {
      if (frame_buffer.data[0] == 0x55 && frame_buffer.data[3] == 0x55 && frame_buffer.data[7] == 0x55) { 
      // se o frame 0,3 e 7 contem 0x55 assumimos que tudo e 0x55 e logo e uma mensagem de anuncio do motor;
        send_can_handshake();
      }
      else {
        // lida com manter o motor apos a primeira que nao seja de anuncio e recebida;
        // Se nao for uma mensagem de anuncio de motor, responde com uma mensagem de funcionamento;
        can_normal_message();
        // handle_acceleration e colocado aqui por que queremos que ele execute somente em intervalos de 50 em 50 ms, e nao antes.
        // Se colocarmos no main loop ele vai executar muito rapido e a velocidade mudaria nao a cada tick de 50ms, mas a cada
        // loop do esp32 que e muito mais rapido;
        handle_accelaration();
      }
      // envia a mensagem can.
      send_can_message();
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
    target_speed = 32000 + l_speed;
    //imprime na tela as informacoes;
    Serial.print("Nova velocidade alvo escolhida com sucesso, ");
    Serial.print(l_speed);
    Serial.print("rpm");
    Serial.print(". Valor exato alvo sendo enviado: ");
    Serial.print(target_speed);
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

void loop() {
handle_serial_input();
handle_incoming_messages();
}
  
