#include <Arduino.h> 
#include <ESP32-TWAI-CAN.hpp>

#define number_of_resends 4
#define can_queue_max_size 40

CanFrame can_frame;

unsigned short int resends_counter = 0;

// short int para guardar o tamanho usado da fila, para facilitar operacoes;
unsigned short int can_queue_size = 0;

// O vetor que guardara os membros da fila
CanFrame queue_vector[can_queue_max_size];

void print_can_message(CanFrame can_message) {
    Serial.printf("%x", can_message.identifier);
    Serial.print(" ");
    Serial.printf("[%x]", can_message.data_length_code);
    Serial.print(" "); 
    Serial.printf("%x %x %x %x %x %x %x %x\n", can_message.data[0], can_message.data[1], can_message.data[2], can_message.data[3], can_message.data[4], can_message.data[5], can_message.data[6], can_message.data[7]);
}

// Add an item to the quere
bool can_enqueue(CanFrame item) {
    if (can_queue_size < can_queue_max_size) {
            Serial.println("Added to can queue");
            // adicona frame na fila
            queue_vector[can_queue_size] = item;
            // adiciona 1 para o numero de itens na fila
            can_queue_size += 1;
        }
    // Retorna -1 para dizer que houve um problema;
    return false;
}

// Funcao para remover item da fila --> e consequentemente mover outros itens para frente da fila;
// Requer um ponteiro para que a mensagem can seja gravada se houver;
int can_dequeue (CanFrame *message) {
    if (message == NULL) return -1;
    // checa se existe algum item na fila
    if (can_queue_size > 1) {
        // guarda o item a ser defileirado
        CanFrame item = queue_vector[0];
        // remove 1 do numeros de itens na fila
        can_queue_size -= 1;
        // Puxa os n-1 membros para baixo
        for (unsigned short int i = 0; i < can_queue_size; i++) {
            queue_vector[i] = queue_vector[i+1];
        }
        // torna o valor do po item presente no ponteiro enviado no canframe.
        *message = item;
        //
        Serial.print("Retornando velocidade de rampa da fila:");
        // retorna 1 para dizer que deu tudo certo;
        return 1;
    }
    // nao existem itens na fila a serem enviados, retorna -1 para saber que nao existem itens na fila
    return -1;
}

// Funcao que retorna o frame can a ser enviado no barramento can.
// Recebe um ponteiro para guardar o frame can e o tempo de delay antes de enviar o mesmo.
short int c_queue(CanFrame *returning_from_queue) {
    // se a funcao tirar da fila tiver alguem para tirar da fila;
    if (can_queue_size > 0) {
        // cria um espaco para guarda a possivel mensagem can recebida.
        CanFrame can_message;
        print_can_message(can_message);
        can_dequeue(&can_message);
        *returning_from_queue = can_message;
        return 1;
    }
    return -1;
}

