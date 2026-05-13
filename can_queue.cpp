#include <Arduino.h> 
#include <ESP32-TWAI-CAN.hpp>


#define can_queue_max_size 10

// Definindo a estrutura que guardara
typedef struct __can_queue_item
{
    // cada can_queue_item_t tem um can frame
    CanFrame can_frame;
    // e um horario que ele deve ser enviado.
    unsigned short int time_to_be_sent;
} can_queue_item_t;

// short int para guardar o tamanho usado da fila, para facilitar operacoes;
unsigned short int can_queue_size;

// O vetor que guardara os membros da fila
can_queue_item_t queue_vector[can_queue_max_size];

// ??
unsigned long can_queue_timer;

// Add an item to the quere
bool can_enqueue(CanFrame item, unsigned short int time_to_be_sent) {
    // verifica se a fila nao esta cheia
    if (can_queue_size < can_queue_max_size) {
            // cria o frame de can_queue_item_t para ser enviado para fila;
            can_queue_item_t frame;
            // adiciona o horario que o can_frame devera ser enviado;
            frame.time_to_be_sent = time_to_be_sent;
            // adiciona o frame can no item da fila;
            frame.can_frame = item;
            // adiciona o item na fila;
            queue_vector[can_queue_size] = frame;
            // adiciona 1 para o numero de itens na fila
            can_queue_size += 1;
            // retorna 1 para dize que tudo deu certo
            return 1;
        }
    // Retorna -1 para dizer que houve um problema;
    return -1;
}

// Funcao para remover item da fila e mover outros itens para frente da fila;
// Requer um ponteiro para que a mensagem can seja gravada se houver;
int can_dequeue (CanFrame *message) {
    // checa se existe algum item na fila
    if (can_queue_size > 1) {
        // guarda a posicao do possivel frame que esta a tempo de ser enviado.
        unsigned short int on_time = NULL;
        // guarda o possivel frame a ser enviado;
        CanFrame item;
        // para todos membros da fila;
        for (unsigned short int i = 0; i < can_queue_size; i++) {
            // checa se existe uma mensagem na fila em tempo de ser enviada
            if (queue_vector[i].time_to_be_sent >= millis()) {
                // seestiver, retorna a mensagem
                on_time = i;
            } 
        }
        // Se existir alguma mensagem da fila para ser enviada
        if (on_time != NULL) {
                // guarda o item a ser defileirado
                can_queue_item_t item = queue_vector[on_time];
                // remove 1 do numeros de itens na fila
                can_queue_size -= 1;
                // Puxa os n-1 membros para baixo
                for (unsigned short int i = (on_time); i < can_queue_size; i++) {
                    queue_vector[i] = queue_vector[i+1];
                }
                // torna o valor do po item presente no ponteiro enviado no canframe.
                *message = item.can_frame;
                // retorna 1 para dizer que deu tudo certo;
                return 1;
            }
        // nao existem itens na fila a serem enviados, retorna -1 para saber que nao existem itens na fila
        return -1;
    }
}

// Funcao que retorna o frame can a ser enviado no barramento can.
// Recebe o frame can e o tempo de delay antes de enviar o mesmo.
unsigned short int can_queue(CanFrame *message_from_queue, CanFrame *default_message) {
    // cria um espaco para guarda a possivel mensagem can recebida.
    CanFrame *can_message;
    // se a funcao tirar da fila tiver alguem para tirar da fila;
    if (can_dequeue(can_message) == 1) {
        // Escreve no ponteiro recebido a mensageem da fila.
        *message_from_queue = *can_message;
        return -1;
    } else {
        // Se nao tiver, escreve a mensagem padrao;
        *message_from_queue = *default_message;
        return 1;
    }
}