#include <Arduino.h> 
#include <ESP32-TWAI-CAN.hpp>

// Definindo a estrutura que guardara
typedef struct __can_queue_item can_queue_item_t;

// short int para guardar o tamanho usado da fila, para facilitar operacoes;
unsigned short int can_queue_size;

// O vetor que guardara os membros da fila
can_queue_item_t queue_vector;

// ??
unsigned long can_queue_timer;

// Add an item to the quere
bool can_enqueue;

// Funcao para remover item da fila e mover outros itens para frente da fila;
// Requer um ponteiro para que a mensagem can seja gravada se houver;
int can_dequeue (CanFrame *message);

// Funcao que retorna o frame can a ser enviado no barramento can.
// Recebe o frame can e o tempo de delay antes de enviar o mesmo.
unsigned short int can_queue(CanFrame *message_from_queue, CanFrame *default_message);
