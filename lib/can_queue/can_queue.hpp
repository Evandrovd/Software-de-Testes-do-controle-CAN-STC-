#ifndef can_queue
#define can_queue

#include <can_queue.cpp>
#include <ESP32-TWAI-CAN.hpp>

// Definindo a estrutura que guardara
typedef struct __can_queue_item can_queue_item_t;

// Add an item to the quere
bool can_enqueue(CanFrame item);

// Funcao para remover item da fila e mover outros itens para frente da fila;
// Requer um ponteiro para que a mensagem can seja gravada se houver;
int can_dequeue (CanFrame *message);

// Funcao que retorna o frame can a ser enviado no barramento can.
// Recebe uma funcao que retorne o frame can padrao, caso nao haja nada na fila, retorna o frame padrao;
short int c_queue(CanFrame *message_from_queue, CanFrame (*default_message)());
#endif