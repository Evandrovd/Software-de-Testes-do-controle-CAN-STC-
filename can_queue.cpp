#include <Arduino.h> 
#include <ESP32-TWAI-CAN.hpp>


#define queue_size 10

typedef struct __can_queue_item
{
    CanFrame frame;
    unsigned short int delay;
} can_queue_item_t;


can_queue_item_t queue_vector[queue_size];

unsigned long can_queue_timer;

bool add_to_can_cue(CanFrame item, unsigned short int delay) {
    for (unsigned short int i; (queue_vector[i].frame.identifier == 0x00); i++) {
        if (i < queue_size) {
            can_queue_item_t frame;
            frame.delay = delay;
            frame.frame = item;
            return 1;
        }
    }
    return -1;
}

// Funcao que retorna o frame can a ser enviado no barramento can.
// Recebe o frame can e o tempo de delay antes de enviar o mesmo.
void can_queue() {
    unsigned long time_now = millis();
    bool was_message_sent = 0;
    for (unsigned short int i; (queue_vector[i].frame.identifier != 0x00); i++) {
        if ((queue_vector[i].delay - millis()) <= 0) {
            // send can maessage in queue;
                // TODO
            // escreve 0x00 no identificador do item == remove o item da fila;
            queue_vector[i].frame.identifier == 0x00;
            //
            was_message_sent = 1;
        }
    }
    if (!was_message_sent) {
        // send can keepalive message;
            // TODO
        was_message_sent = 1;
    }
}