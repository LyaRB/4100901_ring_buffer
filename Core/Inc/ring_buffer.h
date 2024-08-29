/*
 * ring_buffer.h
 *
 *  Created on: Aug 23, 2024
 *      Author: Paulina Ruíz
 */
//Librerías
#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_
#include <stdint.h>
#include <string.h>

//estructura para las variables

typedef struct ring_buffer_{
	uint8_t *buffer;
	uint8_t head;
	uint8_t tail;
	uint8_t is_full;
	uint8_t capacity;
} ring_buffer_t;


//void ring_buffer_write(uint8_t data);
//uint8_t ring_buffer_read(uint8_t *byte);

void ring_buffer_write(ring_buffer_t *rb, uint8_t data); //funcion con los nuevos argumentos
uint8_t ring_buffer_read(ring_buffer_t *rb,uint8_t *byte);

//uint8_t ring_buffer_reset();
//uint8_t ring_buffer_empty(void);
//uint8_t ring_buffer_size(void);
//uint8_t ring_buffer_is_full(void);
//uint8_t right_ID(uint8_t *data);

void ring_buffer_init(ring_buffer_t *rb, uint8_t *mem, uint8_t capacity);
void ring_buffer_reset(ring_buffer_t *rb);
uint8_t ring_buffer_size(ring_buffer_t *rb);
uint8_t ring_buffer_is_full(ring_buffer_t *rb);
uint8_t ring_buffer_is_empty(ring_buffer_t *rb);

#endif /* INC_RING_BUFFER_H_ */
