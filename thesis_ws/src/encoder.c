#include "encoder.h"  // Include the header file for encoder functions
#include "quadrature_encoder.pio.h"  // Include the header file for the PIO program
#include <math.h>  // Include math functions

// TODO: Add copyright

// Global variables to store encoder pins and PIO instance
uint ENCODER1;
uint ENCODER2;
uint PIN_AB;
uint PIN_CD;
PIO pio = pio0;

// Function to reset encoder counts to zero
void reset_encoders() {
    // Reset the encoder counts to zero
    pio_sm_exec(pio, ENCODER1, pio_encode_set(pio_y, 0));
    pio_sm_exec(pio, ENCODER2, pio_encode_set(pio_y, 0));
}

// Function to initialize PIO for encoder input
int init_PIO_encoder(uint pin_ab, uint pin_cd, uint encoder1, uint encoder2) {
    // Initialize global variables with provided pin and encoder values
    PIN_AB = pin_ab;
    PIN_CD = pin_cd;
    ENCODER1 = encoder1;
    ENCODER2 = encoder2;

    // Add the PIO program for quadrature encoding
    uint offset = pio_add_program(pio, &quadrature_encoder_program);
    // Initialize the PIO state machines for both encoders
    quadrature_encoder_program_init(pio, encoder1, PIN_AB, 0);
    quadrature_encoder_program_init(pio, encoder2, PIN_CD, 0);

    // Reset encoder counts to zero
    reset_encoders();

    // Return success
    return 0;
}

// Function to read encoder count for Encoder A
int32_t get_encoder_count_A() {
    // Read and return the encoder count for Encoder A
    return quadrature_encoder_get_count(pio, ENCODER1);
}

// Function to read encoder count for Encoder B
int32_t get_encoder_count_B() {
    // Read and return the encoder count for Encoder B
    return quadrature_encoder_get_count(pio, ENCODER2);
}

// Function to read encoder count for a specific encoder and reset its count to zero
int32_t encoder_read_and_reset(uint encoder) {
    // Read the encoder count
    int32_t count = quadrature_encoder_get_count(pio, encoder);
    // Reset the encoder count to zero
    pio_sm_exec(pio, encoder, pio_encode_set(pio_y, 0));
    // Return the read count
    return count;
}

// Function to read encoder count for Encoder A and reset its count to zero
int32_t encoder_read_and_reset_A() {
    // Read and reset the encoder count for Encoder A
    int32_t count = quadrature_encoder_get_count(pio, ENCODER1);
    pio_sm_exec(pio, ENCODER1, pio_encode_set(pio_y, 0));
    return count;
}

// Function to read encoder count for Encoder B and reset its count to zero
int32_t encoder_read_and_reset_B() {
    // Read and reset the encoder count for Encoder B
    int32_t count = quadrature_encoder_get_count(pio, ENCODER2);
    pio_sm_exec(pio, ENCODER2, pio_encode_set(pio_y, 0));
    return count;
}
