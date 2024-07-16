



add_executable(${NAME}
    main.c
    pico_uart_transport.c
    custom_pwm.c
    motor_control.c
    utils.c
    encoder.c
    motor_pid.c
    motor_calcs.c
    motor.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
    pico_stdlib
    micro_ros
    hardware_gpio hardware_pwm
    hardware_pio
    hardware_i2c
	)
	
pico_generate_pio_header(${NAME}  ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
     )
     
target_compile_definitions(${NAME} PRIVATE
    configNUM_CORES=1
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# Configure Pico
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)