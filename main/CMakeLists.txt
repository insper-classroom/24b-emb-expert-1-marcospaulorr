add_executable(main main.c)

# pull in common dependencies
target_link_libraries(main pico_stdlib hardware_adc hardware_pwm
hardware_dma
        hardware_pio
        hardware_timer
        hardware_clocks
        hardware_pwm)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)
