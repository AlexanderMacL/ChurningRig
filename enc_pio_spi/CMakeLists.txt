cmake_minimum_required(VERSION 3.23)

if (TARGET tinyusb_device)

        add_executable(enc_pio_spi)

        pico_generate_pio_header(enc_pio_spi ${CMAKE_CURRENT_LIST_DIR}/enc_pio_spi.pio)

        target_sources(enc_pio_spi PRIVATE
                enc_pio_spi.c
                SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.c
                )

        target_link_libraries(enc_pio_spi PRIVATE
                pico_stdlib
                hardware_pio
		hardware_irq
                hardware_spi
                hardware_i2c
                )

        # enable usb output, disable uart output
        pico_enable_stdio_usb(enc_pio_spi 1)
        pico_enable_stdio_uart(enc_pio_spi 0)

        pico_add_extra_outputs(enc_pio_spi)

        # add url via pico_set_program_url
        example_auto_set_url(enc_pio_spi)

elseif(PICO_ON_DEVICE)
        message(WARNING "not building enc_pio_spi because TinyUSB submodule is not initialized in the SDK")
endif()