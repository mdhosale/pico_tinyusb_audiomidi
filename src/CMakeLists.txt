cmake_minimum_required(VERSION 3.13)

add_executable(pico_tinyusb_audiomidi)
target_sources(pico_tinyusb_audiomidi PUBLIC
    main.c
    usb/usb_descriptors.c
)

pico_set_program_name(pico_tinyusb_audiomidi "pico_tinyusb_audiomidi")
pico_set_program_version(pico_tinyusb_audiomidi  "0.1")


# Make sure compiler can find .h files
target_include_directories(pico_tinyusb_audiomidi PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/usb
)



# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example

target_link_libraries(pico_tinyusb_audiomidi PUBLIC 
	pico_stdlib 
	tinyusb_device 
	tinyusb_board
		
	hardware_adc
    hardware_dma
)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_tinyusb_audiomidi)
