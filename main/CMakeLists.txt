idf_component_register(SRCS "main.c" "ble_midi_service.c" "ble_config_service.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt nvs_flash)
