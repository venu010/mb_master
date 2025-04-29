set(PROJECT_NAME "modbus_master")

idf_component_register(SRCS "test_ota.c" "Sungrow_inv.c" "Solis_inv.c" "inverter_generation.c" "Growatt_inv.c" "Goodwe_inv.c" "dev_conf_flash.c" "data_logger.c" "RMS_MAIN.c" "EMS_meter.c"
                        INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
