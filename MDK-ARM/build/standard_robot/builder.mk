EIDE_UNIFY_BUILDER := 1
CFLAGS := -c --apcs=interwork --cpu Cortex-M4.fp --c99 -D__MICROLIB -O0 --split_sections --diag_suppress=1 --diag_suppress=1295 -g -I.\..\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc\Legacy -I.\..\Middlewares\Third_Party\FreeRTOS\Source\include -I.\..\Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS -I.\..\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F -I.\..\Drivers\CMSIS\Device\ST\STM32F4xx\Include -I.\..\Drivers\CMSIS\Include -I.\..\application -I.\..\bsp\boards -I.\..\components\devices -I.\..\components\algorithm -I.\..\components\algorithm\Include -I.\..\components\support -I.\..\application\protocol -I.\..\components\controller -I.\..\Middlewares\ST\STM32_USB_Device_Library\Core\Inc -I.\..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc -I.\.eide\deps -DUSE_HAL_DRIVER -DSTM32F407xx -DARM_MATH_CM4 -D__FPU_USED="1U" -D__FPU_PRESENT="1U" -D__CC_ARM -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING
CXXFLAGS := -c --cpp --apcs=interwork --cpu Cortex-M4.fp -D__MICROLIB -O0 --split_sections --diag_suppress=1 --diag_suppress=1295 -g -I.\..\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc\Legacy -I.\..\Middlewares\Third_Party\FreeRTOS\Source\include -I.\..\Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS -I.\..\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F -I.\..\Drivers\CMSIS\Device\ST\STM32F4xx\Include -I.\..\Drivers\CMSIS\Include -I.\..\application -I.\..\bsp\boards -I.\..\components\devices -I.\..\components\algorithm -I.\..\components\algorithm\Include -I.\..\components\support -I.\..\application\protocol -I.\..\components\controller -I.\..\Middlewares\ST\STM32_USB_Device_Library\Core\Inc -I.\..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc -I.\.eide\deps -DUSE_HAL_DRIVER -DSTM32F407xx -DARM_MATH_CM4 -D__FPU_USED="1U" -D__FPU_PRESENT="1U" -D__CC_ARM -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING
ASMFLAGS := --apcs=interwork --cpu Cortex-M4.fp --pd "__MICROLIB SETA 1" -g -I.\..\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc -I.\..\Drivers\STM32F4xx_HAL_Driver\Inc\Legacy -I.\..\Middlewares\Third_Party\FreeRTOS\Source\include -I.\..\Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS -I.\..\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F -I.\..\Drivers\CMSIS\Device\ST\STM32F4xx\Include -I.\..\Drivers\CMSIS\Include -I.\..\application -I.\..\bsp\boards -I.\..\components\devices -I.\..\components\algorithm -I.\..\components\algorithm\Include -I.\..\components\support -I.\..\application\protocol -I.\..\components\controller -I.\..\Middlewares\ST\STM32_USB_Device_Library\Core\Inc -I.\..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc -I.\.eide\deps
LDFLAGS := --cpu Cortex-M4.fp --library_type=microlib --scatter "f:/Robot/Medical_chassis_V0.1/MDK-ARM/build/standard_robot/MDK-ARM.sct" --strict --summary_stderr --info summarysizes --map --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --list .\build\standard_robot\MDK-ARM.map
LDLIBS := 