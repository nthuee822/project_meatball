openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f /target/stm32f1x.cfg -c "program $1 verify reset exit"
