# STM32
@Cách thêm thư viện lcd trong STM32 IDE:  
  lcd.c --> Src  
  lcd.h --> Inc  
@Cách thêm thư viện lcd trong Keil C V5:  
  <>Hiển thị trong Keil C:  
    lcd.c --> Drivers/CMSIS  
    lcd.h --> Drivers/CMSIS/lcd.c  
    lcd.h --> Application/User/main.c  
  <>Hiển thị trong folder:  
    lcd.c --> MDK-ARM  
    lcd.h --> Drivers/STM32F0xxHAL_Driver/Inc  
 => file main.c (#include "lcd.h")  
 
 
