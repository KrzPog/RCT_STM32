# Modbus Protocol

Modbus protocol used according to [Official Modbus Specification](https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf).

## Enabled functions

* 03 (0x03) - Read Holding Registers
* 04 (0x04) - Read Input Registers
* 06 (0x06) - Write Single Register
* 16 (0x10) - Write Multiple registers
* 17 (0x11) - Report Server ID

## CRC Calculation

[Example CRC Calculator](https://crccalc.com/?crc=010400000005&method=CRC-16/MODBUS&datatype=hex&outtype=hex)

> For testing connection:
> 
> REQUEST:  `0x0111C02C`  
> RESPONSE: `0x011101FF1013` 

**NOTE:** Low Order Byte First (Calculator gives `0x0930` -> append `0x3009` to end of frame)  
**NOTE:** Sometimes serial viewers don't display last byte -> `\r`


# Flash Values

**NOTE:** UPDATE ALL WHEN CHANGED !!!

## Currently set flash values

| Address   | Register Name                    | Value   | Type    | Description            |
|-----------|----------------------------------|---------|---------|------------------------|
| `0x5000`  | **REG_FLASH_ROT_CONFIG**         | `0x0006`| uint16  |                        |
| `0x5002`  | **REG_FLASH_ELEV_CONFIG**        | `0x0006`| uint16  |                        |
| `0x5004`  | **REG_FLASH_ROT_POSITION_MIN**   | `-3600` | int16   | degree * 10            |
| `0x5006`  | **REG_FLASH_ROT_POSITION_MAX**   | `3600`  | int16   | degree * 10            |
| `0x5008`  | **REG_FLASH_ELEV_POSITION_MIN**  | `-50`   | int16   | degree * 10            |
| `0x500A`  | **REG_FLASH_ELEV_POSITION_MAX**  | `500`   | int15   | degree * 10            |
| `0x500C`  | **REG_FLASH_ROT_SPEED_MIN**      | `10`    | uint16  | degree * 10 / s        |
| `0x500E`  | **REG_FLASH_ROT_SPEED_MAX**      | `600`   | uint16  | degree * 10 / s        |
| `0x5010`  | **REG_FLASH_ELEV_SPEED_MIN**     | `10`    | uint16  | degree * 10 / s        |
| `0x5012`  | **REG_FLASH_ELEV_SPEED_MAX**     | `600`   | uint16  | degree * 10 / s        |
| `0x5014`  | **REG_FLASH_ROT_DUTY_PWM_MIN**   | `10`    | uint16  | percent * 10           |
| `0x5016`  | **REG_FLASH_ROT_DUTY_PWM_MAX**   | `600`   | uint16  | percent * 10           |
| `0x5018`  | **REG_FLASH_ELEV_DUTY_PWM_MIN**  | `10`    | uint16  | percent * 10           |
| `0x501A`  | **REG_FLASH_ELEV_DUTY_PWM_MAX**  | `600`   | uint16  | percent * 10           |
| `0x501C`  | **REG_FLASH_ROT_UART_SPEED_MIN** | `1`     | uint16  | rpm                    |
| `0x501E`  | **REG_FLASH_ROT_UART_SPEED_MAX** | `10`    | uint16  | rpm                    |
| `0x5020`  | **REG_FLASH_ELEV_UART_SPEED_MIN**| `1`     | uint16  | rpm                    |
| `0x5022`  | **REG_FLASH_ELEV_UART_SPEED_MAX**| `10`    | uint16  | rpm                    |

## Modbus frame to write them

### Enable flash writing

> REQUEST:  
> `01 06 4000 0010 9DC6`
> 
> RESPONSE:  
> `01 06 4000 0010 9DC6`

**NOTE**: Update when **CONTROL_WORD** changed

### Verify that flash is enabled

> REQUEST:  
> `01 04 3000 0001 3ECA`
> 
> RESPONSE:  
> *Bit **REG_INPUT_STATUS_WORD_BIT_FLASH_UNLOCKED** should be set*

**NOTE**: Update when **STATUS_WORD** changed

### Write data

> REQUEST:  
> `01 10 5000 0012 24 0006 0006 F220 0E10 FFCE 01F4 000A 0258 000A 0258 000A 0258 000A 0258 0001 000A 0001 000A 6534`
> 
> RESPONSE:  
> *Should start with `01 10 5000`*

**NOTE**: Update when table changed

### Disable flash writing

> REQUEST:  
> `01 06 4000 0010 9DC6`
> 
> RESPONSE:  
> `01 06 4000 0010 9DC6`

**NOTE**: Update when **CONTROL_WORD** changed

### Verify that flash is disabled

> REQUEST:  
> `01 04 3000 0001 3ECA`
> 
> RESPONSE:  
> *Bit **REG_INPUT_STATUS_WORD_BIT_FLASH_UNLOCKED** should be unset*

**NOTE**: Update when **STATUS_WORD** changed

### Verify written values

Power off, power on and:

> REQUEST:  
> `01 03 5000 0012 D4C7`
> 
> RESPONSE:  
> *Should contain written values*