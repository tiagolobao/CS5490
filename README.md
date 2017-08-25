<h1>CS5490</h1>

<h3>PT-BR</h3>
Biblioteca Arduino/ESP Para Comunicação com o Chip Cirrus Logic CS5490
<h3>EN</h3>
Arduino Library / ESP for Communication with the Cirrus Logic CS5490 Chip


## Contributing [![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](https://github.com/dwyl/esta/issues)

## Warning [![code status](https://img.shields.io/badge/code-incomplete-red.svg?style=flat)](https://github.com/dwyl/esta/issues)

<h2>Informações Importantes / Important Information</h2>


<h3> Baud rate </h3>

how to calculate:
baud rate = BR[15:0] / (524288/MCLK)

default: 
MCLK = 4.096Mhz
baud rate = 600


<h3> Serial time out </h3>

after 128 ms


<h3> Command Format </h3>


Function         |     Binary Value       | specification

Register Read    | 0 0 A5 A4 A3 A2 A1 A0  | A[5:0] 
 
Register Write   | 0 1 A5 A4 A3 A2 A1 A0  | A[5:0]

Page Select      | 1 0 P5 P4 P3 P2 P1 P0  | P[5:0]

Instruction      | 1 1 C5 C4 C3 C2 C1 C0  | C[5:0] 


<h3> Binaries Instructions Format  </h3>

Controls

0 00001 - Software Reset
0 00010 - Standby
0 00011 - Wakeup
0 10100 - Single Conv.
0 10101 - Continuous Conv.
0 11000 - Halt Conv.


Calibration

1 C[4:0]

1 00 C[2:0] - DC Offset
1 10 C[2:0] - AC Offset
1 11 C[2:0] - Gain



<h3> How to Read register info  </h3>

After setting the page:

1 - Send the register address (1 Byte)
2 - Recieve register data (3 Bytes)


<h3> How to Write register  </h3>

After setting the page:

1 - Send the register address (1 Byte)
2 - Send register data (3 Bytes)


<h3>Hardware important topics </h3>

VIN Max voltage: 250mV (input impedance: 2  MOhm)
IIN Max current: 250mV (input impedance: 30 KOhm)