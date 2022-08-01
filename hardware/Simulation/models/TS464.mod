* WARNING : please consider following remarks before usage *
* 1) All models are a tradeoff between accuracy and complexity (ie. simulation 
*    time).
* 2) Macromodels are not a substitute to breadboarding, they rather confirm the
*    validity of a design approach and help to select surrounding component values.
*
* 3) A macromodel emulates the NOMINAL performance of a TYPICAL device within
*    SPECIFIED OPERATING CONDITIONS (ie. temperature, supply voltage, etc.).
*    Thus the macromodel is often not as exhaustive as the datasheet, its goal 
*    is to illustrate the main parameters of the product.
*
* 4) Data issued from macromodels used outside of its specified conditions 
*    (Vcc, Temperature, etc) or even worse: outside of the device operating 
*    conditions (Vcc, Vicm, etc) are not reliable in any way.
*-----------------------------------------------------------------------------------------
** Standard Linear Ics Macromodels, 1999. 
** CONNECTIONS :
* INN INVERTING INPUT
* INP NON-INVERTING INPUT
* OUT OUTPUT
* VCC POSITIVE POWER SUPPLY
* VEE NEGATIVE POWER SUPPLY
*
.SUBCKT TS464 INP INN VCC VEE OUT
*****************************
CCC N119 N19 16p
CC30 VEE OUT 130p
CCIP INP VEE 1p
CCIN INN VEE 1p
CCPS N11 N15 100p
EEIP N10 VEE INP VEE 1
EEIN N16 VEE INN VEE 1
RRAN N119 VEE 260K
RR28 N19 N23 6
RRAP N119 VCC 260K
RR2N VEE N19 10Meg
RR2P N19 VCC 10Meg
RRIP N11 N10 8.12
RRIS N11 N15 220
RRIN N15 N16 8.12
DDOPM N19 N22 MDTH 400p
DDONM N21 N19 MDTH 400p
DDCOPY N504 N505 MDTH 400E-9
DDINR N15 N18 MDTH 400E-12
DDOP N19 N25 MDTH 400p
DD39 N506 N504 MDTH 400E-9
DDON N24 N19 MDTH 400p
DDIN N15 N14 MDTH 400p
DDIP N11 N12 MDTH 400p
DDINN N17 N13 MDTH 400E-12
VVOFN N13 N14 0
VVINM VEE N27 60
VVIPM N28 VCC 76
VVOFP N12 N13 -1m
VVCOPYP N505 0 0
VVIN N17 VEE 900m
VVOUT OUT N23 0
VVIP VCC N18 1.4
VVINT2 N503 0 5
VVINT1 N500 0 5
VVN N502 0 0
VVP N501 0 0
RVVN N501 0 10
RVVP N502 0 10
VVCOPYN 0 N506 0
VVOP VCC N25 1
VVON N24 VEE 1.025
IIPOL N13 VEE 63.6u
FFGM1P N119 VEE VVOFP 1
FFGM1N N119 VEE VVOFN 1
FFCOPY N503 N504 VVOUT 1
FFCP VCC VEE VVOFP 33.8
FFIBP INP VEE VVOFP 7.85e-3
FFIBN VEE INN VVOFN 7.85e-3
FFCN VEE VCC VVOFN 33.8
GG2P N19 VEE N119 VEE 1.92e-2
GGCONVP N500 0 N119 VCC 19.38
GGCONVN N500 0 N119 VEE 19.38
GG2N N19 VEE N119 VCC 1.92e-2
HHONM N21 N27 VVOUT 625
HHOPN N22 N28 VVOUT 62500
.MODEL MDTH D IS=1E-8 KF=2.664234E-16 CJO=10F
F2PP N19 VEE POLY(2) VVCOPYP VVP 0 0 0 0 0.5 
F2PN N19 VEE POLY(2) VVCOPYP VVN 0 0 0 0 0.5
.ENDS ;TS46x