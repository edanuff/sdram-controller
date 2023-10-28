#!/bin/sh
rm sdram32_tb.vvp
rm sdram32_tb.log
rm sdram32_tb.vcd
iverilog -g2012 -o sdram32_tb.vvp ../sdram.sv mt48lc2m32b2.v sdram32_tb.sv
vvp -lsdram32_tb.log sdram32_tb.vvp
