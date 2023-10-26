#!/bin/sh
rm sdram_tb.vvp
rm sdram_tb.log
rm sdram_tb.vcd
iverilog -g2012 -o sdram_tb.vvp ../sdram.sv mt48lc2m32b2.v sdram_tb.sv
vvp -lsdram_tb.log sdram_tb.vvp
