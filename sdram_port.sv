// Wrap SDRAM module with SystemVerilog port interfaces for easier integration

module sdram_port #(
    parameter CLOCK_SPEED_MHZ = 0,

    parameter BURST_LENGTH = 1,
    parameter BURST_TYPE = 0,
    parameter WRITE_BURST = 0,
    parameter CAS_LATENCY = 2,
    parameter DATA_WIDTH = 16,
    parameter ROW_WIDTH = 13,
    parameter COL_WIDTH = 10,
    parameter PRECHARGE_BIT = 10,
    parameter BANK_WIDTH = 2,
    parameter DQM_WIDTH = 2,

    parameter PORT_ADDR_WIDTH = 25,
    parameter PORT_BURST_LENGTH = BURST_LENGTH,
    parameter PORT_OUTPUT_WIDTH = PORT_BURST_LENGTH * DATA_WIDTH,

    parameter SETTING_INHIBIT_DELAY_MICRO_SEC = 100,
    parameter SETTING_T_CK_MIN_CLOCK_CYCLE_TIME_NANO_SEC = 6,
    parameter SETTING_T_RAS_MIN_ROW_ACTIVE_TIME_NANO_SEC = 48,
    parameter SETTING_T_RC_MIN_ROW_CYCLE_TIME_NANO_SEC = 60,
    parameter SETTING_T_RP_MIN_PRECHARGE_CMD_PERIOD_NANO_SEC = 18,
    parameter SETTING_T_RFC_MIN_AUTOREFRESH_PERIOD_NANO_SEC = 80,
    parameter SETTING_T_RC_MIN_ACTIVE_TO_ACTIVE_PERIOD_NANO_SEC = 60,
    parameter SETTING_T_RCD_MIN_READ_WRITE_DELAY_NANO_SEC = 18,
    parameter SETTING_T_WR_MIN_WRITE_AUTO_PRECHARGE_RECOVERY_NANO_SEC = 15,
    parameter SETTING_T_MRD_MIN_LOAD_MODE_CLOCK_CYCLES = 2,
    parameter SETTING_REFRESH_TIMER_NANO_SEC = 7500,
    parameter SETTING_USE_FAST_INPUT_REGISTER = 1
) (
    input wire clk,
    input wire sdram_clk,
    input wire reset,  // Used to trigger start of FSM
    output wire init_complete,  // SDRAM is done initializing

    // Port
    sdram_port_if.controller port,

    inout  wire [DATA_WIDTH-1:0] SDRAM_DQ,    // Bidirectional data bus
    output wire  [ROW_WIDTH-1:0] SDRAM_A,     // Address bus
    output wire  [DQM_WIDTH-1:0] SDRAM_DQM,   // High/low byte mask
    output wire  [BANK_WIDTH-1:0] SDRAM_BA,    // Bank select (single bits)
    output wire        SDRAM_nCS,   // Chip select, neg triggered
    output wire        SDRAM_nWE,   // Write enable, neg triggered
    output wire        SDRAM_nRAS,  // Select row address, neg triggered
    output wire        SDRAM_nCAS,  // Select column address, neg triggered
    output wire         SDRAM_CKE,   // Clock enable
    output wire        SDRAM_CLK    // Chip clock
);
    
    sdram #(
        .CLOCK_SPEED_MHZ(CLOCK_SPEED_MHZ),

        .BURST_LENGTH(BURST_LENGTH),
        .BURST_TYPE(BURST_TYPE),
        .WRITE_BURST(WRITE_BURST),
        .CAS_LATENCY(CAS_LATENCY),
        .DATA_WIDTH(DATA_WIDTH),
        .ROW_WIDTH(ROW_WIDTH),
        .COL_WIDTH(COL_WIDTH),
        .PRECHARGE_BIT(PRECHARGE_BIT),
        .BANK_WIDTH(BANK_WIDTH),
        .DQM_WIDTH(DQM_WIDTH),

        .PORT_ADDR_WIDTH(PORT_ADDR_WIDTH),
        .PORT_BURST_LENGTH(PORT_BURST_LENGTH),
        .PORT_OUTPUT_WIDTH(PORT_OUTPUT_WIDTH),

        .SETTING_INHIBIT_DELAY_MICRO_SEC(SETTING_INHIBIT_DELAY_MICRO_SEC),
        .SETTING_T_CK_MIN_CLOCK_CYCLE_TIME_NANO_SEC(SETTING_T_CK_MIN_CLOCK_CYCLE_TIME_NANO_SEC),
        .SETTING_T_RAS_MIN_ROW_ACTIVE_TIME_NANO_SEC(SETTING_T_RAS_MIN_ROW_ACTIVE_TIME_NANO_SEC),
        .SETTING_T_RC_MIN_ROW_CYCLE_TIME_NANO_SEC(SETTING_T_RC_MIN_ROW_CYCLE_TIME_NANO_SEC),
        .SETTING_T_RP_MIN_PRECHARGE_CMD_PERIOD_NANO_SEC(SETTING_T_RP_MIN_PRECHARGE_CMD_PERIOD_NANO_SEC),
        .SETTING_T_RFC_MIN_AUTOREFRESH_PERIOD_NANO_SEC(SETTING_T_RFC_MIN_AUTOREFRESH_PERIOD_NANO_SEC),
        .SETTING_T_RC_MIN_ACTIVE_TO_ACTIVE_PERIOD_NANO_SEC(SETTING_T_RC_MIN_ACTIVE_TO_ACTIVE_PERIOD_NANO_SEC),
        .SETTING_T_RCD_MIN_READ_WRITE_DELAY_NANO_SEC(SETTING_T_RCD_MIN_READ_WRITE_DELAY_NANO_SEC),
        .SETTING_T_WR_MIN_WRITE_AUTO_PRECHARGE_RECOVERY_NANO_SEC(SETTING_T_WR_MIN_WRITE_AUTO_PRECHARGE_RECOVERY_NANO_SEC),
        .SETTING_T_MRD_MIN_LOAD_MODE_CLOCK_CYCLES(SETTING_T_MRD_MIN_LOAD_MODE_CLOCK_CYCLES),
        .SETTING_REFRESH_TIMER_NANO_SEC(SETTING_REFRESH_TIMER_NANO_SEC),
        .SETTING_USE_FAST_INPUT_REGISTER(SETTING_USE_FAST_INPUT_REGISTER)
    ) sdram_inst (
        .clk(clk),
        .sdram_clk(sdram_clk),
        .reset(reset),
        .init_complete(init_complete),

        // Port
        .port_addr(port.addr),
        .port_data(port.data),
        .port_byte_en(port.byte_en),
        .port_q(port.q),
        .port_wr(port.wr),
        .port_rd(port.rd),
        .port_available(port.available),
        .port_ready(port.ready),

        .SDRAM_DQ(SDRAM_DQ),
        .SDRAM_A(SDRAM_A),
        .SDRAM_DQM(SDRAM_DQM),
        .SDRAM_BA(SDRAM_BA),
        .SDRAM_nCS(SDRAM_nCS),
        .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS),
        .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_CLK(SDRAM_CLK)
    );


endmodule