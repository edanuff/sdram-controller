`timescale 1ns / 1ns

module sdram32_tb;

  // VCD Dumping
  initial begin
    $dumpfile("sdram32_tb.vcd");
    $dumpvars(0, sdram32_tb);
  end

  reg clk = 1;
  reg reset = 0;

  wire init_complete;

  parameter NUM_PORTS = 1;
  parameter PORT_ADDR_WIDTH = 21;
  parameter DATA_WIDTH = 32;
  parameter DQM_WIDTH = 4;
  parameter PORT_OUTPUT_WIDTH = 32;

  wire [PORT_ADDR_WIDTH-1:0] port_addr [NUM_PORTS-1:0];
  wire [DATA_WIDTH-1:0] port_data [NUM_PORTS-1:0];
  wire [DQM_WIDTH-1:0] port_byte_en [NUM_PORTS-1:0];
  wire [PORT_OUTPUT_WIDTH-1:0] port_q [NUM_PORTS-1:0];

  wire port_wr [NUM_PORTS-1:0];
  wire port_rd [NUM_PORTS-1:0];

  wire port_available [NUM_PORTS-1:0];
  wire  port_ready [NUM_PORTS-1:0];

  reg [20:0] p0_addr = 0;
  reg [31:0] p0_data = 0;
  reg [3:0] p0_byte_en = 0;
  wire [31:0] p0_q;

  reg p0_wr_req = 0;
  reg p0_rd_req = 0;

  wire p0_available;
  wire p0_ready;

  assign port_addr[0] = p0_addr;
  assign port_data[0] = p0_data;
  assign port_byte_en[0] = p0_byte_en;
  assign p0_q = port_q[0];

  assign port_wr[0] = p0_wr_req;
  assign port_rd[0] = p0_rd_req;

  assign p0_available = port_available[0];
  assign p0_ready = port_ready[0];

  always begin
    #5 clk <= ~clk;
  end

  wire [31:0] dq;
  wire [10:0] addr;
  wire [3:0] dqm;
  wire [1:0] ba;
  wire cs_n;
  wire we_n;
  wire ras_n;
  wire cas_n;
  wire cke;
  wire chip_clk;

  wire [2:0] sdram_cmd = {ras_n, cas_n, we_n};

  mt48lc2m32b2 sdram0 (
      dq,
      addr,
      ba,
      chip_clk,
      cke,
      cs_n,
      ras_n,
      cas_n,
      we_n,
      dqm
  );

  sdram #(
      .CLOCK_SPEED_MHZ(100),
      .NUM_PORTS(1),
      .PORT_ADDR_WIDTH(PORT_ADDR_WIDTH),
      .PORT_OUTPUT_WIDTH(PORT_OUTPUT_WIDTH),
      .CAS_LATENCY(2),
      .SETTING_REFRESH_TIMER_NANO_SEC(15000),
      .SETTING_T_WR_MIN_WRITE_AUTO_PRECHARGE_RECOVERY_NANO_SEC(16),
      .BURST_LENGTH(1),
      .PORT_BURST_LENGTH(1),
      .DATA_WIDTH(DATA_WIDTH),
      .ROW_WIDTH(11),
      .COL_WIDTH(9),
      .PRECHARGE_BIT(10),
      .DQM_WIDTH(DQM_WIDTH)
  ) sdram (
      .clk(clk),
      .sdram_clk(!clk),
      .reset(reset),
      .init_complete(init_complete),

      // Port 0
      .port_addr(port_addr),
      .port_data(port_data),
      .port_byte_en(port_byte_en),
      .port_q(port_q),

      .port_wr(port_wr),
      .port_rd(port_rd),

      .port_available(port_available),
      .port_ready(port_ready),

      .SDRAM_DQ(dq),
      .SDRAM_A(addr),
      .SDRAM_DQM(dqm),
      .SDRAM_BA(ba),
      .SDRAM_nCS(cs_n),
      .SDRAM_nWE(we_n),
      .SDRAM_nRAS(ras_n),
      .SDRAM_nCAS(cas_n),
      .SDRAM_CKE(cke),
      .SDRAM_CLK(chip_clk)
    );

    always @(p0_rd_req, p0_wr_req, p0_addr, p0_data, p0_q, p0_ready, p0_available) begin
        $display("Time: %0t | Port 0 | RD: %b WR: %b ADDR: %h DIN: %h DOUT: %h READY: %b AVAIL: %b", $time, p0_rd_req, p0_wr_req, p0_addr, p0_data, p0_q, p0_ready, p0_available);
    end
    
    always @(sdram_cmd, dq, addr, dqm, ba, cs_n, we_n, ras_n, cas_n, cke) begin
        $display("Time: %0t | CMD: %h SDRAM_DQ: %h SDRAM_A: %h SDRAM_DQM: %b SDRAM_BA: %b SDRAM_nCS: %b SDRAM_nWE: %b SDRAM_nRAS: %b SDRAM_nCAS: %b SDRAM_CKE: %b", $time, sdram_cmd, dq, addr, dqm, ba, cs_n, we_n, ras_n, cas_n, cke);
    end
    
    /*
    wire rdq = sdram.port_rd_queue[0];
    wire wrq = sdram.port_wr_queue[0];
    always @(rdq, wrq) begin
        $display("Time: %0t | Port 0 | RD: %b WR: %b RDQ: %b WRQ: %b", $time, p0_rd_req, p0_wr_req, rdq, wrq);
    end
   */

  initial begin
    reset = 1;

    #40;

    reset = 0;

    // #100000;
    //@(posedge clk iff init_complete);
    wait (init_complete);
    $display("Init complete at %t", $time());

    #25;

    wait (clk==0);


    $display("Write 1 at %t", $time());
    p0_addr = 21'h00_2020;
    p0_data = 32'h1234;

    p0_byte_en = 4'hf;
    p0_wr_req = 1;
    $display("Write 1a");

    #20;
    $display("Write 1b");
    p0_addr = 0;
    p0_data = 0;
    #20;
    p0_wr_req = 0;
    $display("Write 1c");

    //@(posedge clk iff p0_ready);
    wait(p0_ready);
    #5;

    $display("Write 2 at %t", $time());
    p0_addr   = 21'h00_2021;
    p0_data   = 32'h5678;

    p0_wr_req = 1;

    #20;
    p0_addr   = 0;
    p0_data   = 0;
    #20;
    p0_wr_req = 0;

    //@(posedge clk iff p0_ready);
    wait(p0_ready);
    #5;

    $display("Write 3 at %t", $time());
    p0_addr   = 21'h00_2022;
    p0_data   = 32'h9ABC;

    p0_wr_req = 1;

    #20;
    p0_addr   = 0;
    p0_data   = 0;
    p0_wr_req = 0;

    //@(posedge clk iff p0_ready);
    wait(p0_ready);
    #5;

    $display("Write 4 at %t", $time());
    p0_addr   = 21'h00_2023;
    p0_data   = 32'hDEF0;

    p0_wr_req = 1;

    #20;
    p0_addr   = 0;
    p0_data   = 0;
    p0_wr_req = 0;

    //@(posedge clk iff p0_ready);
    wait(p0_ready);
    #5;

    $display("Write 5 at %t", $time());
    p0_addr   = 21'h00_2024;
    p0_data   = 32'hFEDC;

    p0_wr_req = 1;

    #20;
    p0_addr   = 0;
    p0_data   = 0;
    p0_wr_req = 0;

    //@(posedge clk iff p0_ready);
    wait(p0_ready);
    #5;

    $display("Write 6 at %t", $time());
    p0_addr   = 21'h00_2025;
    p0_data   = 32'hBA98;

    p0_wr_req = 1;

    #20;
    p0_addr   = 0;
    p0_data   = 0;
    p0_wr_req = 0;

    //@(posedge clk iff p0_ready);
    wait(p0_ready);
    #5;

    $display("Write 7 at %t", $time());
    p0_addr   = 21'h00_2026;
    p0_data   = 32'h7654;

    p0_wr_req = 1;

    #20;
    p0_addr   = 0;
    p0_data   = 0;
    p0_wr_req = 0;

    //@(posedge clk iff p0_ready);
    wait(p0_ready);
    #5;

    $display("Write 8 at %t", $time());
    p0_addr   = 21'h00_2027;
    p0_data   = 32'h3210;

    p0_wr_req = 1;

    #20;
    p0_addr   = 0;
    p0_data   = 0;
    p0_wr_req = 0;

    //@(posedge clk iff p0_ready);
    wait(p0_ready);
    #5;

    // Read
    $display("Read 1 at %t", $time());
    p0_addr = 21'h00_2020;
    p0_data = 32'hFFFF;

    p0_byte_en = 2'h2;
    p0_rd_req = 1;

    #20;
    p0_addr   = 0;
    p0_rd_req = 0;

    //@(posedge clk iff p0_ready);
    wait(p0_ready);
    #5;

    $display("Read 2 at %t", $time());
    p0_addr = 21'h00_2021;
    p0_data = 32'hFFFF;

    p0_byte_en = 2'h2;
    p0_rd_req = 1;

    #10;
    p0_addr   = 0;
    p0_rd_req = 0;

    wait(p0_ready);

    wait(sdram.refresh_counter == 751);
    #5;
    $display("refresh 1 at %t", $time());

     //wait(p0_available | p0_ready);

    p0_addr = 21'h00_2022;
    p0_data = 32'hFFFF;

    p0_byte_en = 2'h2;
    p0_rd_req = 1;

    #10;
    p0_addr   = 0;
    p0_rd_req = 0;

    #1000;

    $finish;
  end

endmodule
