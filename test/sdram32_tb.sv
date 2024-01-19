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
  wire port_ready;
  wire port_available;

  reg [20:0] port_addr = 0;
  reg [31:0] port_data = 0;
  wire [31:0] port_q;

  reg port_wr_req = 0;
  reg port_rd_req = 0;

  reg [3:0] port_byte_en = 0;

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
      .PORT_ADDR_WIDTH(21),
      .PORT_OUTPUT_WIDTH(32),
      .CAS_LATENCY(2),
      .SETTING_REFRESH_TIMER_NANO_SEC(15000),
      .SETTING_T_WR_MIN_WRITE_AUTO_PRECHARGE_RECOVERY_NANO_SEC(16),
      .BURST_LENGTH(1),
      .PORT_BURST_LENGTH(1),
      .DATA_WIDTH(32),
      .ROW_WIDTH(11),
      .COL_WIDTH(8),
      .PRECHARGE_BIT(10),
      .DQM_WIDTH(4),
      .SETTING_USE_FAST_INPUT_REGISTER(1)
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

      .port_wr(port_wr_req),
      .port_rd(port_rd_req),

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

    always @(port_rd_req, port_wr_req, port_addr, port_data, port_q, port_ready, port_available) begin
        $display("Time: %0t | Port 0 | RD: %b WR: %b ADDR: %h DIN: %h DOUT: %h READY: %b AVAIL: %b", $time, port_rd_req, port_wr_req, port_addr, port_data, port_q, port_ready, port_available);
    end
    
    always @(sdram_cmd, dq, addr, dqm, ba, cs_n, we_n, ras_n, cas_n, cke) begin
        $display("Time: %0t | CMD: %h SDRAM_DQ: %h SDRAM_A: %h SDRAM_DQM: %b SDRAM_BA: %b SDRAM_nCS: %b SDRAM_nWE: %b SDRAM_nRAS: %b SDRAM_nCAS: %b SDRAM_CKE: %b", $time, sdram_cmd, dq, addr, dqm, ba, cs_n, we_n, ras_n, cas_n, cke);
    end
    
    wire rdq = sdram.port_rd_queue;
    wire wrq = sdram.port_wr_queue;
    always @(rdq, wrq) begin
        $display("Time: %0t | Port 0 | RD: %b WR: %b RDQ: %b WRQ: %b", $time, port_rd_req, port_wr_req, rdq, wrq);
    end
    
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
    port_addr = 21'h00_2020;
    port_data = 32'h1234;

    port_byte_en = 4'hf;
    port_wr_req = 1;

    #20;
    port_addr = 0;
    port_data = 0;
    #20;
    port_wr_req = 0;

    //@(posedge clk iff port_ready);
    wait(port_ready);
    #5;

    $display("Write 2 at %t", $time());
    port_addr   = 21'h00_2021;
    port_data   = 32'h5678;

    port_wr_req = 1;

    #20;
    port_addr   = 0;
    port_data   = 0;
    #20;
    port_wr_req = 0;

    //@(posedge clk iff port_ready);
    wait(port_ready);
    #5;

    $display("Write 3 at %t", $time());
    port_addr   = 21'h00_2022;
    port_data   = 32'h9ABC;

    port_wr_req = 1;

    #20;
    port_addr   = 0;
    port_data   = 0;
    port_wr_req = 0;

    //@(posedge clk iff port_ready);
    wait(port_ready);
    #5;

    $display("Write 4 at %t", $time());
    port_addr   = 21'h00_2023;
    port_data   = 32'hDEF0;

    port_wr_req = 1;

    #20;
    port_addr   = 0;
    port_data   = 0;
    port_wr_req = 0;

    //@(posedge clk iff port_ready);
    wait(port_ready);
    #5;

    $display("Write 5 at %t", $time());
    port_addr   = 21'h00_2024;
    port_data   = 32'hFEDC;

    port_wr_req = 1;

    #20;
    port_addr   = 0;
    port_data   = 0;
    port_wr_req = 0;

    //@(posedge clk iff port_ready);
    wait(port_ready);
    #5;

    $display("Write 6 at %t", $time());
    port_addr   = 21'h00_2025;
    port_data   = 32'hBA98;

    port_wr_req = 1;

    #20;
    port_addr   = 0;
    port_data   = 0;
    port_wr_req = 0;

    //@(posedge clk iff port_ready);
    wait(port_ready);
    #5;

    $display("Write 7 at %t", $time());
    port_addr   = 21'h00_2026;
    port_data   = 32'h7654;

    port_wr_req = 1;

    #20;
    port_addr   = 0;
    port_data   = 0;
    port_wr_req = 0;

    //@(posedge clk iff port_ready);
    wait(port_ready);
    #5;

    $display("Write 8 at %t", $time());
    port_addr   = 21'h00_2027;
    port_data   = 32'h3210;

    port_wr_req = 1;

    #20;
    port_addr   = 0;
    port_data   = 0;
    port_wr_req = 0;

    //@(posedge clk iff port_ready);
    wait(port_ready);
    #5;

    // Read
    $display("Read 1 at %t", $time());
    port_addr = 21'h00_2020;
    port_data = 32'hFFFF;

    port_byte_en = 2'h2;
    port_rd_req = 1;

    #20;
    port_addr   = 0;
    port_rd_req = 0;

    //@(posedge clk iff port_ready);
    wait(port_ready);
    #5;

    $display("Read 2 at %t", $time());
    port_addr = 21'h00_2021;
    port_data = 32'hFFFF;

    port_byte_en = 2'h2;
    port_rd_req = 1;

    #10;
    port_addr   = 0;
    port_rd_req = 0;

    wait(port_ready);

    wait(sdram.refresh_counter == 751);
    #5;
    $display("refresh 1 at %t", $time());

     //wait(port_available | port_ready);

    port_addr = 21'h00_2022;
    port_data = 32'hFFFF;

    port_byte_en = 2'h2;
    port_rd_req = 1;

    #10;
    port_addr   = 0;
    port_rd_req = 0;

    #1000;

    $finish;
  end

endmodule
