`timescale 1ns / 1ns

module sdram_tb;

  reg clk = 1;
  reg reset = 0;

  wire init_complete;
  wire p0_ready;
  wire p0_available;

  reg [20:0] p0_addr = 0;
  reg [31:0] p0_data = 0;
  wire [31:0] p0_q;

  reg p0_wr_req = 0;
  reg p0_rd_req = 0;

  reg [3:0] p0_byte_en = 0;

  always begin
    #5 clk <= ~clk;
  end

  wire [31:0] dq;
  wire [31:0] dq_o;
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
      dq_o,
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
      .BURST_LENGTH(1),
      .P0_BURST_LENGTH(1)
  ) sdram (
      .clk(clk),
      .sdram_clk(!clk),
      .reset(reset),
      .init_complete(init_complete),

      // Port 0
      .p0_addr(p0_addr),
      .p0_data(p0_data),
      .p0_byte_en(p0_byte_en),
      .p0_q(p0_q),

      .p0_wr_req(p0_wr_req),
      .p0_rd_req(p0_rd_req),

      .p0_available(p0_available),
      .p0_ready(p0_ready),

      .SDRAM_DQ(dq_o),
      .SDRAM_DQ_o(dq),
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

   // VCD Dumping
    initial begin
        $dumpfile("sdram_tb.vcd");
        $dumpvars(0, sdram_tb);
    end

    always @(p0_rd_req, p0_wr_req, p0_addr, p0_data, p0_q, p0_ready, p0_available) begin
        $display("Time: %0t | Port 0 | RD: %b WR: %b ADDR: %h DIN: %h DOUT: %h READY: %b AVAIL: %b", $time, p0_rd_req, p0_wr_req, p0_addr, p0_data, p0_q, p0_ready, p0_available);
    end
    
    always @(sdram_cmd, dq_o, dq, addr, dqm, ba, cs_n, we_n, ras_n, cas_n, cke) begin
        $display("Time: %0t | CMD: %h SDRAM_DQ: %h SDRAM_DQ_o: %h SDRAM_A: %h SDRAM_DQM: %b SDRAM_BA: %b SDRAM_nCS: %b SDRAM_nWE: %b SDRAM_nRAS: %b SDRAM_nCAS: %b SDRAM_CKE: %b", $time, sdram_cmd, dq_o, dq, addr, dqm, ba, cs_n, we_n, ras_n, cas_n, cke);
    end
    
    wire rdq = sdram.p0_rd_queue;
    wire wrq = sdram.p0_wr_queue;
    always @(rdq, wrq) begin
        $display("Time: %0t | Port 0 | RD: %b WR: %b RDQ: %b WRQ: %b", $time, p0_rd_req, p0_wr_req, rdq, wrq);
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
    p0_addr = 21'h00_2020;
    p0_data = 32'h1234;

    p0_byte_en = 4'hf;
    p0_wr_req = 1;

    #20;
    p0_addr = 0;
    p0_data = 0;
    #20;
    p0_wr_req = 0;

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
