interface sdram_port_if #(
    parameter PORT_ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 16,
    parameter DQM_WIDTH = 2,
    parameter PORT_OUTPUT_WIDTH = DATA_WIDTH * 2
);

    logic clk;

    logic [PORT_ADDR_WIDTH-1:0] addr;
    logic [DATA_WIDTH-1:0] data;
    logic [DQM_WIDTH-1:0] byte_en;    // Byte enable for writes
    logic [PORT_OUTPUT_WIDTH-1:0] q;

    logic wr;
    logic rd;

    logic available;                 // The port is able to be used
    logic ready;                      // The port has finished its task. Will rise for a single cycle

    modport controller (
        input clk,
        
        input addr,
        input data,
        input byte_en,
        output q,

        input wr,
        input rd,

        output available,
        output ready
    );

    modport client (
        output clk,

        output addr,
        output data,
        output byte_en,
        input q,

        output wr,
        output rd,

        input available,
        input ready
    );

endinterface: sdram_port_if
