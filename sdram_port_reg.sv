module sdram_port_reg  (
    input clk_i,
    input clk_logic,
    input reset_i,

    sdram_port_if.controller in,

    output active_o,
    sdram_port_if.client prev,

    input next_active_i,
    sdram_port_if.controller next
);

    logic [$bits(in.addr)-1:0] addr_r;
    logic [$bits(in.data)-1:0] data_r;
    logic [$bits(in.byte_en)-1:0] byte_en_r;    // Byte enable for writes
    logic [$bits(in.q)-1:0] q_r;

    logic wr_r;
    logic rd_r;

    logic ready_set_r;
    logic ready_set_prev_r;
    logic ready_ack_r;

    assign in.ready = ready_ack_r;
    assign in.q = q_r;
    assign in.available = !active_o;

    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            addr_r <= 0;
            data_r <= 0;
            byte_en_r <= 0;
            wr_r <= 0;
            rd_r <= 0;
        end else begin
            addr_r <= in.addr;
            data_r <= in.data;
            byte_en_r <= in.byte_en;
            wr_r <= in.wr;
            rd_r <= in.rd;
        end
    end

    always_ff @(posedge clk_logic) begin
        if (reset_i) begin
            ready_ack_r <= 0;
        end else begin
            ready_ack_r <= 0;
            ready_set_prev_r <= ready_set_r;
            if (ready_set_r & !ready_set_prev_r) begin
                ready_ack_r <= 1;
                q_r <= port_q_queue_r;
            end
        end
    end

    logic port_wr_prev_r;
    logic port_rd_prev_r;
    wire port_wr_req_w = wr_r && !port_wr_prev_r;
    wire port_rd_req_w = rd_r && !port_rd_prev_r;
    wire port_req_w = port_wr_req_w || port_rd_req_w;

    logic port_wr_queue_r;
    logic port_rd_queue_r;
    reg [$bits(in.byte_en)-1:0] port_byte_en_queue_r;
    reg [$bits(in.addr)-1:0] port_addr_queue_r;
    reg [$bits(in.data)-1:0] port_data_queue_r;
    reg [$bits(in.q)-1:0] port_q_queue_r;

    wire activate_w = port_req_w & prev.available & !next_active_i & !activated_r;
    logic activated_r;
    wire active_w = activated_r /*| activate_w */;
    assign active_o = active_w;

    assign next.available = (port_req_w || active_w) & !next_active_i ? 0 : prev.available;
    assign next.q = next_active_i ? prev.q : 0;
    assign next.ready = next_active_i ? prev.ready : 0;

    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            port_wr_queue_r <= 0;
            port_rd_queue_r <= 0;
            port_byte_en_queue_r <= 0;
            port_addr_queue_r <= 0;
            port_data_queue_r <= 0;
            port_q_queue_r <= 0;
            activated_r <= 0;
        end else begin
            port_wr_prev_r <= wr_r;
            port_rd_prev_r <= rd_r;
            if (port_wr_req_w) begin
                port_wr_queue_r <= 1;
                port_byte_en_queue_r <= byte_en_r;
                port_addr_queue_r <= addr_r;
                port_data_queue_r <= data_r;
            end else if (port_rd_req_w) begin
                port_rd_queue_r <= 1;
                port_addr_queue_r <= addr_r;
            end
            if (ready_ack_r) begin
                ready_set_r <= 0;
            end
            if (activate_w) begin
                activated_r <= 1;
            end else if (activated_r) begin
                if (prev.ready) begin
                    port_q_queue_r <= prev.q;
                    port_rd_queue_r <= 0;
                    port_wr_queue_r <= 0;
                    activated_r <= 0;
                    ready_set_r <= 1;
                end
            end
        end
    end

    assign prev.clk = 0;
    assign prev.addr = active_w ? port_addr_queue_r : next.addr;
    assign prev.data = active_w ? port_data_queue_r : next.data;
    assign prev.byte_en = active_w ? port_byte_en_queue_r : next.byte_en;
    assign prev.wr = active_w ? port_wr_queue_r : next.wr;
    assign prev.rd = active_w ? port_rd_queue_r : next.rd;

endmodule