`timescale 1ns/1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    // AXI4-Lite read/write interface
    // Write channel
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    // Read channel
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata, 
    
    // data input(AXI-Stream)
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 

    // data output(AXI-Stream)
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
begin
    // parameter
    parameter DataLength = 32;
    parameter ConfigLength = 32;
    parameter S_AXIL_IDLE  = 2'd0;
    parameter S_AXIL_RADDR = 2'd1;
    parameter S_AXIL_RDATA = 2'd2;
    parameter S_AXIL_WRITE = 2'd3; //WRITE ADDR/DATA can do independently

    parameter S_AP_IDLE = 2'd0;
    parameter S_AP_PROC_FIRST = 2'd1;
    parameter S_AP_PROC_SECOND = 2'd2;
    parameter S_AP_DONE = 2'd3;
    wire ap_start;
    wire ap_done;
    wire ap_idle;
    wire finish_compute;
    
    // AXI4-Lite Declaration
    reg [1:0] AXIL_state, AXIL_state_next;
    reg [2:0] ap_config_r, ap_config_w;
    reg [DataLength-1:0] data_length_config_r, data_length_config_w;
    reg awready_r, awready_w;
    reg wready_r, wready_w;
    reg arready_r, arready_w;
    reg rvalid_r, rvalid_w;
    reg [pDATA_WIDTH-1:0] rdata_r, rdata_w;
    // wire wen_ctrl_axil;
    // AXI4-Stream state
    reg [1:0] AP_state, AP_state_next;
    reg ss_tready_w, ss_tready_r;
    reg sm_tvalid_w, sm_tvalid_r;
    reg [pDATA_WIDTH-1:0] sm_tdata_w, sm_tdata_r;
    reg sm_tlast_w, sm_tlast_r;
    wire last_data_transferred;
    // ram for tap
    reg tap_WE_w, tap_WE_r;
    reg tap_EN_w, tap_EN_r;
    reg [pDATA_WIDTH-1:0] tap_Di_w, tap_Di_r;
    reg [pADDR_WIDTH-1:0] tap_A_w, tap_A_r;

    // ram for data RAM
    reg [3:0] data_WE_w, data_WE_r;
    reg data_EN_w, data_EN_r;
    reg [pDATA_WIDTH-1:0] data_Di_w, data_Di_r;
    reg signed  [pADDR_WIDTH-1:0] data_A_w, data_A_r;
    wire wen_ctrl;
    wire wen_data_ctrl;
    wire finish_one_fir;

    //FIR Calculation Declaration
    reg [3:0] tap_counter_w, tap_counter_r;
    reg [3:0] data_pointer_w, data_pointer_r;
    reg finish_one_proc;

    //======= axi4-lite output =========
    assign awready = awready_r;
    assign wready = wready_r;
    assign rvalid = rvalid_r;
    assign rdata = rdata_r;
    assign arready = arready_r;
    //======= axi4-lite output =========
    assign ap_start = ap_config_r[0];
    assign ap_done = ap_config_r[1];
    assign ap_idle = ap_config_r[2];
    assign last_data_transferred = (AP_state == S_AP_PROC_SECOND) && sm_tvalid && sm_tlast;
    assign wen_ctrl = (AXIL_state == S_AXIL_WRITE) && awvalid && wvalid;
    assign tap_EN = 1;
    assign tap_WE = wen_ctrl && (awaddr[7:4] >= 4'd2) ? 4'b1111 : 0;
    assign tap_Di = wdata;
    assign tap_A = tap_A_w; //tap address

    assign data_EN = 1;
    assign wen_data_ctrl = ((AP_state == S_AP_PROC_FIRST) && ss_tvalid && ~sm_tvalid) || ((AP_state == S_AP_IDLE)&&wen_ctrl&&awaddr[7:4] >= 4'd2);
    assign data_WE = wen_data_ctrl ? 4'b1111 : 0;
    assign data_Di = (AP_state==S_AP_PROC_FIRST) ? ss_tdata: 0;
    assign data_A = data_A_w; //data address

    // AXI4-Stream
    // assign finish_one_proc == (AP_state == S_AP_PROC) && (tap_counter_r == 4'd0);
    assign ss_tready = ss_tready_r;
    assign sm_tvalid = sm_tvalid_r;
    assign sm_tdata = sm_tdata_r;
    assign sm_tlast = sm_tlast_r;
    assign finish_one_fir = (AP_state == S_AP_PROC_SECOND) && (tap_counter_r == 4'd11);
    // AXI4-Stream state transition
    always @(*) begin
        case (AP_state)
            S_AP_IDLE: begin
                if (ap_start) begin
                    AP_state_next = S_AP_PROC_FIRST;
                end
                else begin
                    AP_state_next = S_AP_IDLE;
                end
            end
            S_AP_PROC_FIRST: begin
                AP_state_next = S_AP_PROC_SECOND;
            end
            S_AP_PROC_SECOND: begin
                if (sm_tready && sm_tvalid && ss_tlast) begin
                    AP_state_next = S_AP_DONE;
                end
                else if (sm_tready && sm_tvalid) begin
                    AP_state_next = S_AP_PROC_FIRST;
                end
                else begin
                    AP_state_next = S_AP_PROC_SECOND;
                end
            end
            S_AP_DONE: begin
                // if (arvalid) begin
                    AP_state_next = S_AP_IDLE;
                // end
                // else begin
                    // AP_state_next = S_AP_DONE;
                // end
                    
            end
        endcase
    end
    always @(*) begin
        if (sm_tready && sm_tvalid) begin
            sm_tvalid_w = 0;
        end
        else if (AP_state == S_AP_PROC_SECOND && finish_one_fir && ~(sm_tready && sm_tvalid)) begin
            sm_tvalid_w = 1;
        end
        else begin
            sm_tvalid_w = 0;
        end
    end
    always @(*) begin
        if ((AP_state == S_AP_PROC_SECOND) && finish_one_fir && ~(sm_tready && sm_tvalid)) begin
            ss_tready_w = 1;
        end
        else begin
            ss_tready_w = 0;
        end
    end

    always @(*) begin
        if (AP_state == S_AP_DONE)begin
            sm_tlast_w = 0;
        end
        else if (AP_state == S_AP_PROC_SECOND && finish_one_fir && ~(sm_tready && sm_tvalid) && ss_tlast ) begin
            sm_tlast_w = 1;
        end
        else begin
            sm_tlast_w = sm_tlast_r;
        end
    end
    always @(*) begin
        if (AP_state != S_AP_PROC_SECOND || AP_state == S_AP_PROC_SECOND && sm_tvalid) begin
            sm_tdata_w = 0;
        end
        else if ((AP_state == S_AP_PROC_SECOND)) begin
            sm_tdata_w = sm_tdata_r + $signed(tap_Do) * $signed(data_Do);
        end
        else begin
            sm_tdata_w = sm_tdata_r;
        end
    end
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (~axis_rst_n) begin
            AP_state <= S_AP_IDLE;
            ss_tready_r <= 0;
            sm_tvalid_r <= 0;
            sm_tdata_r <= 0;
            sm_tlast_r <= 0;
        end
        
        else begin
            AP_state <= AP_state_next;
            ss_tready_r <= ss_tready_w;
            sm_tvalid_r <= sm_tvalid_w;
            sm_tdata_r <= sm_tdata_w;
            sm_tlast_r <= sm_tlast_w;
        end        
    end

    // =============tap address=============
    always @(*) begin
        if (AXIL_state == S_AXIL_WRITE && AP_state == S_AP_IDLE) begin
            tap_A_w = awaddr[7:0]-8'h20;
        end
        else if(AXIL_state==S_AXIL_RADDR && AP_state == S_AP_IDLE) begin
            tap_A_w = araddr[7:0]-8'h20;
        end
        else if (AP_state == S_AP_PROC_SECOND || AP_state ==S_AP_PROC_FIRST) begin
            tap_A_w = tap_counter_r <<2;
        end
        else begin 
            tap_A_w = tap_A_r;
        end
    end
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (~axis_rst_n) begin
            tap_A_r <= 0;
        end
        else begin
            tap_A_r <= tap_A_w;
        end       
        
    end
    // =============tap address=============
    // =============data address=============
    always @(*) begin
        if (AP_state == S_AP_IDLE && wen_data_ctrl) begin
            data_A_w = awaddr[7:0]-8'h20;
        end
        // else if (wen_data_ctrl) begin
        //     data_A_w = data_pointer_w <<2;
        // end
        else begin //read/write in S_AP_PROC
            data_A_w = data_pointer_r <<2;
        end
    end

    //==============tap counter================
    always @(*) begin
        
        if (sm_tvalid) begin
            tap_counter_w =0;
        end
        else if (AP_state==S_AP_PROC_FIRST || AP_state == S_AP_PROC_SECOND ) begin
            tap_counter_w = tap_counter_r + 1;
        end
        else begin
            tap_counter_w = 0;
        end
    end
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (~axis_rst_n) begin
            tap_counter_r <= 0;
        end
        else begin
            tap_counter_r <= tap_counter_w;
        end       
    end
    //==============tap counter================

    //==============data pointer================
    reg [3:0] data_pointer_tmp;

    always @(*) begin
        data_pointer_tmp = data_pointer_r+1;
        if (AP_state == S_AP_IDLE) begin
            data_pointer_w = 0;
        end
        else if(tap_counter_r <= 4'd9)begin
            if (data_pointer_tmp == 4'd11) begin
                data_pointer_w = 0;
            end
            else begin
                data_pointer_w = data_pointer_tmp;
            end
        end
        else begin
            data_pointer_w = data_pointer_r;
        end
    end

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (~axis_rst_n) begin
            data_pointer_r <= 0;
        end
        else begin
            data_pointer_r <= data_pointer_w;
        end       
    end
    // ================================= AXI4-lite =================================
    always @(*) begin
        arready_w = (AXIL_state == S_AXIL_RADDR);
        rvalid_w = (AXIL_state == S_AXIL_RDATA);
        awready_w = (AXIL_state == S_AXIL_WRITE) && (awvalid) && ~awready_r;
        wready_w = (AXIL_state == S_AXIL_WRITE) && (wvalid) && ~wready_r;
        // rdata_w = (AXIL_state == S_AXIL_RDATA) ? tap_Do : 32'h0000_0000;
        
    end

    always @(*) begin
        if (AXIL_state == S_AXIL_RDATA ) begin
            if (araddr == 32'h0000) begin
                rdata_w = ap_config_r;
            end
            else if (araddr == 32'h00010) begin
                rdata_w = data_length_config_r;
            end
            else if (araddr >= 32'h0020) begin
                rdata_w = tap_Do;
            end
            else begin
                rdata_w = 0;
            end
            
        end
        else begin
            rdata_w = rdata_r;
        end
    end
    // =============AXIL_state=============
    // assign wen_ctrl_axil = awvalid && wvalid;
    always @(*) begin
        case (AXIL_state)
            S_AXIL_IDLE: begin
                if (awvalid) begin
                    AXIL_state_next = S_AXIL_WRITE;
                end
                else if (arvalid) begin
                    AXIL_state_next = S_AXIL_RADDR;
                end
                else begin
                    AXIL_state_next = S_AXIL_IDLE;
                end
            end
            S_AXIL_RADDR: begin
                if (arready_w) begin
                    AXIL_state_next = S_AXIL_RDATA;
                end
                else begin
                    AXIL_state_next = S_AXIL_RADDR;
                end
            end
            S_AXIL_RDATA: begin
                if (rready && rvalid) begin
                    AXIL_state_next = S_AXIL_IDLE;
                end
                else begin
                    AXIL_state_next = S_AXIL_RDATA;
                end
            end
            S_AXIL_WRITE: begin
                if (wready) begin
                    AXIL_state_next = S_AXIL_IDLE;
                end
                else begin
                    AXIL_state_next = S_AXIL_WRITE;
                end
            end
                
        endcase
    end

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (~axis_rst_n) begin
            arready_r <= 0;
            rvalid_r <= 0;
            awready_r <= 0; 
            wready_r <= 0;
            rdata_r <= 0;
        end
        else begin
            arready_r <= arready_w;
            rvalid_r <= rvalid_w;
            awready_r <= awready_w;
            wready_r <= wready_w;
            rdata_r <= rdata_w;
        end
    end
    
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (~axis_rst_n) begin
            AXIL_state <= S_AXIL_IDLE;
        end
        else begin
            AXIL_state <= AXIL_state_next;
        end
    end 
    // =============ap_config_w=============
    always @(*) begin
        if (AXIL_state == S_AXIL_WRITE && awaddr == 32'h0000_0000 && wdata[0]==1) begin // ap_start == 1
            ap_config_w = 3'b001; 
        end
        else if (AP_state == S_AP_PROC_SECOND && last_data_transferred) begin
            ap_config_w = 3'b110;
        end
        else if (AXIL_state == S_AXIL_RDATA && araddr == 32'h0000_0000 && ap_config_r[1] == 1) begin
            ap_config_w = 3'b100;
        end
        else if (AP_state != S_AP_IDLE) begin
            ap_config_w[0] = 0; // ap_start == 1 when processing data
            ap_config_w[1] = ap_config_r[1]; 
            ap_config_w[2] = ap_config_r[2]; 
        end
        else begin
            ap_config_w = ap_config_r;
        end
    end
    // =============data_length=============
    always @(*) begin
        if (AXIL_state == S_AXIL_WRITE&& wen_ctrl && awaddr == 32'h0000_0010)begin
            data_length_config_w = wdata;
        end
        else begin
            data_length_config_w = data_length_config_r;
        end
    end

    always @(posedge axis_clk) begin
        data_length_config_r <= data_length_config_w;
    end
    // =============data_length=============

    // =============ap_config_r=============
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (~axis_rst_n) begin
            ap_config_r <= 3'b100;
        end
        else begin
            ap_config_r <= ap_config_w;
        end
    end
     // =============ap_config_r=============
    // ================================= AXI4-lite =================================

    
    

end
    endmodule