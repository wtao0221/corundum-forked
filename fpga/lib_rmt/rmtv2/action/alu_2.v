/****************************************************/
//	Module name: alu_2
//	Authority @ yangxiangrui (yangxiangrui11@nudt.edu.cn)
//	Last edited time: 2020/09/23
//	Function outline: 2st type ALU (with load/store) module in RMT
/****************************************************/

`timescale 1ns / 1ps

module alu_2 #(
    parameter STAGE_ID = 0,
    parameter ACTION_LEN = 25,
    parameter DATA_WIDTH = 32,  //data width of the ALU
    parameter ACTION_ID = 3,

	parameter C_S_AXIS_DATA_WIDTH = 512,
	parameter C_S_AXIS_TUSER_WIDTH = 128
)
(
    input clk,
    input rst_n,

    //input from sub_action
    input [ACTION_LEN-1:0]            action_in,
    input                             action_valid,
    input [DATA_WIDTH-1:0]            operand_1_in,
    input [DATA_WIDTH-1:0]            operand_2_in,
    input [DATA_WIDTH-1:0]            operand_3_in,

    //input vlan id, we need this for isolation
    input [11:0]                       vlan_id,

    //output to form PHV
    output reg [DATA_WIDTH-1:0]       container_out,
    output reg                        container_out_valid,

    //control path
    input [C_S_AXIS_DATA_WIDTH-1:0]			c_s_axis_tdata,
	input [C_S_AXIS_TUSER_WIDTH-1:0]		c_s_axis_tuser,
	input [C_S_AXIS_DATA_WIDTH/8-1:0]		c_s_axis_tkeep,
	input									c_s_axis_tvalid,
	input									c_s_axis_tlast,

    output reg [C_S_AXIS_DATA_WIDTH-1:0]		c_m_axis_tdata,
	output reg [C_S_AXIS_TUSER_WIDTH-1:0]		c_m_axis_tuser,
	output reg [C_S_AXIS_DATA_WIDTH/8-1:0]		c_m_axis_tkeep,
	output reg								    c_m_axis_tvalid,
	output reg								    c_m_axis_tlast

);

/********intermediate variables declared here********/
localparam width_6B = 48;
localparam width_4B = 32;
localparam width_2B = 16;


reg  [3:0]           action_type, action_type_r;

//regs for RAM access
reg                         store_en, store_en_r;
reg  [4:0]                  store_addr, store_addr_r;
reg  [31:0]                 store_din,	 store_din_r;

wire [31:0]                 load_data;
wire [4:0]                  load_addr;

reg  [2:0]                  alu_state, alu_state_next;
reg [DATA_WIDTH-1:0]		container_out_r;
reg							container_out_valid_next;


//checkme: regs/wires for isolation
wire [7:0]                  base_addr;
wire [7:0]                  addr_len;

reg                         overflow, overflow_r;

//checkme: this is for load/loadd
reg [4:0]                   op2_imm, op2_imm_r;


/********intermediate variables declared here********/

//support tenant isolation
assign load_addr = store_addr[4:0] + base_addr;

/*
7 operations to support:

1,2. add/sub:   0001/0010
              extract 2 operands from pkt header, add(sub) and write back.

3,4. addi/subi: 1001/1010
              extract op1 from pkt header, op2 from action, add(sub) and write back.

5: load:      0101
              load data from RAM, write to pkt header according to addr in action.

6. store:     0110
              read data from pkt header, write to ram according to addr in action.

7. loadd:     0111
              load data from RAM, increment by 1 write it to container, and write it
              back to the RAM. 
*/

localparam  IDLE_S = 3'd0,
            EMPTY1_S = 3'd1,
            OB_ADDR_S = 3'd2,
            EMPTY2_S = 3'd3,
            OUTPUT_S = 3'd4;

always @(*) begin
	alu_state_next = alu_state;

	action_type_r = action_type;
	container_out_r = container_out;
	container_out_valid_next = 0;

	store_en_r = store_en;
	store_addr_r = store_addr;
	store_din_r = store_din;
    op2_imm_r = op2_imm;
    overflow_r = overflow;
	
	case (alu_state)
		IDLE_S: begin
            //reset all store signals
            store_addr_r = 0;
            store_din_r = 0;
            store_en_r = 0;
            overflow_r = 0;
            container_out_r = 0;
			
            if (action_valid) begin
				action_type_r = action_in[24:21];
                overflow_r = 0;
                op2_imm_r = operand_2_in[4:0];
				alu_state_next = EMPTY1_S;
                
                case(action_in[24:21])
                    //add/addi ops 
                    4'b0001, 4'b1001: begin
                        container_out_r = operand_1_in + operand_2_in;
                    end 
                    //sub/subi ops
                    4'b0010, 4'b1010: begin
                        container_out_r = operand_1_in - operand_2_in;
                    end
                    //store op (interact with RAM)
                    4'b1000: begin
                        container_out_r = operand_3_in;
                        //store_en_r = 1;
                        store_addr_r = operand_2_in[4:0];
                        store_din_r = operand_1_in;
                    end
                    //load op (interact with RAM)
                    4'b1011: begin
						//checkme
                        container_out_r = operand_3_in;
                    end
                    //checkme: loadd operation added
                    4'b0111: begin
                        // do nothing now
                        //checkme
                        container_out_r = operand_3_in;
                        store_addr_r = operand_2_in[4:0];
                    end
                    //cannot go back to IDLE since this
                    //might be a legal action.
                    default: begin
                        container_out_r = operand_3_in;
                    end
				endcase
			end
		end
		EMPTY1_S: begin
			// an empty cycle waiting for page table result
            alu_state_next = OB_ADDR_S;
        end
        OB_ADDR_S: begin
            //ok, if its `load` op, needs to check overflow.
            if(action_type == 4'b1011 || action_type == 4'b0111 || action_type == 4'b1000) begin
                if(op2_imm > addr_len) begin
                    overflow_r = 1'b1;
                end
                else begin
                    overflow_r = 1'b0;
                    //checkme: its the right time to write for `store`
                    if(action_type == 4'b1000) begin
                        store_addr_r = base_addr + op2_imm;
                        //store_din_r = operand_1_in;
                        store_en_r = 1'b1;
                    end
                end
            end
            else begin
                //I think we shoud do nothing, no?
            end
            alu_state_next = EMPTY2_S;
        end
        EMPTY2_S: begin
            //wait for the result of RAM
            alu_state_next = OUTPUT_S;
            store_en_r = 1'b0;
        end
        OUTPUT_S: begin
			// load data is ready
            // output the value
			container_out_valid_next = 1;
            case(action_type)
                //load
                4'b1011: container_out_r = load_data;
                //loadd
                4'b0111: begin
                    container_out_r = load_data + 32'b1;
                    //checkme: we need to write the data back
                    store_en_r = 1'b1;
                    store_din_r = load_data + 32'b1;
                end
                default:  ;//nothing to do;
            endcase

            alu_state_next = IDLE_S;
        end
	endcase
end

always @(posedge clk) begin
	if (~rst_n) begin
		alu_state <= IDLE_S;

		action_type <= 0;
		container_out <= 0;
		container_out_valid <= 0;

		store_en <= 0;
		store_addr <= 0;
		store_din <= 0;

        overflow <= 0;
        op2_imm <= 5'b0;
	end
	else begin
		alu_state <= alu_state_next;
		action_type <= action_type_r;
		container_out <= container_out_r;
		container_out_valid <= container_out_valid_next;

		store_en <= store_en_r;
		store_addr <= store_addr_r;
		store_din <= store_din_r;

        overflow <= overflow_r;
        op2_imm <= op2_imm_r;
	end
end




/*
    checkme: CONTROL PATH
*/

/****control path for 512b*****/
wire [7:0]          mod_id; //module ID
wire [15:0]         control_flag; //dst udp port num
reg  [7:0]          c_index; //table index(addr)
reg                 c_wr_en; //enable table write(wen)

reg  [2:0]          c_state;

localparam IDLE_C = 1,
           WRITE_C = 2;

assign mod_id = c_s_axis_tdata[368+:8];
assign control_flag = c_s_axis_tdata[335:320];

//LE to BE switching
wire[C_S_AXIS_DATA_WIDTH-1:0] c_s_axis_tdata_swapped;
assign c_s_axis_tdata_swapped = {	c_s_axis_tdata[0+:8],
									c_s_axis_tdata[8+:8],
									c_s_axis_tdata[16+:8],
									c_s_axis_tdata[24+:8],
									c_s_axis_tdata[32+:8],
									c_s_axis_tdata[40+:8],
									c_s_axis_tdata[48+:8],
									c_s_axis_tdata[56+:8],
									c_s_axis_tdata[64+:8],
									c_s_axis_tdata[72+:8],
									c_s_axis_tdata[80+:8],
									c_s_axis_tdata[88+:8],
									c_s_axis_tdata[96+:8],
									c_s_axis_tdata[104+:8],
									c_s_axis_tdata[112+:8],
									c_s_axis_tdata[120+:8],
									c_s_axis_tdata[128+:8],
									c_s_axis_tdata[136+:8],
									c_s_axis_tdata[144+:8],
									c_s_axis_tdata[152+:8],
									c_s_axis_tdata[160+:8],
									c_s_axis_tdata[168+:8],
									c_s_axis_tdata[176+:8],
									c_s_axis_tdata[184+:8],
									c_s_axis_tdata[192+:8],
									c_s_axis_tdata[200+:8],
									c_s_axis_tdata[208+:8],
									c_s_axis_tdata[216+:8],
									c_s_axis_tdata[224+:8],
									c_s_axis_tdata[232+:8],
									c_s_axis_tdata[240+:8],
									c_s_axis_tdata[248+:8],
                                    c_s_axis_tdata[256+:8],
                                    c_s_axis_tdata[264+:8],
                                    c_s_axis_tdata[272+:8],
                                    c_s_axis_tdata[280+:8],
                                    c_s_axis_tdata[288+:8],
                                    c_s_axis_tdata[296+:8],
                                    c_s_axis_tdata[304+:8],
                                    c_s_axis_tdata[312+:8],
                                    c_s_axis_tdata[320+:8],
                                    c_s_axis_tdata[328+:8],
                                    c_s_axis_tdata[336+:8],
                                    c_s_axis_tdata[344+:8],
                                    c_s_axis_tdata[352+:8],
                                    c_s_axis_tdata[360+:8],
                                    c_s_axis_tdata[368+:8],
                                    c_s_axis_tdata[376+:8],
                                    c_s_axis_tdata[384+:8],
                                    c_s_axis_tdata[392+:8],
                                    c_s_axis_tdata[400+:8],
                                    c_s_axis_tdata[408+:8],
                                    c_s_axis_tdata[416+:8],
                                    c_s_axis_tdata[424+:8],
                                    c_s_axis_tdata[432+:8],
                                    c_s_axis_tdata[440+:8],
                                    c_s_axis_tdata[448+:8],
                                    c_s_axis_tdata[456+:8],
                                    c_s_axis_tdata[464+:8],
                                    c_s_axis_tdata[472+:8],
                                    c_s_axis_tdata[480+:8],
                                    c_s_axis_tdata[488+:8],
                                    c_s_axis_tdata[496+:8],
                                    c_s_axis_tdata[504+:8]
                                };

always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        c_wr_en <= 1'b0;
        c_index <= 4'b0;

        c_m_axis_tdata <= 0;
        c_m_axis_tuser <= 0;
        c_m_axis_tkeep <= 0;
        c_m_axis_tvalid <= 0;
        c_m_axis_tlast <= 0;

        c_state <= IDLE_C;
    end
    else begin
        case(c_state)
            IDLE_C: begin
                if(c_s_axis_tvalid && mod_id[7:3] == STAGE_ID && mod_id[2:0] == ACTION_ID && control_flag == 16'hf2f1)begin
                    c_wr_en <= 1'b1;
                    c_index <= c_s_axis_tdata[384+:8];

                    c_m_axis_tdata <= 0;
                    c_m_axis_tuser <= 0;
                    c_m_axis_tkeep <= 0;
                    c_m_axis_tvalid <= 0;
                    c_m_axis_tlast <= 0;

                    c_state <= WRITE_C;

                end
                else begin
                    c_wr_en <= 1'b0;
                    c_index <= 4'b0; 

                    c_m_axis_tdata <= c_s_axis_tdata;
                    c_m_axis_tuser <= c_s_axis_tuser;
                    c_m_axis_tkeep <= c_s_axis_tkeep;
                    c_m_axis_tvalid <= c_s_axis_tvalid;
                    c_m_axis_tlast <= c_s_axis_tlast;

                    c_state <= IDLE_C;
                end
            end
            //support full table flush
            WRITE_C: begin
                if(c_s_axis_tlast && c_s_axis_tvalid) begin
                    c_wr_en <= 1'b0;
                    c_index <= 4'b0;
                    c_state <= IDLE_C;
                end
                else begin
                    if (c_s_axis_tvalid) begin
                        c_wr_en <= 1'b1;
                        c_index <= c_index + 4'b1;
                    end
                    c_state <= WRITE_C;
                end
            end
        endcase

    end
end


//checkme: page table
page_tbl_16w_16d
page_tbl_16w_16d
(
    //write
    .addra(c_index),
    .clka(clk),
    .dina(c_s_axis_tdata_swapped[C_S_AXIS_DATA_WIDTH-1 -: 16]),
    .ena(1'b1),
    .wea(c_wr_en),

    //match
    .addrb(vlan_id[7:4]),
    .clkb(clk),
    .doutb({addr_len,base_addr}),
    .enb(1'b1)
);

//ram for key-value
//2 cycles to get value
blk_mem_gen_0
data_ram_32w_32d
(
    //store-related
    .addra(store_addr),
    .clka(clk),
    .dina(store_din),
    .ena(1'b1),
    .wea(store_en),

    //load-related
    .addrb(load_addr),
    .clkb(clk),
    .doutb(load_data),
    .enb(1'b1)
);


endmodule