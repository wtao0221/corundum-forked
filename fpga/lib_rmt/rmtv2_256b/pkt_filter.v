`timescale 1ns / 1ps

`define ETH_TYPE_IPV4	16'h0008
`define IPPROT_UDP		8'h11
`define CONTROL_PORT    16'hf2f1

module pkt_filter #(
	parameter C_S_AXIS_DATA_WIDTH = 256,
	parameter C_S_AXIS_TUSER_WIDTH = 128
)
(
	input				clk,
	input				aresetn,

	// input Slave AXI Stream
	input [C_S_AXIS_DATA_WIDTH-1:0]			s_axis_tdata,
	input [((C_S_AXIS_DATA_WIDTH/8))-1:0]	s_axis_tkeep,
	input [C_S_AXIS_TUSER_WIDTH-1:0]		s_axis_tuser,
	input									s_axis_tvalid,
	output									s_axis_tready,
	input									s_axis_tlast,

	// output Master AXI Stream
	output reg [C_S_AXIS_DATA_WIDTH-1:0]		m_axis_tdata,
	output reg [((C_S_AXIS_DATA_WIDTH/8))-1:0]	m_axis_tkeep,
	output reg [C_S_AXIS_TUSER_WIDTH-1:0]		m_axis_tuser,
	output reg									m_axis_tvalid,
	input										m_axis_tready,
	output reg									m_axis_tlast,

	//TODO a back-pressure is needed?
	output reg [C_S_AXIS_DATA_WIDTH-1:0]		ctrl_m_axis_tdata,
	output reg [((C_S_AXIS_DATA_WIDTH/8))-1:0]	ctrl_m_axis_tkeep,
	output reg [C_S_AXIS_TUSER_WIDTH-1:0]		ctrl_m_axis_tuser,
	output reg									ctrl_m_axis_tvalid,
	output reg									ctrl_m_axis_tlast

);

localparam WAIT_FIRST_PKT	= 0,
		   WAIT_SECOND_PKT	= 1,
		   DROP_PKT			= 2, 
		   FLUSH_DATA		= 3,
		   FLUSH_CTL		= 4;


reg [C_S_AXIS_DATA_WIDTH-1:0]		r_tdata;
reg [((C_S_AXIS_DATA_WIDTH/8))-1:0]	r_tkeep;
reg [C_S_AXIS_TUSER_WIDTH-1:0]		r_tuser;
reg									r_tvalid;
reg									r_tlast;

reg [2:0] state, state_next;

// 1 for control, 0 for data;
reg 								c_switch;

// 1-clk delayed signals
reg [C_S_AXIS_DATA_WIDTH-1:0]			s_axis_tdata_d1;
reg [((C_S_AXIS_DATA_WIDTH/8))-1:0]		s_axis_tkeep_d1;
reg [C_S_AXIS_TUSER_WIDTH-1:0]			s_axis_tuser_d1;
reg										s_axis_tlast_d1;
reg										s_axis_tvalid_d1;
reg										m_axis_tready_d1;

/*
// signals for pkt fifo
wire									pkt_fifo_nearly_full;
wire									pkt_fifo_empty;
reg										pkt_fifo_rd_en;
wire [C_S_AXIS_DATA_WIDTH-1:0]			tdata_fifo;
wire [((C_S_AXIS_DATA_WIDTH/8))-1:0]	tkeep_fifo;
wire [C_S_AXIS_TUSER_WIDTH-1:0]			tuser_fifo;
wire									tlast_fifo;

//
assign s_axis_tready = !pkt_fifo_nearly_full;

fallthrough_small_fifo #(
	.WIDTH(C_S_AXIS_DATA_WIDTH + C_S_AXIS_TUSER_WIDTH + C_S_AXIS_DATA_WIDTH/8 + 1),
	.MAX_DEPTH_BITS(8)
)
pkt_fifo
(
	.din									({s_axis_tdata, s_axis_tuser, s_axis_tkeep, s_axis_tlast}),
	.wr_en									(s_axis_tvalid & ~pkt_fifo_nearly_full),
	.rd_en									(pkt_fifo_rd_en),
	.dout									({tdata_fifo, tuser_fifo, tkeep_fifo, tlast_fifo}),
	.full									(),
	.prog_full								(),
	.nearly_full							(pkt_fifo_nearly_full),
	.empty									(pkt_fifo_empty),
	.reset									(~aresetn),
	.clk									(clk)
);*/

assign s_axis_tready = m_axis_tready_d1;

always @(*) begin

	r_tdata = s_axis_tdata_d1;
	r_tkeep = s_axis_tkeep_d1;
	r_tuser = s_axis_tuser_d1;
	r_tlast = s_axis_tlast_d1;
	r_tvalid = s_axis_tvalid_d1;

	c_switch = 1'b0;

	state_next = state;

	case (state) 
		WAIT_FIRST_PKT: begin
			// 1st pkt
			if (m_axis_tready && s_axis_tvalid) begin
				if ((s_axis_tdata[143:128]==`ETH_TYPE_IPV4) && 
					(s_axis_tdata[223:216]==`IPPROT_UDP)) begin
					state_next = WAIT_SECOND_PKT;
				end
				else begin
					state_next = DROP_PKT;
				end
			end
		end
		// send out data with 1-clk delayed signals
		WAIT_SECOND_PKT: begin
			if (s_axis_tvalid) begin
				if (s_axis_tdata[79:64] == `CONTROL_PORT) begin
					// ctrl pkt
					c_switch = 1'b1;
					state_next = FLUSH_CTL;
				end
				else begin
					state_next = FLUSH_DATA;
				end
			end
		end
		FLUSH_DATA: begin
			if (s_axis_tlast_d1) begin
				state_next = WAIT_FIRST_PKT;
			end
		end
		FLUSH_CTL: begin
			c_switch = 1'b1;
			if (s_axis_tlast_d1) begin
				state_next = WAIT_FIRST_PKT;
			end
		end
		DROP_PKT: begin
			r_tvalid = 0;
			if (s_axis_tlast_d1) begin
				state_next = WAIT_FIRST_PKT;
			end
		end
	endcase
end

always @(posedge clk or negedge aresetn) begin
	if (~aresetn) begin
		state <= WAIT_FIRST_PKT;

		m_axis_tdata <= 0;
		m_axis_tkeep <= 0;
		m_axis_tuser <= 0;
		m_axis_tlast <= 0;
		m_axis_tvalid <= 0;

		// control
		ctrl_m_axis_tdata <= 0;
		ctrl_m_axis_tkeep <= 0;
		ctrl_m_axis_tuser <= 0;
		ctrl_m_axis_tlast <= 0;
		ctrl_m_axis_tvalid <= 0;

	end
	else begin
		state <= state_next;

		if(!c_switch) begin
			m_axis_tdata <= r_tdata;
			m_axis_tkeep <= r_tkeep;
			m_axis_tuser <= r_tuser;
			m_axis_tlast <= r_tlast;

			m_axis_tvalid <= r_tvalid;
			// reset control path output 
			ctrl_m_axis_tdata <= 0;
			ctrl_m_axis_tkeep <= 0;
			ctrl_m_axis_tuser <= 0;
			ctrl_m_axis_tlast <= 0;
			ctrl_m_axis_tvalid <= 0;
		end
		else begin
			ctrl_m_axis_tdata <= r_tdata;
			ctrl_m_axis_tkeep <= r_tkeep;
			ctrl_m_axis_tuser <= r_tuser;
			ctrl_m_axis_tlast <= r_tlast;
			ctrl_m_axis_tvalid <= r_tvalid;
		end
		
	end
end

always @(posedge clk) begin
	if (~aresetn) begin
		s_axis_tdata_d1 <= 0;
		s_axis_tuser_d1 <= 0;
		s_axis_tkeep_d1 <= 0;
		s_axis_tlast_d1 <= 0;
		s_axis_tvalid_d1 <= 0;

		m_axis_tready_d1 <= 0;
	end
	else begin
		s_axis_tdata_d1 <= s_axis_tdata;
		s_axis_tuser_d1 <= s_axis_tuser;
		s_axis_tkeep_d1 <= s_axis_tkeep;
		s_axis_tlast_d1 <= s_axis_tlast;
		s_axis_tvalid_d1 <= s_axis_tvalid;

		m_axis_tready_d1 <= m_axis_tready;
	end
end

endmodule
