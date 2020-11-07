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
		   FLUSH_CTL		= 4,
		   FLUSH_DATA_LAST	= 5,
		   FLUSH_CTL_LAST	= 6,
		   EMPTY_DATA_CYCLE		= 7,
		   EMPTY_CTL_CYCLE	= 8;


reg [C_S_AXIS_DATA_WIDTH-1:0]		r_tdata;
reg [((C_S_AXIS_DATA_WIDTH/8))-1:0]	r_tkeep;
reg [C_S_AXIS_TUSER_WIDTH-1:0]		r_tuser;
reg									r_tvalid;
reg									r_tlast;

reg [3:0] state, state_next;

// 1 for control, 0 for data;
reg 								c_switch;

// signals for pkt fifo
wire									pkt_fifo_nearly_full;
wire									pkt_fifo_empty;
reg										pkt_fifo_rd_en;
wire [C_S_AXIS_DATA_WIDTH-1:0]			tdata_fifo;
wire [((C_S_AXIS_DATA_WIDTH/8))-1:0]	tkeep_fifo;
wire [C_S_AXIS_TUSER_WIDTH-1:0]			tuser_fifo;
wire									tlast_fifo;
reg [C_S_AXIS_DATA_WIDTH-1:0]			tdata_fifo_d1;
reg [((C_S_AXIS_DATA_WIDTH/8))-1:0]		tkeep_fifo_d1;
reg [C_S_AXIS_TUSER_WIDTH-1:0]			tuser_fifo_d1;
reg										tlast_fifo_d1;

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
);

always @(*) begin

	r_tdata = tdata_fifo_d1;
	r_tkeep = tkeep_fifo_d1;
	r_tuser = tuser_fifo_d1;
	r_tlast = tlast_fifo_d1;
	r_tvalid = 0;

	c_switch = 0;

	state_next = state;
	pkt_fifo_rd_en = 0;

	case (state) 
		WAIT_FIRST_PKT: begin
			// 1st packet
			if (!pkt_fifo_empty) begin
				if ((tdata_fifo[143:128]==`ETH_TYPE_IPV4) && 
					(tdata_fifo[223:216]==`IPPROT_UDP)) begin
					state_next = WAIT_SECOND_PKT;
					pkt_fifo_rd_en = 1;
				end
				else begin
					state_next = DROP_PKT;
				end
			end
		end
		WAIT_SECOND_PKT: begin
			// 2nd packet
			if (!pkt_fifo_empty && m_axis_tready) begin
				if (tdata_fifo[79:64]==`CONTROL_PORT) begin
					c_switch = 1'b1;
					pkt_fifo_rd_en = 1;
					r_tvalid = 1;
					state_next = FLUSH_CTL;
				end
				else begin
					pkt_fifo_rd_en = 1;
					r_tvalid = 1;
					state_next = FLUSH_DATA;
				end
			end
		end
		FLUSH_DATA: begin
			if (m_axis_tready) begin
				r_tvalid = 1;

				if (!pkt_fifo_empty) begin
					pkt_fifo_rd_en = 1;
				end
				else begin
					state_next = EMPTY_DATA_CYCLE;
				end

				if (tlast_fifo) begin
					state_next = FLUSH_DATA_LAST;
				end
			end
		end
		FLUSH_DATA_LAST: begin
			if (m_axis_tready) begin
				r_tvalid = 1;
				state_next = WAIT_FIRST_PKT;
			end
		end
		EMPTY_DATA_CYCLE: begin
			if (!pkt_fifo_empty) begin
				pkt_fifo_rd_en = 1;
				state_next = FLUSH_DATA;
			end
		end
		FLUSH_CTL: begin
			if (m_axis_tready) begin
				r_tvalid = 1;
				c_switch = 1;

				if (!pkt_fifo_empty) begin
					pkt_fifo_rd_en = 1;
				end
				else begin
					state_next = EMPTY_CTL_CYCLE;
				end

				if (tlast_fifo) begin
					state_next = FLUSH_CTL_LAST;
				end
			end
		end
		FLUSH_CTL_LAST: begin
			if (m_axis_tready) begin
				r_tvalid = 1;
				c_switch = 1;
				state_next = WAIT_FIRST_PKT;
			end
		end
		EMPTY_CTL_CYCLE: begin
			if (!pkt_fifo_empty) begin
				pkt_fifo_rd_en = 1;
				state_next = FLUSH_CTL;
			end
		end
		DROP_PKT: begin
			pkt_fifo_rd_en = 1;
			if (tlast_fifo) begin
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
			m_axis_tdata <= 0;
			m_axis_tkeep <= 0;
			m_axis_tuser <= 0;
			m_axis_tlast <= 0;
			m_axis_tvalid <= 0;
			// 
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
		tdata_fifo_d1 <= 0;
		tuser_fifo_d1 <= 0;
		tkeep_fifo_d1 <= 0;
		tlast_fifo_d1 <= 0;
	end
	else begin
		tdata_fifo_d1 <= tdata_fifo;
		tuser_fifo_d1 <= tuser_fifo;
		tkeep_fifo_d1 <= tkeep_fifo;
		tlast_fifo_d1 <= tlast_fifo;
	end
end

ila_1
debug1 (
	.clk		(clk),

	.probe0		(state),
	.probe1		(ctrl_m_axis_tvalid),
	.probe2		(c_switch),
	.probe3		(s_axis_tvalid),
	.probe4		(s_axis_tlast),
	.probe5		(m_axis_tvalid),
	.probe6		(m_axis_tlast),
	.probe7		(m_axis_tdata[255-:16])
);

endmodule
