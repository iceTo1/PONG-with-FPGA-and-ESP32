
// ======================================================
// PONG UART + VGA
// UART: GPIO17 -> IO1(PIN_AB6) -> uart_rx
// ======================================================
module FPGA (
	input  wire       MAX10_CLK1_50, // 50 MHz
   input  wire       uart_rx,       // IO1 (PIN_AB6)
   output wire [3:0] VGA_R,         // VGA output red channel
   output wire [3:0] VGA_G,         // VGA output green channel
   output wire [3:0] VGA_B,         // VGA output blue channel
   output wire       VGA_HS,        // Horizontal sync signal
   output wire       VGA_VS,        // Vertical sync signal
   output wire [9:0] LEDR           // Debugging LED
);

   // =============================
   // 50MHz -> 25MHz
   // =============================
   reg pixclk = 1'b0; // New clk for VGA output
    
	always @(posedge MAX10_CLK1_50)
	begin
       pixclk <= ~pixclk;
   end
	
   // =============================
   // VGA Timing
   // =============================
   wire [9:0] hcount; // Horizontal position
   wire [9:0] vcount; // Vertical position
   wire       hsync, vsync, visible;
	
	// submodule for vga timing; u_sync
	// .portName(signalToConnect); like C++ class initializing
   vga_sync_640x480_60 u_sync (
        .clk    (pixclk), // connect new clk to clk
        .hcount (hcount), // hcount above to hcount
        .vcount (vcount), // vcount above to vcount
        .hsync  (hsync),  // hsync above to hsync
        .vsync  (vsync),  // vsync above to vsync
        .visible(visible) // visible above to visible
   );
	
	// Assign HS, VS
   assign VGA_HS = hsync;
   assign VGA_VS = vsync;

   // ==================================
   // UART Receive from ESP (115200bps)
   // ==================================
   wire       rx_dv;   // "Data Valid" flag, 1 when new byte is completed
   wire [7:0] rx_byte; // received 8bit data
	
	// submodule for UART
   uart_rx_simple #(
       .CLKS_PER_BIT(434)          // 50MHz / 115200 = 434
   ) u_rx (
       .i_Clock    (MAX10_CLK1_50),
       .i_Rx_Serial(uart_rx),
       .o_Rx_DV    (rx_dv),
       .o_Rx_Byte  (rx_byte)
   );

   // ===================================
   // Packet Parser
   //  0xA5, p1_hi, p1_lo, p2_hi, p2_lo,
   //  bx_hi, bx_lo, by_hi, by_lo, 0x5A
   // ===================================
   localparam S_IDLE  = 2'd0; // State: waiting for header (0xA5)
   localparam S_DATA  = 2'd1; // State: receiving data (p1, p2, ball)
   localparam S_END   = 2'd2; // State: committing after end (0x5A)

   reg [1:0]  pkt_state   = S_IDLE; // Set current to idle
   reg [3:0]  byte_idx    = 4'd0;   // Count data received (to 8; 0~7)

	// Temp resistor for UART data
   reg [15:0] p1_y_next   = 16'd0;
   reg [15:0] p2_y_next   = 16'd0;
   reg [15:0] ball_x_next = 16'd0;
   reg [15:0] ball_y_next = 16'd0;
	reg [15:0] score = 16'd0;
  // reg [15:0] p2_score = 8'd0;

   // Actual values for rendering (0~639, 0~479); use 10 bits
	// Initially centered
   reg [9:0]  p1_y_reg    = 10'd200;
   reg [9:0]  p2_y_reg    = 10'd200;
   reg [9:0]  ball_x_reg  = 10'd320;
   reg [9:0]  ball_y_reg  = 10'd240;
	//reg [9:0] 	p1_score = 10'd0;
  // reg [9:0] 	p2_score = 10'd0;

   // Debug: store last byte received, send to LED output;
   // (Testing period of UART Communication)
   reg [7:0] last_rx_byte = 8'd0;
	
	//==============================================================
	// Clamp Function: Adjust the values to stay within visible area
	//==============================================================
	//
	// Y-clamping Function
   function [9:0] clamp_y;
      input [15:0] val; // Input Value; 16 bit for possible overflow
      reg   [15:0] tmp; // "reg" enables to assign value with '=', 16 bit to match with input
   begin
		tmp = val; // Preserving input value
      
		// Chech if sign bit is 1
		if (tmp[15])
		begin
			// Then make it 0
			tmp = 16'd0;
		end
      
		// Check overflow
		if (tmp > 16'd479)
		begin
			// Reassign to maximum of visible
			tmp = 16'd479;
		end
		
		// Output the adjusted value, 10bit
      clamp_y = tmp[9:0];
	end
   endfunction
	
	// X-clamping Function; same operation
   function [9:0] clamp_x;
      input [15:0] val;
      reg   [15:0] tmp;
   begin
      tmp = val;
      if (tmp[15])
		begin
			tmp = 16'd0;
		end
		// Maximum x-value of visible is 639
      if (tmp > 16'd639)
		begin
			tmp = 16'd639;
		end
		
      clamp_x = tmp[9:0];

   end
   endfunction
	
	//===========================================================
   // Packet Parser; convert data from UART to coordinate values
	//===========================================================
   always @(posedge MAX10_CLK1_50) // Use 50MHz for logic
	begin
		// valid bit only
      if (rx_dv)
		begin
			// for debug, update last byte
         last_rx_byte <= rx_byte;
			
			// switch-case for state
         case (pkt_state)
				// Wait for start (0xA5)
            S_IDLE:
				begin
					// 0xA5
					if (rx_byte == 8'hA5)
					begin
						// after 0xA5, consider as data
						pkt_state <= S_DATA;
						// count again from 4th data (store index)
                  byte_idx  <= 4'd0; // for tracking the current location
               end
            end
				// Receiving data
            S_DATA:
				begin
					// from index
					case (byte_idx)
						// Interpret 8 bit received from UART
						// Combine the received 8 bit to 16 bit again
                  4'd0: p1_y_next[15:8]   <= rx_byte; // assign received data to upper 8 of p1_y
                  4'd1: p1_y_next[7:0]    <= rx_byte; // rest of p1_y
                  4'd2: p2_y_next[15:8]   <= rx_byte; // upper 8 of p2_y
                  4'd3: p2_y_next[7:0]    <= rx_byte; // rest of p2_y
                  4'd4: ball_x_next[15:8] <= rx_byte; // first 8 of ball_x
                  4'd5: ball_x_next[7:0]  <= rx_byte; // rest of ball_x
                  4'd6: ball_y_next[15:8] <= rx_byte; // first 8 of ball_x
                  4'd7: ball_y_next[7:0]  <= rx_byte; // rest of ball_x
						4'd8: score[15:8]       <= rx_byte; // first 8 of score
                  4'd9: score[7:0]  <= rx_byte; // rest of score
					default: ;
					endcase
					// If all received (expect)
					if (byte_idx == 4'd9)
					begin
						// update state to end
						pkt_state <= S_END;
					end
						// move ahead
						byte_idx <= byte_idx + 1'b1;
				end
				
				// After receiving one cycle
				S_END:
				begin
					// Check if the data is actually pointing to "end"	
					if (rx_byte == 8'h5A)
					begin
						// Assign the values adjusted via the clamp function
						p1_y_reg   <= clamp_y(p1_y_next);
						p2_y_reg   <= clamp_y(p2_y_next);
						ball_x_reg <= clamp_x(ball_x_next);
						ball_y_reg <= clamp_y(ball_y_next);
					end
					// Back to IDLE state
					pkt_state <= S_IDLE;
				end
				
				// If the data is incorrect, discard
				default: pkt_state <= S_IDLE;
			endcase
			
		end
		
	end

	// =============================
   // VGA Rendering
   // =============================
   localparam PADDLE_W   = 10; // Paddle width
   localparam PADDLE_H   = 60; // Paddle height
   localparam BALL_SIZE  = 8;  // Ball radius (square side)
	
	localparam tick_W = 3;
	localparam tick_H = 10;


   // Paddle location
   localparam P1_X = 20;               // Fixed x-coordinate
   localparam P2_X = 640-20-PADDLE_W;  // slight margin from the edge
	
	// Score location
	localparam tickp1_x1 = 20;
	localparam tickp1_x2 = tickp1_x1 + 8;
	localparam tickp1_x3 = tickp1_x2 + 8;
	localparam tickp1_x4 = tickp1_x3 + 8;
	localparam tickp2_x1 = 640-20-tick_W;
	localparam tickp2_x2 = tickp2_x1 - 8;
	localparam tickp2_x3 = tickp2_x2 - 8;
	localparam tickp2_x4 = tickp2_x3 - 8;
	
	localparam tick_y = 20;
	
	
	// Sort of Bool value; whether the coordinate is in visible location
   wire paddle1_on = (hcount >= P1_X) &&
                     (hcount <  P1_X + PADDLE_W) &&
                     (vcount >= p1_y_reg) &&
                     (vcount <  p1_y_reg + PADDLE_H);

   wire paddle2_on = (hcount >= P2_X) &&
                     (hcount <  P2_X + PADDLE_W) &&
                     (vcount >= p2_y_reg) &&
                     (vcount <  p2_y_reg + PADDLE_H);

   wire ball_on    = (hcount >= ball_x_reg) &&
                     (hcount <  ball_x_reg + BALL_SIZE) &&
                     (vcount >= ball_y_reg) &&
                     (vcount <  ball_y_reg + BALL_SIZE);
	// Marking the Score						
	wire score_on = (hcount >= tickp1_x1) &&
                    (hcount <  tickp1_x1 + tick_W) &&
                    (vcount >= tick_y) &&
                    (vcount <  tick_y + tick_H) && (score[7:0] < 8'd4) 
						  ||
						  (hcount >= tickp1_x2) &&
                    (hcount <  tickp1_x2 + tick_W) &&
                    (vcount >= tick_y) &&
                    (vcount <  tick_y + tick_H) && (score[7:0] < 8'd3) 
						  ||
						  (hcount >= tickp1_x3) &&
                    (hcount <  tickp1_x3 + tick_W) &&
                    (vcount >= tick_y) &&
                    (vcount <  tick_y + tick_H) && (score[7:0] < 8'd2) 
						  ||
						  (hcount >= tickp1_x4) &&
                    (hcount <  tickp1_x4 + tick_W) &&
                    (vcount >= tick_y) &&
                    (vcount <  tick_y + tick_H) && (score[7:0] < 8'd1) 
						  ||
						  (hcount >= tickp2_x1) &&
                    (hcount <  tickp2_x1 + tick_W) &&
                    (vcount >= tick_y) &&
                    (vcount <  tick_y + tick_H) && (score[15:8] < 8'd4)
						  ||
						  (hcount >= tickp2_x2) &&
                    (hcount <  tickp2_x2 + tick_W) &&
                    (vcount >= tick_y) &&
                    (vcount <  tick_y + tick_H) && (score[15:8] < 8'd3)
						  ||
						  (hcount >= tickp2_x3) &&
                    (hcount <  tickp2_x3 + tick_W) &&
                    (vcount >= tick_y) &&
                    (vcount <  tick_y + tick_H) && (score[15:8] < 8'd2)
						  ||
						  (hcount >= tickp2_x4) &&
                    (hcount <  tickp2_x4 + tick_W) &&
                    (vcount >= tick_y) &&
                    (vcount <  tick_y + tick_H) && (score[15:8] < 8'd1);
	
	
	// variable for rgb channel (4 bit each)
   reg [3:0] r, g, b;
	
	// Update during posedge of clk
   always @(posedge pixclk) // 25MHz
	begin
		// Out of visible area
      if (!visible)
		begin
			// black
			r <= 0;
         g <= 0;
         b <= 0;
      end
		// visible & location of prop
		else if (paddle1_on || paddle2_on || ball_on)
		begin
         // white (full RGB)
         r <= 4'hF;
         g <= 4'hF;
         b <= 4'hF;
      end
		else if (score_on)
		begin
			r <= (4'hF) / 2;
			g <= (4'hF) / 2;
			b <= (4'hF) / 2;
		end
		else
		begin
         // black for bg (visible & not prop)
         r <= 0;
         g <= 0;
         b <= 0;
      end
   end
	
	// assign values for VGA pins
   assign VGA_R = r;
   assign VGA_G = g;
   assign VGA_B = b;

   // Debug: assign last rx byte to LED out
   assign LEDR[7:0] = last_rx_byte;
   assign LEDR[9:8] = 2'b0;

endmodule


// ======================================================
// 640x480@60Hz VGA Timing
// ======================================================
module vga_sync_640x480_60(
   input  wire       clk,        // 25 MHz
   output reg  [9:0] hcount = 0, // 0~799
   output reg  [9:0] vcount = 0, // 0~524
   output wire       hsync,      // horizontal sync signal
   output wire       vsync,      // vertical sync signal
   output wire       visible     // flag for visible
);
	
	localparam H_VISIBLE = 640; // Horizontal visible pixels
   localparam H_FP      = 16;  // Front Porch
   localparam H_SYNC    = 96;  // Horizontal Sync signal (down during this period)
   localparam H_BP      = 48;  // Break before next cycle
   localparam H_TOTAL   = H_VISIBLE + H_FP + H_SYNC + H_BP; // 800

   localparam V_VISIBLE = 480; // Vertical visible pixels
   localparam V_FP      = 10;  // V Front Porch
   localparam V_SYNC    = 2;   // Vertical Sync siglna
   localparam V_BP      = 33;  // Break
   localparam V_TOTAL   = V_VISIBLE + V_FP + V_SYNC + V_BP; // 525
	
	// 
   always @(posedge clk)
	begin
		if (hcount == H_TOTAL-1) // If horizontal line cycle is complete,
		begin
			hcount <= 0; // assign 0 to reset count
         if (vcount == V_TOTAL-1) // If vetical line cycle is complete,
			begin
				vcount <= 0; // assign 0 to reset count
			end
			else
			begin
				// Default Operation: increase count (inside if to add after 1 H-cycle)
				vcount <= vcount + 1;
			end
	   end
	   else
	   begin
			// Default Operation: increase count
			hcount <= hcount + 1;
      end
	end
	
	// assign value; active low
   assign hsync   = ~((hcount >= (H_VISIBLE + H_FP)) &&
                      (hcount <  (H_VISIBLE + H_FP + H_SYNC)));
   assign vsync   = ~((vcount >= (V_VISIBLE + V_FP)) &&
                      (vcount <  (V_VISIBLE + V_FP + V_SYNC)));
	
	// assign visible flag if within visible range
   assign visible = (hcount < H_VISIBLE) && (vcount < V_VISIBLE);

endmodule


// ======================================================
// UART RX module
// ======================================================
module uart_rx_simple
#(
	parameter CLKS_PER_BIT = 434 // 50MHz, 115200bps
)
(
   input  wire       i_Clock,          // standard clk (50MHz)
   input  wire       i_Rx_Serial,      // UART input
   output reg        o_Rx_DV   = 1'b0, // valid flag
   output reg [7:0]  o_Rx_Byte = 8'd0  // received data (raw 8 bit)
);
	
   localparam s_IDLE         = 3'b000; // Detects start (0xA5)
   localparam s_RX_START_BIT = 3'b001; // Double check if UART is actually coming in
   localparam s_RX_DATA_BITS = 3'b010; // Save data
   localparam s_RX_STOP_BIT  = 3'b011; // Detects end (0x5A)
   localparam s_CLEANUP      = 3'b100; // Clean up

   reg [2:0] r_SM_Main      = s_IDLE; // Current state; default IDLE
   reg [8:0] r_Clock_Count  = 9'd0;   // Clock counter for current bit (max: 434)
   reg [2:0] r_Bit_Index    = 3'd0;   // Bit index of 8 (3bit)
   reg       r_Rx_Data      = 1'b1;   // Data
	
	// Stablize the external input by assigning to wire
   always @(posedge i_Clock) 
	begin
		r_Rx_Data <= i_Rx_Serial;
   end
	
	// Main logic to receive incoming UART
   always @(posedge i_Clock)
	begin
		case (r_SM_Main)
			// IDLE
			s_IDLE:
			begin
				o_Rx_DV       <= 1'b0; // Initialize with 0; no data
            r_Clock_Count <= 0;    // Re-initialize clk count
            r_Bit_Index   <= 0;    // Re-initialize bit idx
				
				// If input line goes to 0, proceed
            if (r_Rx_Data == 1'b0)
				begin
					// Proceed to Double-Checker (case)
					r_SM_Main <= s_RX_START_BIT;
            end
				else
				begin
					r_SM_Main <= s_IDLE;
            end
         end
			
			// Double Check
         s_RX_START_BIT:
			begin
				// At the middle point of UART cycle
				if (r_Clock_Count == (CLKS_PER_BIT-1)/2)
				begin
					// Check if still 0
					if (r_Rx_Data == 1'b0)
					begin
						// Re-initialize clk count
						r_Clock_Count <= 0;
						// Proceed to receive data
                  r_SM_Main     <= s_RX_DATA_BITS;
               end
					else
					begin
						// Back to IDLE if data was a noise (back to 1)
						r_SM_Main <= s_IDLE;
					end
				end
				else
				begin
					// Wait until reaching half point of UART cycle
					r_Clock_Count <= r_Clock_Count + 1;
               r_SM_Main     <= s_RX_START_BIT;
            end
         end
			
			// Save data
         s_RX_DATA_BITS:
			begin
				// When clk count hits 433 (one bit)
				if (r_Clock_Count == CLKS_PER_BIT-1)
				begin
					// Re-initialize counter
					r_Clock_Count <= 0;
					// Store the data at this moment, at bit index
               o_Rx_Byte[r_Bit_Index] <= r_Rx_Data;
					
					// When fully received the data
               if (r_Bit_Index == 3'd7)
					begin
						// Re-initialize bit idx
						r_Bit_Index <= 0;
						// Proceed to next step (case)
                  r_SM_Main   <= s_RX_STOP_BIT;
               end
					else
					begin
						// Increase bit idx
						r_Bit_Index <= r_Bit_Index + 1;
						// Continue saving data (case)
						r_SM_Main   <= s_RX_DATA_BITS;
					end
				end
				else
				begin
					// Standby until 433, increasing clk count
					r_Clock_Count <= r_Clock_Count + 1;
					// Continue (case)
               r_SM_Main     <= s_RX_DATA_BITS;
            end
         end
			
			// Wait for STOP
         s_RX_STOP_BIT:
			begin
				// If waited one full stop-bit time
				if (r_Clock_Count == CLKS_PER_BIT-1)
				begin
					// Mark valid, reset clk count
					o_Rx_DV       <= 1'b1;
					r_Clock_Count <= 0;
					// Proceed to cleanup (case)
					r_SM_Main     <= s_CLEANUP;
				end
				else
				begin
					// Standby, continue
					r_Clock_Count <= r_Clock_Count + 1;
					r_SM_Main     <= s_RX_STOP_BIT;
            end
         end

			// Cleanup
			s_CLEANUP:
			begin
				// Proceed to next cycle (case)
				r_SM_Main <= s_IDLE;
				// Reset the validity
				o_Rx_DV   <= 1'b0;
			end
			
			// Incorrect signal -> IDLE
			default: r_SM_Main <= s_IDLE;
		endcase
   end
	
endmodule
