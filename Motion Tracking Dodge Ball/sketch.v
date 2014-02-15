// Etch-and-sketch

module sketch
	(
		CLOCK_50,						//	On Board 50 MHz
		startgame,							//	Push Button[3:0]
		x,
		y,
		color,
		writeEn,
		centerX,
		centerY//	DPDT Switch[17:0]
	);

	input			CLOCK_50;				//	50 MHz
	input startgame;
	input [8:0]centerX;
	input [7:0]centerY;
	output [8:0]x;
	output [7:0]y;
	output [15:0]color;
	output writeEn;
	
	
	wire resetnn = 1; //Useless, for vga reset
	
	wire resetn;	//Useless for now.
	assign resetn = 1;

	wire enable;    //Used to link clearscreen to downcount -- after screen is cleared, pulse is given to confirm screen cleared and proceed to print next frame
	
	wire gameover;		//Reset Not enabled atm
	wire [179:0]objCoordcomb;	//Register containing info for all obj xy
	wire imagedone;			//Not used yet, gives pulse after all obj xy are updated
	
	GameObjManip gom(CLOCK_50, startgame, gameover, objCoordcomb, imagedone);	//gameobj module
	
	// Create the color, x, y and writeEn wires that are inputs to the controller.

	
	//x
	wire [8:0] x;
	wire [8:0] xReset;
	wire [8:0] xNonReset[10:0];			//x is for output to vga, xreset is assigned to x when clearing, xnonreset is assigned to x when printing square		
	//
	
	//y
	wire [7:0] y;
	wire [7:0] yReset;
	wire [7:0] yNonReset[10:0];			//y same as x
	//
	
	
	wire [10:0]enablenext;
	wire [10:0]downCountPlot, arithPlot;
	wire doreset; //different enable triggers
	
	wire [3:0] xcount[10:0];
	wire [3:0] ycount[10:0];					//counters for drawing square
	assign writeEn = (arithPlot[0]&downCountPlot[0]) | ~doreset |(arithPlot[1]&downCountPlot[1])|(arithPlot[2]&downCountPlot[2])|(arithPlot[3]&downCountPlot[3])|(arithPlot[4]&downCountPlot[4])|(arithPlot[5]&downCountPlot[5])|(arithPlot[6]&downCountPlot[6])|(arithPlot[7]&downCountPlot[7])|(arithPlot[8]&downCountPlot[8])|(arithPlot[9]&downCountPlot[9])|(arithPlot[10]&downCountPlot[10]);
	reg [15:0]colorReg;								//Entire block is for implementation of animation

	
	

	integer i;
	

	wire pulse;										//Pulse at 0.1s
	reg tempimage;
	
	wire [4:0]gameoverw;
	assign gameover = (gameoverw==4'b0000) ? 0 : 1;
	
	objspeed Hz601(CLOCK_50, resetn, pulse);//Pulse generator
	ColDETCT C1(CLOCK_50, imagedone, centerX, centerY, objCoordcomb, gameoverw);




	
	downcounter25  d1(CLOCK_50, doreset, enable, downCountPlot[0], xcount[0], ycount[0], enablenext[0], objCoordcomb[17]);
	xyarith 			x1(xcount[0], ycount[0], objCoordcomb[8:0], objCoordcomb[16:9], xNonReset[0], yNonReset[0], arithPlot[0]);

	
	downcounter25  d2(CLOCK_50, 1, enablenext[0], downCountPlot[1], xcount[1], ycount[1], enablenext[1], objCoordcomb[35]);
	xyarith 			x2(xcount[1], ycount[1], objCoordcomb[26:18], objCoordcomb[34:27], xNonReset[1], yNonReset[1], arithPlot[1]);
	
	downcounter25  d3(CLOCK_50, 1, enablenext[1], downCountPlot[2], xcount[2], ycount[2], enablenext[2], objCoordcomb[53]);
	xyarith 			x3(xcount[2], ycount[2], objCoordcomb[44:36], objCoordcomb[52:45], xNonReset[2], yNonReset[2], arithPlot[2]);
	
	downcounter25  d4(CLOCK_50, 1, enablenext[2], downCountPlot[3], xcount[3], ycount[3], enablenext[3], objCoordcomb[71]);
	xyarith 			x4(xcount[3], ycount[3], objCoordcomb[62:54], objCoordcomb[70:63], xNonReset[3], yNonReset[3], arithPlot[3]);
	
	downcounter25  d5(CLOCK_50, 1, enablenext[3], downCountPlot[4], xcount[4], ycount[4], enablenext[4], objCoordcomb[89]);
	xyarith 			x5(xcount[4], ycount[4], objCoordcomb[80:72], objCoordcomb[88:81], xNonReset[4], yNonReset[4], arithPlot[4]);
	
	downcounter25  d6(CLOCK_50, 1, enablenext[4], downCountPlot[5], xcount[5], ycount[5], enablenext[5], objCoordcomb[107]);
	xyarith 			x6(xcount[5], ycount[5], objCoordcomb[98:90], objCoordcomb[106:99], xNonReset[5], yNonReset[5], arithPlot[5]);
	
	downcounter25  d7(CLOCK_50, 1, enablenext[5], downCountPlot[6], xcount[6], ycount[6], enablenext[6], objCoordcomb[125]);
	xyarith 			x7(xcount[6], ycount[6], objCoordcomb[116:108], objCoordcomb[124:117], xNonReset[6], yNonReset[6], arithPlot[6]);
	
	downcounter25  d8(CLOCK_50, 1, enablenext[6], downCountPlot[7], xcount[7], ycount[7], enablenext[7], objCoordcomb[143]);
	xyarith 			x8(xcount[7], ycount[7], objCoordcomb[134:126], objCoordcomb[142:135], xNonReset[7], yNonReset[7], arithPlot[7]);
	
	downcounter25  d9(CLOCK_50, 1, enablenext[7], downCountPlot[8], xcount[8], ycount[8], enablenext[8], objCoordcomb[161]);
	xyarith 			x9(xcount[8], ycount[8], objCoordcomb[152:144], objCoordcomb[160:153], xNonReset[8], yNonReset[8], arithPlot[8]);
	
	downcounter25  d10(CLOCK_50, 1, enablenext[8], downCountPlot[9], xcount[9], ycount[9], enablenext[9], objCoordcomb[179]);
	xyarith 			x10(xcount[9], ycount[9], objCoordcomb[170:162], objCoordcomb[178:171], xNonReset[9], yNonReset[9], arithPlot[9]);
	
	downcounter25  d11(CLOCK_50, ~downCountPlot[9], enablenext[9], downCountPlot[10], xcount[10], ycount[10], enablenext[10], 1);
	xyarith 			x11(xcount[10], ycount[10], centerX, centerY, xNonReset[10], yNonReset[10], arithPlot[10]);
	
	
	clearscreen		c1(CLOCK_50, imagedone, xReset, yReset, doreset, enable);	//3 modules for drawing square/clear screen
	
	always@(posedge CLOCK_50)
	begin
		if(!doreset)
			colorReg <= 16'h000F;
		else
			begin
			if(downCountPlot[0])
			colorReg <= 16'h00FF;
			else if(downCountPlot[1])
			colorReg <= 16'h0F0F;
			else if(downCountPlot[2])
			colorReg <= 16'hF00F;
			else if(downCountPlot[3])
			colorReg <= 16'h0FFF;
			else if(downCountPlot[4])
			colorReg <= 16'hF0FF;
			else if(downCountPlot[5])
			colorReg <= 16'hFF0F;
			else if(downCountPlot[6])
			colorReg <= 16'h735F;
			else if(downCountPlot[7])
			colorReg <= 16'h892F;
			else if(downCountPlot[8])
			colorReg <= 16'h937F;
			else if(downCountPlot[9])
			colorReg <= 16'h283F;
			else if(downCountPlot[10])
			colorReg <= 16'hFFFF;
			else
			colorReg <= 16'h000F;
			end
	end	//Select between clearing color (black) or square color
	
	assign color = colorReg;
	assign x = {9{(~doreset)}}&xReset | ((xNonReset[10]&{9{downCountPlot[10]}})|{9{(doreset)}}&((xNonReset[0]&{9{downCountPlot[0]}})|(xNonReset[1]&{9{downCountPlot[1]}})|(xNonReset[2]&{9{downCountPlot[2]}})|(xNonReset[3]&{9{downCountPlot[3]}})|(xNonReset[4]&{9{downCountPlot[4]}})|(xNonReset[5]&{9{downCountPlot[5]}})|(xNonReset[6]&{9{downCountPlot[6]}})|(xNonReset[7]&{9{downCountPlot[7]}})|(xNonReset[8]&{9{downCountPlot[8]}})|(xNonReset[9]&{9{downCountPlot[9]}})));
	assign y = {8{(~doreset)}}&yReset | ((yNonReset[10]&{8{downCountPlot[10]}})|{8{(doreset)}}&((yNonReset[0]&{8{downCountPlot[0]}})|(yNonReset[1]&{8{downCountPlot[1]}})|(yNonReset[2]&{8{downCountPlot[2]}})|(yNonReset[3]&{8{downCountPlot[3]}})|(yNonReset[4]&{8{downCountPlot[4]}})|(yNonReset[5]&{8{downCountPlot[5]}})|(yNonReset[6]&{8{downCountPlot[6]}})|(yNonReset[7]&{8{downCountPlot[7]}})|(yNonReset[8]&{8{downCountPlot[8]}})|(yNonReset[9]&{8{downCountPlot[9]}})));
	
	
	
	
endmodule


///////////////////////////////////////////////////////////////////////////////////////////////
//Downcounter				Starts when enabled, plot = 1, xcount and ycount increment towards 5x5 then ends


module downcounter25(clk, resetn, enable, plot, xcount, ycount, enablenext, nodraw);
input clk, resetn, enable, nodraw;
output plot, enablenext;
output [3:0]xcount, ycount;

reg [1:0]cstate, nstate;
reg write, write2, enablenext;
reg [3:0]xreg, yreg;

reg pulseextender = 0;

parameter[1:0] ST_IDLE = 0, ST_DOCOUNT = 1, ST_COUNTING = 2;

always@(*)
begin
	case(cstate)
		ST_IDLE:
			begin
				write = 0;
				if(enable)
					nstate = ST_DOCOUNT;
				else
				begin
					nstate = ST_IDLE;
				end
			end
		ST_DOCOUNT:
			begin
				write = 1;
				nstate = ST_COUNTING;
			end
		ST_COUNTING:
			begin
			if(xreg==15)
				begin
					if(yreg==15)
					begin
					nstate = ST_IDLE;
					end
					else
					begin
						nstate = ST_COUNTING;
					end
				end
			else
				begin
					nstate = ST_COUNTING;
				end
			end
		default:
			begin
				nstate = ST_IDLE;
				write = 0;
			end
		endcase
end

always@(posedge clk)
begin
	if(!resetn)
	begin
		cstate <= ST_IDLE;
	end
	else
		cstate <= nstate;
end

always@(posedge clk)
begin
	if(cstate == ST_IDLE)
	begin
		if(pulseextender)
		begin
			pulseextender <= 0;
			write2 <= 1;
		end
		else
		begin
			enablenext <= 0;
			write2 <= 1;
		end
	end
	else if(cstate == ST_DOCOUNT)
	begin
		xreg <= 0;
		yreg <= 0;
	end
	else if(cstate == ST_COUNTING)
	begin
		if(xreg == 15)
		begin
			yreg <= yreg + 1;
			xreg <= 0;
			if(yreg==15)
			begin
			write2<=0;
			enablenext <= 1;
			pulseextender <= 1;
			end
		end
		else
		begin
			xreg <= xreg +1;
		end
	end
end
				

assign xcount = xreg;
assign ycount = yreg;
assign plot = write&write2&nodraw;

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////
//xyarith				Adds current xcoord ycoord to the xycounters, draws square


module xyarith(xadd, yadd, xin, yin, xout, yout, write);
input [3:0]xadd, yadd;
input [8:0]xin;
input [7:0]yin;
output [8:0]xout;
output [7:0]yout;
output write;

reg [8:0]xreg;
reg [7:0]yreg;
reg writetemp = 1;

always@(*)
begin
	xreg = xin + xadd;
	yreg = yin + yadd;
end

assign xout = xreg;
assign yout = yreg;
assign write = writetemp;

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////
//clearscreen				Clears the screen. Sends out enableother pulse at the end to indicate finished



module clearscreen(clk, resetn, xout, yout, doreset, enableother);
input resetn, clk;
output [8:0]xout;
output [7:0]yout;
output doreset;
output enableother;
reg eother;

reg [1:0]cstate, nstate;
reg doresetreg, write2;
reg [8:0]xreg;
reg [7:0]yreg;

reg pulseextender = 0;

parameter[1:0] ST_IDLE = 0, ST_DOCOUNT = 1, ST_COUNTING = 2;

always@(*)
begin
	case(cstate)
		ST_IDLE:
			begin
				doresetreg = 1;
				if(!resetn)
					nstate = ST_DOCOUNT;
				else
				begin
					nstate = ST_IDLE;
				end
			end
		ST_DOCOUNT:
			begin
				doresetreg = 0;
				nstate = ST_COUNTING;
			end
		ST_COUNTING:
			begin
			if(xreg==9'b100111111)
				begin
					if(yreg==8'b11101111)
					begin
					nstate = ST_IDLE;
					end
					else
					begin
						nstate = ST_COUNTING;
					end
				end
			else
				begin
					nstate = ST_COUNTING;
				end
			end
		default:
			begin
				nstate = ST_IDLE;
				doresetreg = 1;
			end
		endcase
end

always@(posedge clk)
begin
	cstate <= nstate;
end

always@(posedge clk)
begin
	if(cstate == ST_IDLE)
	begin
	if(pulseextender)
		pulseextender <= 0;
	else
		eother <= 0;
	end
	else if(cstate == ST_DOCOUNT)
	begin
		eother <= 0;
		xreg <= 0;
		yreg <= 0;
		write2<=0;
	end
	else if(cstate == ST_COUNTING)
	begin
		if(xreg == 9'b100111111)
		begin
			yreg <= yreg + 1;
			xreg <= 0;
		end
		else
		begin
			xreg <= xreg +1;
		end
		if(xreg == 9'b100111111)
		begin
			if(yreg == 8'b11101111)
			begin
			write2<=1;
			eother <= 1;
			pulseextender <= 1;
			end
		end
	end
end
				

assign xout = xreg;
assign yout = yreg;
assign doreset = doresetreg&write2;
assign enableother = eother;

endmodule

///////////////////////////////////////////////////OBJSPEED
// Pulse to increase x position. Speed is 1 pixel/pulsetime
module Hz60(clock, reset, pulse);
input clock, reset;
output pulse;
reg [25:0]count = {26{1'b0}};
assign pulse = (count == 1'b1);

always@(posedge clock, negedge reset)
begin
if(!reset)
	count<={26{1'b0}};
else
	count <= pulse ? 0 : count+1'b1;
end

endmodule
/////////////////////////////////////////////////////////////

module ColDETCT(clk, imagedone, xobj, yobj, xyreg, gameoverw);
input imagedone;
input clk;
input [8:0]xobj;
input [7:0]yobj;
input [89:0]xyreg;
output [4:0]gameoverw;
integer i;

reg [4:0]gameoverw;

always@(posedge clk)
begin fe:

if(~(gameoverw==4'b0000))
	gameoverw<=0;
else
if(imagedone)
for(i=0;i<5;i=i+1)
begin
if(xyreg[i*18+17])
if(xyreg[i*18+:9]>=xobj)
	begin
	if(xyreg[i*18+:9]-xobj<16)
		begin
			if(xyreg[(i*18+9)+:8]>=yobj)
				if(xyreg[(i*18+9)+:8]-yobj<16)
					gameoverw[i]<=1;
				else
					gameoverw[i]<=0;
			else
				if(yobj-xyreg[(i*18+9)+:8]<16)
					gameoverw[i]<=1;
				else
					gameoverw[i]<=0;
		end
	else
		gameoverw[i]<=0;
	end
	
else
	begin
	if(xobj-xyreg[i*18+:9]<16)
		begin
			if(xyreg[(i*18+9)+:8]>=yobj)
				if(xyreg[(i*18+9)+:8]-yobj<16)
					gameoverw[i]<=1;
				else
					gameoverw[i]<=0;
			else
				if(yobj-xyreg[(i*18+9)+:8]<16)
					gameoverw[i]<=1;
				else
					gameoverw[i]<=0;
		end
	else
		gameoverw[i]<=0;
	end
	
end

end

endmodule


//////////////////////////////////////////////
/*module controlFSM(clk, enable, enableout);
input clk;
input enable;
output [10:0]enableout;

reg [10:0]enableout;
reg [9:0]counter = 0;

reg [1:0]cstate, nstate;

parameter[1:0] ST_IDLE = 0, ST_ENABLE = 1, ST_WAIT = 2;

always@(*)
begin
	case(cstate)
		ST_IDLE:
			begin
			if(enable)
			nstate = ST_WAIT;
			else
			nstate = ST_ENABLE;
			end
		ST_ENABLE:
			begin
			nstate=ST_WAIT;
			end
		ST_WAIT:
			begin
			if(done)
			nstate = ST_IDLE;
			else
			if(gonext)
			nstate=ST_ENABLE;
			else
			nstate=ST_WAIT;
			end
		default:
			begin
			nstate = ST_WAIT;
			end
	endcase
end

always@(posedge clk)
begin
	cstate <= nstate;
end

always@(posedge clk)
begin
	if(cstate == ST_IDLE)
		begin
		end
	else if(cstate == ST_ENABLE)
		begin
		end
	else if(cstate == ST_WAIT)
		begin
		end



	count <= pulse ? 0 : count+1'b1;
end*/