module GameObjManip(CLOCK_50, startgame, gameover, objCoordcomb, imagedone);

parameter[2:0] ST_IDLE = 0, ST_INITIALIZE = 1, ST_GAME = 2, ST_WAIT = 3, ST_GAME2 = 4;

input startgame;
input gameover;
input CLOCK_50;
output [179:0]objCoordcomb;
output imagedone;

assign objCoordcomb = {objCoord[9],objCoord[8],objCoord[7],objCoord[6],objCoord[5],objCoord[4],objCoord[3],objCoord[2],objCoord[1],objCoord[0]};
	
reg [2:0]cstate, nstate;
reg [3:0]level;
reg [17:0]objCoord[9:0];							//10 objects, 16:9 is y, 8:0 is x, 17 is enable//10 objects, 16:9 is y, 8:0 is x, 17 is enable
wire objdone;
integer i;
reg [5:0]counter;
reg [3:0]currpos;
reg resetSClock;
reg firstw;
reg donewait;

wire objclock;
reg imagedone, gogame, gogame2;

//y random gen

reg randReq;
wire [7:0]randNum;

reg[28:0]y_rand = 29'h55555555;
wire seed_low_bit, y_low_bit;
assign y_low_bit=y_rand[26] ^ y_rand[28];

always@(posedge CLOCK_50)
begin
y_rand<={y_rand[27:0], y_low_bit};
end

assign randNum = y_rand % 240;

////////////////////////////////////////////






assign objdone = objCoord[9][17]|objCoord[8][17]|objCoord[7][17]|objCoord[6][17]|objCoord[5][17]|objCoord[4][17]|objCoord[3][17]|objCoord[2][17]|objCoord[1][17]|objCoord[0][17]; //Active low

objspeed obj1(CLOCK_50, resetSClock, objclock);

always@(*)	
begin

	case(cstate)
			ST_IDLE:
					begin
						if(startgame)
							nstate = ST_INITIALIZE;
						else
							nstate = ST_IDLE;
					end
			ST_INITIALIZE:
					begin
						nstate = ST_GAME;
					end
			ST_GAME:
					begin
						if(!objdone)
							nstate=ST_WAIT;
						else
							if(gogame2)
							nstate=ST_GAME2;
							else
							nstate=ST_GAME;
					end
			ST_GAME2:
					begin
						if(gogame)
						nstate=ST_GAME;
						else
						nstate=ST_GAME2;
					end
			ST_WAIT:
					begin
						if(donewait)
							if(level==10)
							nstate=ST_IDLE;
							else
							nstate=ST_INITIALIZE;
						else
							nstate=ST_WAIT;
					end
			default:
					begin
					nstate = ST_IDLE;
					end
			endcase
			
end

always@(posedge CLOCK_50)
begin
	if(gameover)
		cstate <= ST_IDLE;
	else
		cstate <= nstate;
end

always@(posedge CLOCK_50)
begin

		if(cstate == ST_IDLE)
		begin
			level <= 0;
		end
		else if(cstate == ST_INITIALIZE)
					begin
						for(i=0; i < 10; i = i + 1)
						begin
							if(i==0)
							begin
								objCoord[i][17]<=1'b1;
								objCoord[i][16:9]<=randNum;
								objCoord[i][8:0]<=0;
								randReq <= 1;
							end
							else
							objCoord[i] <= 0;
							randReq <= 0;
						end
						currpos <= 0;
						counter <= 0;
						resetSClock <= 0;
						donewait <= 0;
						imagedone <=1;
					end
		else if(cstate ==	ST_GAME)
					begin
						resetSClock <= 1;
						gogame <= 0;
					if(objclock)
						begin
						imagedone <= 0;
						gogame2 <= 1;
						for(i=0;i<10;i=i+1) // Add to x positions
						begin
							if(objCoord[i][17])
							begin
								if(objCoord[i][8:0]<9'b101000000)
									objCoord[i]<=objCoord[i]+1;
								else
									objCoord[i][17]<=0;
							end
						end
							
							if(counter == 19)
							begin
								if(currpos<level)
								begin
									currpos <= currpos +1;
									objCoord[(currpos+1)][17]<=1;
									objCoord[(currpos+1)][16:9]<=randNum;
									randReq<=1;
								end
								else
								begin
								randReq<=0;
								end
							counter <= 0;
							end
							else
								counter <= counter + 1;
						end
						else
							imagedone <= 1;
						firstw <= 1;
					end
		else if(cstate == ST_GAME2)
			begin
				gogame2<=0;
				if(objclock)
				begin
					gogame<=1;
				end
				imagedone<=1;
			end
		else if(cstate == ST_WAIT)
					begin
						if(objclock)
						begin
							if(firstw)
							begin
								level <= level + 1;
								counter <= 0;
								firstw <= 0;
							end
							else
							begin
								counter <= counter + 1;
							end
						end
						if(counter == 10)
						donewait <= 1;
					end
			
end

endmodule

///////////////////////////////////////////////////OBJSPEED
// Pulse to increase x position. Speed is 1 pixel/pulsetime
module objspeed(clock, reset, pulse);
input clock, reset;
output pulse;
reg [25:0]count = {26{1'b0}};
assign pulse = (count == 19'b1100101101110011010);//10111110101111000010

always@(posedge clock, negedge reset)
begin
if(!reset)
	count<={26{1'b0}};
else
	count <= pulse ? 0 : count+1'b1;
end

endmodule
/////////////////////////////////////////////////////////////










//Useless, probably

/*input [3:0]numblocks;
input gstart;

reg [17:0]objCoord[9:0]; 															//10 objects, 16:9 is y, 8:0 is x, 17 is enable
wire clock_obj;
wire resetc = 1;
integer i

initial begin																			//Initialize regs to 0;
for(i=0; i < 9; i = i + 1)
	begin
		objCoord[i] = 0;
	end
end

objspeed objs1(CLOCK_50, resetc, clock_obj);


always@(posedge clock_obj, negedge resetreg)									//At every clockobj, update x coord
begin

	if(!resetreg)
	begin
		for(i=0; i < 9; i = i + 1)
		begin
			objCoord[i] = 0;
		end
	end
	
	else
	for(i=0; i < 9; i = i + 1)
	begin
		if(objCoord[i][17])
		objCoord[i] = objCoord[i] + 1;
	end
	
end


always@(posedge gstart)
begin
	*/