typedef enum logic [2:0] {STATE_RESET, STATE_WAIT, STATE_READ_MEMORY, STATE_CALCULATE_ALG, STATE_WRITE_MEMORY, STATE_DONE} statetype;


module testbench();

	logic clk;
	logic clk2;
	logic reset;
	logic val_i;
	logic rdy_i;
	logic h_m;
	logic [2:0] set_index;
	logic [1:0] cache_hit_index;
	

	logic val_o;
	logic rdy_o;
	logic [1:0] wayindex;
	logic [7:0] outputdata;
	
    
    // The device under test
    //lru_datapath dut(inputData, hit_miss_signal, way_hit_index, clk, outputdata, wayindex);
	LRU_unit dut(clk, clk2, reset, val_i, rdy_i, h_m, set_index, cache_hit_index, val_o, rdy_o, wayindex, outputdata);

	`include "testfixture.verilog"
    
endmodule


module LRU_unit #(parameter WIDTH=8)
         (input logic clk, clk2, reset,
          //input (request) side
          input logic val_i,
          input logic rdy_i,
          input logic h_m,
          input logic [2:0] set_index,
          input logic [1:0] cache_hit_index,

          //output (response) side
          output logic val_o,
          output logic rdy_o,
          output logic [1:0] wayindex,
          output logic [WIDTH-1:0] outputdata);

    //Command signals
    logic we; //write enable
    logic output_rdy; //output is done writing to the memory and is ready to output

    //Status signals
    logic read_done; //the data is read from the memory and is ready to go in to the alg
    logic output_data_rdy; //the data is ready to be outputted
    logic output_wayindex_rdy; //the way index is ready to be outputted
    logic write_done; //output data was written to memory


    //Control
    lru_controller_synth controller (.*);

    //Datapath
    lru_datapath datapath (.*);

endmodule

module lru_controller_synth  (input clk, clk2, reset,
               input val_i,
                output rdy_o,
            output val_o,
            input rdy_i,
                output logic we,
            output logic output_rdy,
            input logic read_done,
            input logic output_data_rdy,
            input logic output_wayindex_rdy,
            input logic write_done);

    statetype state;

    
    //statelogic controls the fms
    statelogic sl(clk, clk2, reset, val_i, rdy_o, val_o, rdy_i, read_done, output_data_rdy,            output_wayindex_rdy, write_done, state);
    //outputlogic controls the output values of each state
    outputlogic ol(state, rdy_o, val_o, we, output_rdy);

endmodule

module statelogic (input logic clk, clk2, reset,
           input logic val_i,
           input logic rdy_o,
           input logic val_o,
           input logic rdy_i,
               input logic read_done,
               input logic output_data_rdy,
           input logic output_wayindex_rdy,
               input logic write_done,
           output statetype state);


    statetype nextstate;
    //logic req_go;
    logic is_read_dn;
    logic is_alg_dn;
    logic is_write_dn;
    logic resp_go;
    logic [2:0] ns, state_logic;


    //mux to select the next state when the reset bit is low
    mux2 #(3) resetmux(nextstate, STATE_RESET, reset, ns);

    //flip flop to put the nextstate into the register
    statereg #(3) sreg(clk, clk2, ns, state_logic); 


    assign req_go = val_i && rdy_o; //val_i indicates that the LRU is ready to take in a value and rdy_o indicates that its ready to receive a new output
    assign is_read_dn = read_done; //the data is read from the memory and is ready to be fed to the lru algorithm
    assign is_alg_dn = output_data_rdy && output_wayindex_rdy; //output data and the wayindex is ready from the algorithm
    assign is_write_dn = write_done; //is it done writing new data to the memory
    assign resp_go = val_o && rdy_i; //sink is ready to read the output and the lru is ready to send the output
    assign state = statetype'(state_logic);

    always_comb
    begin
        case (state)
        STATE_RESET:        if(!reset)    nextstate = STATE_WAIT;
        STATE_WAIT:             if(req_go)      nextstate = STATE_READ_MEMORY;
                    else        nextstate = STATE_WAIT;
        STATE_READ_MEMORY:    if(is_read_dn)  nextstate = STATE_CALCULATE_ALG;
                    else        nextstate = STATE_READ_MEMORY;
        STATE_CALCULATE_ALG:    if(is_alg_dn)   nextstate = STATE_WRITE_MEMORY;
                    else        nextstate = STATE_CALCULATE_ALG;
        STATE_WRITE_MEMORY:    if(is_write_dn) nextstate = STATE_DONE;
                    else        nextstate = STATE_WRITE_MEMORY;
        STATE_DONE:             if(resp_go)     nextstate = STATE_WAIT;
                    else        nextstate = STATE_DONE;
        endcase
      end
    
endmodule

module outputlogic (input statetype state_logic,
            output logic rdy_o,
            output logic val_o,
            output logic we,
            output logic output_rdy);
    
    task cs (input logic cs_rdy_o,
         input logic cs_val_o,
         input logic cs_we,
         input logic cs_output_rdy);
        
    begin
        rdy_o         = cs_rdy_o;
        val_o         = cs_val_o;
        we               = cs_we;
        output_rdy    = cs_output_rdy;
    end    
    endtask

    //state input structure
    /*
           	rdy_o    val_o    we    output_recived    
    WAIT          1        0      0           0
    READ_MEMORY   0        0      0           0    
    CALC          0        0      0           1    
    WRITE_MEMORY  0        0      1           0
    DONE          0        1      0           0    
    */

    always @ (*)
    begin
        cs(0, 0, 0, 0); //default settings
        case (state_logic)
            STATE_WAIT:             cs(1, 0, 0, 0);
            STATE_READ_MEMORY:      cs(0, 0, 0, 0);
            STATE_CALCULATE_ALG:    cs(0, 0, 0, 1);
            STATE_WRITE_MEMORY:     cs(0, 0, 1, 0);
            STATE_DONE:             cs(0, 1, 0, 0);
        endcase
    end
        
endmodule


module statereg #(parameter WIDTH=4)
             (input logic clk,clk2,
              input  logic [WIDTH-1:0] d,
                  output logic [WIDTH-1:0] q);

      logic [WIDTH-1:0] mid;

      latch #(WIDTH) master(clk2, d, mid);
      latch #(WIDTH) slave(clk, mid, q);

endmodule

module mux2 #(parameter WIDTH = 4)
         (input logic [WIDTH-1:0] d0, d1,
          input logic s,
          output logic [WIDTH-1:0] y);

       assign y = s ? d1 : d0;
endmodule

module latch #(parameter WIDTH = 8)
              (input  logic             ph,
               input  logic [WIDTH-1:0] d,
               output logic [WIDTH-1:0] q);

        always_latch
          if (ph) q <= d;
endmodule


module lru_datapath (input logic clk2, reset,
             input logic [2:0] set_index,
             input logic [1:0] cache_hit_index,
             input logic we,
             input logic output_rdy,
             input logic h_m,
             output logic output_data_rdy,
	     output logic output_wayindex_rdy,
             output logic read_done,
             output logic write_done,
             output logic [1:0] wayindex,
             output logic [7:0] outputdata);


    logic [7:0] ram_output_counter_values; //store the counter values that are read from the ram.
    logic [7:0] counter_values;  //counter value read from register after the ram stores it in the register.
    logic [7:0] updated_counters; //counter values that were updated through the LRU arithmetic unit.
    logic [7:0] updated_counters_post_register; //updated counter values that were saved in the register
    logic [1:0] updated_wayindex; //way index that was outputted by the LRU arithmetic unit
    logic [1:0] updated_wayindex_post_register; //way index value that were saved in the register to be outputted


    //read or write into memory
    single_port_ram spr(clk2, we, reset, updated_counters_post_register, set_index, read_done, write_done, ram_output_counter_values);
    
    //call arithmetic unit with counter_values gotten from the register
    lru_arithmetic_unit lru_arith(counter_values, cache_hit_index, h_m, updated_counters, updated_wayindex);
    
    //output latches
    latch #(8) output_reg_counter(~we, updated_counters, updated_counters_post_register);
    latch #(2) index(output_rdy, updated_wayindex, updated_wayindex_post_register);

    //input latches
        latch #(8) input_reg_counter(read_done, ram_output_counter_values, counter_values);

    
    assign output_data_rdy = output_rdy ? 1'b1 : 1'b0;
    assign outputdata = ram_output_counter_values;
    assign output_wayindex_rdy = output_rdy ? 1'b1 : 1'b0;
    assign wayindex = updated_wayindex_post_register;
    
endmodule

module single_port_ram (input logic clk,  we,
            input logic reset,
            input logic [7:0] updated_counter_set,
                input logic [2:0] set_index,
            output logic read_done,
              output logic write_done,
               output logic [7:0] q);
    
           // Declare the RAM variable
           logic [7:0] ram[7:0];
      

           //initialize all rows in memory to 00011011 when the reset flag is up.
           integer i;
           always @(posedge clk)
           begin
            if (reset)
            begin
                for(i=0; i<8; i=i+1)
                begin
                    ram[i] <= 8'b00011011;
                end
            end
            else if (we)
            begin
                    ram[set_index] <= updated_counter_set; //set the new data into the ram given the row (set_index)
            end
            end

        //on the negative edge of the clock mark the read_done as high when write enable is low. This is to ensure that ram[set_index] produced the                 proper value in time
        always @ (negedge clk)
        begin
            read_done <= 1'b0;
            if (!we)
            begin
                read_done <= 1'b1;
            end
            else
            begin
                read_done <= 1'b0;
            end
        end


        assign q = (!we) ? ram[set_index] : 8'b00000000; //only output when write enable is low
        assign write_done = we ? 1'b1 : 1'b0; // if write enable is high then set the write_done bit high that will be sent to the controller
    
endmodule

module lru_arithmetic_unit (input logic [7:0] counter_values,
                input logic [1:0] cache_hit_index,
                input logic h_m,
                output logic [7:0] outputData,
                output logic [1:0] wayIndex);


        //split the data into its 4 different ways
            wire [1:0] way_3 = counter_values[7:6];
            wire [1:0] way_2 = counter_values[5:4];
            wire [1:0] way_1 = counter_values[3:2];
            wire [1:0] way_0 = counter_values[1:0];

            wire[7:0] is_equal_zero;
            wire[7:0] outputMuxData;
            wire[3:0] outputMuxIndex;

            wire[7:0] select;
        
            //call the compare_to_zero module to see if the counter value is 0 or not for all for ways
            compare_to_zero equal1(.counterValue (way_0), .is_equal_zero (is_equal_zero[1:0]));
            compare_to_zero equal2(.counterValue  (way_1), .is_equal_zero (is_equal_zero[3:2]));
            compare_to_zero equal3(.counterValue  (way_2), .is_equal_zero (is_equal_zero[5:4]));
            compare_to_zero equal4(.counterValue (way_3), .is_equal_zero (is_equal_zero[7:6]));
    
            wire [1:0] way_hit_counter;
            wire [7:0] greater_than_way_hit_counter_select;
           
        //if its a cache hit, get the counter value of that way that was hit
            get_way_hit_counter g1(counter_values, cache_hit_index, way_hit_counter);
        
        //compare the cache hit way counter value with all other counters to determine if its greater than the cache hit way counter value so they can             be decremented by -1.
            compare_counter_to_wayhit_counter compare1(way_0, way_hit_counter, greater_than_way_hit_counter_select[1:0]);
            compare_counter_to_wayhit_counter compare2(way_1, way_hit_counter, greater_than_way_hit_counter_select[3:2]);
            compare_counter_to_wayhit_counter compare3(way_2, way_hit_counter, greater_than_way_hit_counter_select[5:4]);
            compare_counter_to_wayhit_counter compare4(way_3, way_hit_counter, greater_than_way_hit_counter_select[7:6]);

	   //2v1 mux to get the appropriate select value which will determine if its value is updated to 3, -1 or stays the same.
	   mux2 #(2) getway0sel(is_equal_zero[1:0], greater_than_way_hit_counter_select[1:0], h_m, select[1:0]);
	   mux2 #(2) getway1sel(is_equal_zero[3:2], greater_than_way_hit_counter_select[3:2], h_m, select[3:2]);
	   mux2 #(2) getway2sel(is_equal_zero[5:4], greater_than_way_hit_counter_select[5:4], h_m, select[5:4]);
	   mux2 #(2) getway3sel(is_equal_zero[7:6], greater_than_way_hit_counter_select[7:6], h_m, select[7:6]);

	   //3x1 mux that will update the counter values to their appropriate value given the select from above.
	   mux3 h1(way_0, select[1:0], outputMuxData[1:0], outputMuxIndex[0]);
	   mux3 h2(way_1, select[3:2], outputMuxData[3:2], outputMuxIndex[1]);
	   mux3 h3(way_2, select[5:4], outputMuxData[5:4], outputMuxIndex[2]);
	   mux3 h4(way_3, select[7:6], outputMuxData[7:6], outputMuxIndex[3]);

           //4x2 encoder to output the way index that was updated to 3
            always @ (outputMuxIndex)
            begin
             case (outputMuxIndex)
             4'b0001: wayIndex = 2'b00;
             4'b0010: wayIndex = 2'b01;
             4'b0100: wayIndex = 2'b10;
             4'b1000: wayIndex = 2'b11;
             endcase
            end
        
           //output the new counter values
           assign outputData = outputMuxData;

endmodule

module get_way_hit_counter (input logic [7:0] inputData,
                input logic [1:0] way_hit_index,
                output logic [1:0] way_hit_counter);


    always @ (*)
    begin
        case(way_hit_index)
            2'b00 : way_hit_counter = inputData[1:0];
            2'b01 : way_hit_counter = inputData[3:2];
            2'b10 : way_hit_counter = inputData[5:4];
            2'b11 : way_hit_counter = inputData[7:6];
        endcase
    end
    

endmodule

module compare_counter_to_wayhit_counter(input logic [1:0] compare_inputData,
                     input logic [1:0] way_hit_counter,
                     output logic [1:0] greater_than_way_hit_counter );

	    wire [1:0] select;

	    assign select[0] = compare_inputData > way_hit_counter; //first bit represents if the counter value is greater than the counter value that the cache was hit
	    assign select[1] = compare_inputData == way_hit_counter; //second bit represents if the counter value is equal to that of the counter value that was the cache hit

	    //depending on the above scenario, the greate_than_way_hit_counter is assigned a bit that will act as the select for the mux3.
	    always @ (*)
	    begin
	    case(select)
	    2'b00: greater_than_way_hit_counter = 2'b10;
	    2'b01: greater_than_way_hit_counter = 2'b00;
	    2'b10: greater_than_way_hit_counter = 2'b11;
	    endcase
	    end

endmodule

module compare_to_zero (
    input logic [1:0] counterValue,
    output wire [1:0] is_equal_zero);
    
    //check the counterValue if its equal to 0. Two bits are needed due to this acting as the mux3 select.
    assign is_equal_zero[0] = ~| counterValue;
    assign is_equal_zero[1] = ~| counterValue;


endmodule

module mux2 #(parameter WIDTH = 4)
         (input logic [WIDTH-1:0] d0, d1,
          input logic s,
          output logic [WIDTH-1:0] y);

    assign y = s ? d1 : d0;
endmodule


module mux3 (input logic [1:0] countValue,
        input logic [1:0] select,
        output logic [1:0] outputMuxData,
        output logic outputMuxwayindex);

    

    always @ (*)
    begin
        case (select)
            2'b11:
            begin
                outputMuxData = 2'b11; //update the counter value to 3.
                outputMuxwayindex = 1'b1; //set the bits high for the way that was updated to 3
            end
            2'b00:
            begin
                 outputMuxData = countValue - 1'b1; //decrement the counter value by -1
                 outputMuxwayindex = 1'b0; //set the bits low for the way that was decremented by -1
            end
            2'b10:
            begin
                outputMuxData = countValue; //the counter value stays the same.
            end
        endcase
    end
endmodule
