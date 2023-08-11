//`include "ifu_define.v"

`define IFU_ADDR_WIDTH 48
`define IFU_DATA_WIDTH 128

`define INSTR_BUFF_BITS (2)
`define INSTR_BUFF_DEEP (1 << `INSTR_BUFF_BITS)
module ifu_mod(
  output reg [`IFU_ADDR_WIDTH-1:0] ifu2bus_ar,
  output reg ifu2bus_ar_valid,
  input wire ifu2bus_ar_ready,
  input wire ifu2bus_r_valid,
  output reg ifu2bus_r_ready,
  input wire [`IFU_DATA_WIDTH-1:0] ifu2bus_data,

  input  clk,
  input  rst_n
  );

  reg [1:0] ld_state;
  reg [`IFU_ADDR_WIDTH-1:0] ifu_pc;
  reg [`IFU_DATA_WIDTH-1:0] ifu_data [`INSTR_BUFF_DEEP];

  wire[`INSTR_BUFF_BITS-1:0] queueslot;
  reg[`INSTR_BUFF_BITS-1:0] queue_c;
  reg[`INSTR_BUFF_BITS-1:0] queue_p;

  assign queueslot = queue_p - queue_c;

  wire ar_en;
  wire r_en;

  assign ar_en = ifu2bus_ar_ready & ifu2bus_ar_valid;
  assign r_en = ifu2bus_r_ready & ifu2bus_r_valid;
  /* instr loader ar */
  always @(posedge clk or negedge rst_n)
  begin 
    if(!rst_n) begin
      ifu2bus_ar <= `IFU_ADDR_WIDTH'b0;
      ifu2bus_ar_valid <= 1'b0;

      ld_state <= 2'b0;
      ifu_pc <= `IFU_ADDR_WIDTH'b0;
    end
    else if (ld_state == 2'b00) begin
      if (queueslot < `INSTR_BUFF_DEEP && !ifu2bus_ar_valid) begin
        ifu2bus_ar_valid <= 1'b1;
        ifu2bus_ar <= ifu_pc;
        ifu2bus_r_ready <= 1'b1;
        ifu_pc <= ifu_pc + 1;
      end
      else if (ar_en == 1'b1) begin
        ifu2bus_ar_valid <= 1'b0;
        ld_state <= 2'b01;
      end
      else begin
        ifu2bus_ar_valid <= 1'b0;
      end
    end
  end
  /* instr loader r */
  always @(posedge clk or negedge rst_n)
  begin 
    if(!rst_n) begin
      ifu2bus_r_ready <= 1'b0;
    end
    else if (ld_state == 2'b01) begin
      if (r_en == 1'b1) begin
        ifu_data[queue_p] <= ifu2bus_data;
        queue_p <= queue_p + 1;
        ld_state <= 2'b0;

        ifu2bus_r_ready <= 1'b0;
      end
    end
  end

  wire op_u8[`IFU_DATA_WIDTH * `INSTR_BUFF_DEEP]
  asign op_u8[][] = ifu_data[]
  /* instr emit */
  always @(posedge clk or negedge rst_n)
  begin 
    if(!rst_n) begin
      queue_c <= 1'b0;
      queue_p <= 1'b0;
    end
    else if (queue_p != queue_c)
      //op_next = ifu_data[queue_c][127:124]

      /* consume ifu_data */
      queue_c <= queue_c + 1;
    else
      queue_c <= queue_c;
  end



endmodule
