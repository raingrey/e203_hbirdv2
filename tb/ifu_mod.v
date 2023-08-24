//`include "ifu_define.v"

`define IFU_ADDR_WIDTH 48
`define IFU_DATA_WIDTH 128
`define IFU_CH_SIZE 16

/* pipeline list
 * pipeline0: fence and dep_graph fence
 * pipeline1: predict with predict_reg/predict_pc
 * pipeline2: loop back/forward loop_reg_pc offset with loop_reg_times
 * pipeline3: reg edit shift/bit replace/self-add/self-sub/set bit/clear bit
 * pipeline4: data move load addr2reg/store reg2addr/move addr2addr
 */
`define IFU_PIPELINE_NUM 5

`define INSTR_BUFF_BITS (2)
`define INSTR_BUFF_DEEP (1 << `INSTR_BUFF_BITS)
module ifu_mod(
  output reg [`IFU_ADDR_WIDTH-1:0] ifu2bus_ar,
  output reg ifu2bus_ar_valid,
  input wire ifu2bus_ar_ready,
  input wire ifu2bus_r_valid,
  output reg ifu2bus_r_ready,
  input wire [`IFU_DATA_WIDTH-1:0] ifu2bus_data,

  input wire ifu_enable,
  input wire [`IFU_ADDR_WIDTH-1:0] ifu_initial_pc,

  input wire [`IFU_CH_SIZE] dep_graph_inc_in,
  output reg [`IFU_CH_SIZE] dep_graph_inc_out,

  input  clk,
  input  rst_n
  );
  reg chnl_produce[`IFU_CH_SIZE];
  reg chnl_consume[`IFU_CH_SIZE];
  genvar j;
  generate
    for (j = 0; j < `IFU_CH_SIZE; j++)
    begin: dep_graph_monitor
        always @(posedge clk or negedge rst_n)
        begin 
          if (dep_graph_inc_in[i] != 0) chnl_produce[i] <= chnl_produce[i] + 1;
        end
    end
  endgenerate
  /* ld_state: 0->1 start from initial_pc
   *           1->2 ar handshake send
   *           2->1 r data handshake receive
   *           1->3 pc switch and buf clean
   *           3->2 fetch again
   */
  reg [1:0] ld_state;
  reg [`IFU_ADDR_WIDTH-1:0] ifu_pc;
  reg [`IFU_ADDR_WIDTH-1:0] pending_pc;
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

      ld_state <= 2'b1;
      ifu_pc <= `IFU_ADDR_WIDTH'b0;
    end
    else if (ld_state == 2'01) begin
      if (ifu_enable) begin
        if (queueslot < `INSTR_BUFF_DEEP && !ifu2bus_ar_valid) begin
          ifu2bus_ar_valid <= 1'b1;
          ifu2bus_ar <= ifu_pc;
          ifu_pc <= ifu_pc + 1;
        end
        else if (ar_en == 1'b1) begin
          ifu2bus_ar_valid <= 1'b0;
          ifu2bus_r_ready <= 1'b1;
          ld_state <= 2'b10;
        else
          ld_state <= 2'b01;
        end
      end else begin
        ifu_pc <= ifu_initial_pc;
        ld_state <= 2'b01;
      end
    end
    else if (ld_state == 2'10) begin
      if (r_en == 1'b1) begin
        ifu_data[queue_p] <= ifu2bus_data;
        queue_p <= queue_p + 1;
        ld_state <= 2'b1;

        ifu2bus_r_ready <= 1'b0;
      end else begin
        ifu2bus_r_ready <= 1'b1;
      end
    end else begin
      ifu_pc <= ifu_initial_pc;
      ld_state <= 2'b01;
    end
  end

  wire [3:0] op_sz;
  wire [3:0] op_code0;
  wire [7:0] op_code1;
  assign op_sz = ifu_data[queue_c][`IFU_DATA_WIDTH-1:`IFU_DATA_WIDTH-4];
  assign op_code0 = ifu_data[queue_c][`IFU_DATA_WIDTH-5:`IFU_DATA_WIDTH-8];
  assign op_code1 = ifu_data[queue_c][`IFU_DATA_WIDTH-9:`IFU_DATA_WIDTH-16];

  assign op_code32_2 = ifu_data[queue_c][`IFU_DATA_WIDTH-17:`IFU_DATA_WIDTH-48];
  assign op_code32_3 = ifu_data[queue_c][`IFU_DATA_WIDTH-49:`IFU_DATA_WIDTH-80];

  assign op_code16_2 = ifu_data[queue_c][`IFU_DATA_WIDTH-17:`IFU_DATA_WIDTH-32];
  assign op_code16_3 = ifu_data[queue_c][`IFU_DATA_WIDTH-33:`IFU_DATA_WIDTH-48];

  assign op_code8_2 = ifu_data[queue_c][`IFU_DATA_WIDTH-17:`IFU_DATA_WIDTH-24];
  assign op_code8_3 = ifu_data[queue_c][`IFU_DATA_WIDTH-25:`IFU_DATA_WIDTH-32];

  reg dec16;
  reg [3:0] dec16_op_code0;
  reg [7:0] dec16_op_code1;
  reg dec32;
  reg [3:0] dec32_op_code0;
  reg [7:0] dec32_op_code1;
  reg dec64;
  reg [3:0] dec64_op_code0;
  reg [7:0] dec64_op_code1;
  reg dec128;
  reg [3:0] dec128_op_code0;
  reg [7:0] dec128_op_code1;

  reg channel_sync;
  /* cycle 0 */
  /* instr emit */
  always @(posedge clk or negedge rst_n)
  begin
    if(!rst_n) begin
      queue_c <= 1'b0;
      queue_p <= 1'b0;
      channel_sync <= 1'b0;
    end
    else if (queue_p != queue_c) begin
      if (op_sz == 4'b0) begin
            /* consume ifu_data */
            queue_c <= queue_c + 1;
      end
      else begin
        case (op_code0)
          /* fence */
          4'h0:
            begin
              case (op_code1)
                /* pipline front fence */
                8'hff:
                  begin
                    if (pipeline_state == `IFU_PIPELINE_NUM'b0)
                      ifu_data[queue_c] <= ifu_data[queue_c];
                    else
                      ifu_data[queue_c] <= ifu_data[queue_c] << op_sz;
                    end
                  end
                /* pipline dep_graph fence */
                default
                begin
                  if (chnl_consume[op_code1] !=
                      chnl_produce[op_code1])
                    begin
                      chnl_consume[op_code1] <=
                        chnl_consume[op_code1] + 1; 
                      ifu_data[queue_c] <= ifu_data[queue_c] << op_sz;
                    end
                  else begin
                      ifu_data[queue_c] <= ifu_data[queue_c];
                  end
                end
              endcase
            end
          /* predict */
          4'h1:
            begin
              case (op_code1)
                /* force jump. relative addressing */
                8'h0:
                  begin
                    if (ifu_enable)
                    begin
                      ifu_enable <= 1'b0;
                      ifu_initial_pc <= op_code32_2;
                      ifu_data <= `IFU_DATA_WIDTH'b0;
                    end else if (!ifu_enable && ld_state == 2'b1) begin
                      ifu_enable <= 1'b1;
                      queue_p <= queue_c;
                    else
                      ifu_enable <= 1'b0;
                    end
                  end
                /* force jump. relative addressing */
                8'h1:
                  if (ifu_enable) begin
                    ifu_enable <= 1'b0;
                    predict_reg[predict_pc] == 1'b0 ?
                        ifu_initial_pc <= op_code32_2 : ifu_initial_pc <= op_code32_3;
                    pridict_pc <= predict_pc + 1;
                    ifu_data <= `IFU_DATA_WIDTH'b0;
                  end else if (!ifu_enable && ld_state == 2'b1) begin
                    ifu_enable <= 1'b1;
                    queue_p <= queue_c;
                  else
                    ifu_enable <= 1'b0;
                  end
                 
                8'h2:
                default
              endcase
            end
          /* loop back */
          4'h2: begin
            if (common_reg[op_code1] && ifu_enable) begin
              ifu_enable <= 1'b0;
              common_reg[op_code1] <= common_reg[op_code1] - 1;
              ifu_initial_pc <= op_code32_2;
              ifu_data <= `IFU_DATA_WIDTH'b0;
            end else if (!ifu_enable && ld_state == 2'b1) begin
              ifu_enable <= 1'b1;
              queue_p <= queue_c;
            else
              ifu_enable <= 1'b0;
            end
          end
          /* reg edit */
          4'h3: begin
          /* op_code_1: 0 self_add 
          /* op_code_1: 1 self_sub
          /* op_code_1: 2 reg X bits change, from A size B with value C(size is B) or value in reg
          /* op_code_1: 3 reg X bits shift Left A bits
          /* op_code_1: 4 reg X bits shift Right B bits
           */
          end
          /* TODO: Maybe consume more channel in one op is better */
          /* dep inc */
          4'h4: begin
            if (dep_graph_inc_out[op_code1]) begin
              dep_graph_inc_out[op_code1] <= 1'b0;
              ifu_data[queue_c] <= ifu_data[queue_c] << op_sz;
            end else begin
              dep_graph_inc_out[op_code1] <= 1'b1;
            end
          end
          /* data move */
          4'h5: begin
          end
        endcase
          


















        /* front fence */
        if (op_code0 == 1'b0 && op_code1 == 8'hff) begin
        end
        else begin
          case (op_sz)
            4'b1:
              begin
                if (dec16 == 0) begin
                  ifu_data[queue_c] <= ifu_data[queue_c] << 16;
                  dec16 <= 1'b1;
                  dec16_op_code0 <= op_code0;
                  dec16_op_code1 <= op_code1;
                end
                else
                  /* 16 pipeline stall */
                  ifu_data[queue_c] <= ifu_data[queue_c];
                end
              end
            4'b10:
              begin
                if (dec32 == 0) begin
                  ifu_data[queue_c] <= ifu_data[queue_c] << 32;
                  dec32 <= 1'b1;
                  dec32_op_code0 <= op_code0;
                  dec32_op_code1 <= op_code1;
                end
                else
                  /* 32 pipeline stall */
                  ifu_data[queue_c] <= ifu_data[queue_c];
                end
              end
            4'b11:
              begin
                if (dec64 == 0) begin
                  ifu_data[queue_c] <= ifu_data[queue_c] << 64;
                  dec64 <= 1'b1;
                  dec64_op_code0 <= op_code0;
                  dec64_op_code1 <= op_code1;
                end
                else
                  /* 64 pipeline stall */
                  ifu_data[queue_c] <= ifu_data[queue_c];
                end
  
            4'b100:
              begin
              end
  
            default:
              /* unsuported op size */
          endcase
        end
      end
    end
    else
      queue_c <= queue_c;

  end
  /* cycle 0 */

  /* cycle 1 */
  /* 16 pipeline */
  always @(posedge clk or negedge rst_n)
  begin 
    if(!rst_n) begin
      dec16 <= 1'b0;
    end
    else if(dec16 != 0) begin
      case (dec16_op_code0)
        /* = 0: front fence
         * > 0: fence by dep_graph
         */
        4'b0:
          begin
              if (chnl_consume[dec16_op_code1] != chnl_produce[dec16_op_code1])
              begin
                chnl_consume[dec16_op_code1] <= chnl_consume[dec16_op_code1] + 1; 
                out_sync <= 1'b0;
              end
              else begin
                out_sync <= 1'b1;
              end
          end
        /* predict */
        4'b1:
          begin
            case (dec16_op_code1)
              8'b0:
                begin

                end
            endcase
          end
        4'b10:
          begin
            case (dec16_op_code1)
              8'b0:
                begin
                end
            endcase
          end
      endcase
    end
  end
  always @(posedge clk or negedge rst_n)
  begin 
    if(!rst_n) begin
    end
    else if (queue_p != queue_c && global_fence == 0'b0) begin
        case (op_code0)
          /* golbal fence */
          4'b0:
            begin
            end
          /* assign code to sub-IFU channel */
          4'b1:
            begin
            end
        endcase
    end
    else begin
    end
  end


  /* cycle 0 */


endmodule
