`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module branch_predictor
 * @brief Preditor de Branch híbrido: gshare + BTB + RAS
 *
 * CORREÇÕES APLICADAS:
 * 1. Inicializações de btb[i].valid, bht[i] e ras[i] no bloco de reset
 *    mudadas de = para <= (BLKSEQ warning do Verilator 5.036).
 */
module branch_predictor #(
    parameter int VADDR_WIDTH = 39,
    parameter int BTB_ENTRIES = 256,
    parameter int BHT_ENTRIES = 1024,
    parameter int RAS_DEPTH   = 8,
    parameter int GHR_LEN     = 10
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Interface de Predição (Fetch Stage)
    input  wire [VADDR_WIDTH-1:0]   pc,
    input  wire                     predict_valid,
    output bp_prediction_t          prediction,

    // Interface de Update (Execute/Commit Stage)
    input  wire                     update_valid,
    input  bp_update_t              update
);

    localparam int BTB_IDX_BITS = $clog2(BTB_ENTRIES);
    localparam int BHT_IDX_BITS = $clog2(BHT_ENTRIES);

    // =========================================================================
    // BTB Entry
    // =========================================================================
    typedef struct packed {
        logic                                        valid;
        logic [VADDR_WIDTH-BTB_IDX_BITS-2-1:0]      tag;
        logic [VADDR_WIDTH-1:0]                      target;
        logic                                        is_call;
        logic                                        is_ret;
        logic                                        is_unconditional;
    } btb_entry_t;

    // =========================================================================
    // Storage
    // =========================================================================
    btb_entry_t         btb [BTB_ENTRIES];
    logic [1:0]         bht [BHT_ENTRIES];
    logic [GHR_LEN-1:0] ghr;
    logic [VADDR_WIDTH-1:0] ras [RAS_DEPTH];
    logic [$clog2(RAS_DEPTH)-1:0] ras_ptr;

    // =========================================================================
    // Index Calculation
    // =========================================================================
    wire [BTB_IDX_BITS-1:0] btb_idx     = pc[BTB_IDX_BITS+1:2];
    wire [VADDR_WIDTH-BTB_IDX_BITS-2-1:0] btb_tag = pc[VADDR_WIDTH-1:BTB_IDX_BITS+2];
    wire [BHT_IDX_BITS-1:0] bht_idx     = pc[BHT_IDX_BITS+1:2] ^
                                          {{(BHT_IDX_BITS-GHR_LEN){1'b0}}, ghr};

    wire [BTB_IDX_BITS-1:0] btb_idx_upd = update.pc[BTB_IDX_BITS+1:2];
    wire [BHT_IDX_BITS-1:0] bht_idx_upd = update.pc[BHT_IDX_BITS+1:2] ^
                                          {{(BHT_IDX_BITS-GHR_LEN){1'b0}}, ghr};

    // =========================================================================
    // BTB Lookup
    // =========================================================================
    btb_entry_t btb_entry;
    logic       btb_hit;

    always_comb begin
        btb_entry = btb[btb_idx];
        btb_hit   = btb_entry.valid && (btb_entry.tag == btb_tag);
    end

    // =========================================================================
    // BHT Lookup
    // =========================================================================
    wire [1:0] bht_counter = bht[bht_idx];
    wire       bht_taken   = bht_counter[1];

    // =========================================================================
    // RAS Lookup
    // =========================================================================
    wire [VADDR_WIDTH-1:0] ras_top = ras[ras_ptr];

    // =========================================================================
    // Prediction Output (combinacional)
    // =========================================================================
    always_comb begin
        prediction        = '0;
        prediction.valid  = predict_valid && btb_hit;
        prediction.confidence = bht_counter;

        if (btb_hit) begin
            if (btb_entry.is_ret) begin
                prediction.taken   = 1'b1;
                prediction.target  = ras_top;
                prediction.is_ret  = 1'b1;
            end
            else if (btb_entry.is_unconditional) begin
                prediction.taken   = 1'b1;
                prediction.target  = btb_entry.target;
                prediction.is_call = btb_entry.is_call;
            end
            else begin
                prediction.taken  = bht_taken;
                prediction.target = btb_entry.target;
            end
        end
    end

    // =========================================================================
    // 2-bit Saturating Counter
    // =========================================================================
    function automatic logic [1:0] update_counter(
        input logic [1:0] counter,
        input logic       taken
    );
        if (taken)
            return (counter == 2'b11) ? 2'b11 : counter + 1;
        else
            return (counter == 2'b00) ? 2'b00 : counter - 1;
    endfunction

    // =========================================================================
    // Sequential Logic
    // FIX: inicializações de arrays usam <= (não =) para evitar BLKSEQ
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ghr     <= '0;
            ras_ptr <= '0;

            // FIX: <= em vez de = para arrays dentro de always_ff
            for (int i = 0; i < BTB_ENTRIES; i++)
                btb[i].valid <= 1'b0;

            for (int i = 0; i < BHT_ENTRIES; i++)
                bht[i] <= 2'b01;   // weakly not-taken

            for (int i = 0; i < RAS_DEPTH; i++)
                ras[i] <= '0;
        end
        else begin
            // -----------------------------------------------------------------
            // Prediction Phase: RAS push on predicted call
            // -----------------------------------------------------------------
            if (predict_valid && btb_hit && btb_entry.is_call) begin
                ras[ras_ptr + 1] <= pc + 4;
                ras_ptr          <= ras_ptr + 1;
            end

            if (predict_valid && btb_hit && btb_entry.is_ret) begin
                ras_ptr <= ras_ptr - 1;
            end

            // -----------------------------------------------------------------
            // Update Phase
            // -----------------------------------------------------------------
            if (update_valid) begin
                // Atualizar GHR
                ghr <= {ghr[GHR_LEN-2:0], update.taken};

                // Atualizar BHT
                bht[bht_idx_upd] <= update_counter(bht[bht_idx_upd], update.taken);

                // Atualizar BTB em branch taken ou mispredição
                if (update.taken || update.mispredicted) begin
                    btb[btb_idx_upd].valid          <= 1'b1;
                    btb[btb_idx_upd].tag            <= update.pc[VADDR_WIDTH-1:BTB_IDX_BITS+2];
                    btb[btb_idx_upd].target         <= update.target;
                    btb[btb_idx_upd].is_call        <= update.is_call;
                    btb[btb_idx_upd].is_ret         <= update.is_ret;
                    btb[btb_idx_upd].is_unconditional <= update.is_call || update.is_ret;
                end

                // Reparar RAS em mispredição
                if (update.mispredicted) begin
                    if (update.is_call && !update.taken)
                        ras_ptr <= ras_ptr - 1;
                    else if (update.is_ret && !update.taken)
                        ras_ptr <= ras_ptr + 1;
                end
            end
        end
    end

endmodule
