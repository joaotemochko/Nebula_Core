`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module branch_predictor
 * @brief Preditor de Branch híbrido: gshare + BTB + RAS
 *
 * @details
 * - BTB (Branch Target Buffer): 256 entradas, direct-mapped
 * - BHT (Branch History Table): 1024 entradas, 2-bit saturating counters
 * - gshare: XOR do PC com Global History Register
 * - RAS (Return Address Stack): 8 entradas para call/return
 *
 * Predição em 1 ciclo para branches diretos.
 */
module branch_predictor #(
    parameter int VADDR_WIDTH = 39,
    parameter int BTB_ENTRIES = 256,
    parameter int BHT_ENTRIES = 1024,
    parameter int RAS_DEPTH = 8,
    parameter int GHR_LEN = 10
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // =========================================================================
    // Interface de Predição (Fetch Stage)
    // =========================================================================
    input  wire [VADDR_WIDTH-1:0]   pc,
    input  wire                     predict_valid,
    
    output bp_prediction_t          prediction,
    
    // =========================================================================
    // Interface de Update (Execute/Commit Stage)
    // =========================================================================
    input  wire                     update_valid,
    input  bp_update_t              update
);

    // =========================================================================
    // Parâmetros Derivados
    // =========================================================================
    
    localparam int BTB_IDX_BITS = $clog2(BTB_ENTRIES);
    localparam int BHT_IDX_BITS = $clog2(BHT_ENTRIES);
    
    // =========================================================================
    // Estruturas
    // =========================================================================
    
    // BTB Entry
    typedef struct packed {
        logic                       valid;
        logic [VADDR_WIDTH-BTB_IDX_BITS-2-1:0] tag;
        logic [VADDR_WIDTH-1:0]     target;
        logic                       is_call;
        logic                       is_ret;
        logic                       is_unconditional;
    } btb_entry_t;
    
    // =========================================================================
    // Storage
    // =========================================================================
    
    // Branch Target Buffer
    btb_entry_t btb [BTB_ENTRIES];
    
    // Branch History Table (2-bit saturating counters)
    logic [1:0] bht [BHT_ENTRIES];
    
    // Global History Register
    logic [GHR_LEN-1:0] ghr;
    
    // Return Address Stack
    logic [VADDR_WIDTH-1:0] ras [RAS_DEPTH];
    logic [$clog2(RAS_DEPTH)-1:0] ras_ptr;
    
    // =========================================================================
    // Index Calculation
    // =========================================================================
    
    // BTB index: direct from PC
    wire [BTB_IDX_BITS-1:0] btb_idx = pc[BTB_IDX_BITS+1:2];  // Align to 4-byte
    wire [VADDR_WIDTH-BTB_IDX_BITS-2-1:0] btb_tag = pc[VADDR_WIDTH-1:BTB_IDX_BITS+2];
    
    // BHT index: gshare (PC XOR GHR)
    wire [BHT_IDX_BITS-1:0] bht_idx = pc[BHT_IDX_BITS+1:2] ^ {{(BHT_IDX_BITS-GHR_LEN){1'b0}}, ghr};
    
    // Update indices
    wire [BTB_IDX_BITS-1:0] btb_idx_upd = update.pc[BTB_IDX_BITS+1:2];
    wire [BHT_IDX_BITS-1:0] bht_idx_upd = update.pc[BHT_IDX_BITS+1:2] ^ 
                                          {{(BHT_IDX_BITS-GHR_LEN){1'b0}}, ghr};
    
    // =========================================================================
    // BTB Lookup
    // =========================================================================
    
    btb_entry_t btb_entry;
    logic btb_hit;
    
    always_comb begin
        btb_entry = btb[btb_idx];
        btb_hit = btb_entry.valid && (btb_entry.tag == btb_tag);
    end
    
    // =========================================================================
    // BHT Lookup
    // =========================================================================
    
    wire [1:0] bht_counter = bht[bht_idx];
    wire bht_taken = bht_counter[1];  // MSB indica predição
    
    // =========================================================================
    // RAS Lookup
    // =========================================================================
    
    wire [VADDR_WIDTH-1:0] ras_top = ras[ras_ptr];
    
    // =========================================================================
    // Prediction Output
    // =========================================================================
    
    always_comb begin
        prediction = '0;
        prediction.valid = predict_valid && btb_hit;
        prediction.confidence = bht_counter;
        
        if (btb_hit) begin
            if (btb_entry.is_ret) begin
                // Return: use RAS
                prediction.taken = 1'b1;
                prediction.target = ras_top;
                prediction.is_ret = 1'b1;
            end
            else if (btb_entry.is_unconditional) begin
                // JAL/JALR unconditional
                prediction.taken = 1'b1;
                prediction.target = btb_entry.target;
                prediction.is_call = btb_entry.is_call;
            end
            else begin
                // Conditional branch: use BHT
                prediction.taken = bht_taken;
                prediction.target = btb_entry.target;
            end
        end
    end
    
    // =========================================================================
    // 2-bit Saturating Counter Update
    // =========================================================================
    
    function automatic logic [1:0] update_counter(
        input logic [1:0] counter,
        input logic taken
    );
        if (taken) begin
            return (counter == 2'b11) ? 2'b11 : counter + 1;
        end else begin
            return (counter == 2'b00) ? 2'b00 : counter - 1;
        end
    endfunction
    
    // =========================================================================
    // Sequential Logic
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ghr <= '0;
            ras_ptr <= '0;
            
            for (int i = 0; i < BTB_ENTRIES; i++) begin
                btb[i].valid = 1'b0;
            end
            
            for (int i = 0; i < BHT_ENTRIES; i++) begin
                bht[i] = 2'b01;  // Weakly not taken
            end
            
            for (int i = 0; i < RAS_DEPTH; i++) begin
                ras[i] = '0;
            end
        end
        else begin
            // =================================================================
            // Prediction Phase: RAS Push (on call prediction)
            // =================================================================
            if (predict_valid && btb_hit && btb_entry.is_call) begin
                // Push return address (PC + 4 or PC + 2 for compressed)
                ras_ptr <= ras_ptr + 1;
                ras[ras_ptr + 1] <= pc + 4;  // Simplified, assume 4-byte
            end
            
            // RAS Pop (on return prediction)
            if (predict_valid && btb_hit && btb_entry.is_ret) begin
                ras_ptr <= ras_ptr - 1;
            end
            
            // =================================================================
            // Update Phase
            // =================================================================
            if (update_valid) begin
                // Update GHR (shift in actual outcome)
                ghr <= {ghr[GHR_LEN-2:0], update.taken};
                
                // Update BHT
                bht[bht_idx_upd] <= update_counter(bht[bht_idx_upd], update.taken);
                
                // Update BTB on taken branch or misprediction
                if (update.taken || update.mispredicted) begin
                    btb[btb_idx_upd].valid <= 1'b1;
                    btb[btb_idx_upd].tag <= update.pc[VADDR_WIDTH-1:BTB_IDX_BITS+2];
                    btb[btb_idx_upd].target <= update.target;
                    btb[btb_idx_upd].is_call <= update.is_call;
                    btb[btb_idx_upd].is_ret <= update.is_ret;
                    btb[btb_idx_upd].is_unconditional <= update.is_call || update.is_ret;
                end
                
                // Repair RAS on misprediction
                if (update.mispredicted) begin
                    if (update.is_call && !update.taken) begin
                        // Predicted call but wasn't taken: pop
                        ras_ptr <= ras_ptr - 1;
                    end
                    else if (update.is_ret && !update.taken) begin
                        // Predicted return but wasn't: push back
                        ras_ptr <= ras_ptr + 1;
                    end
                end
            end
        end
    end

endmodule
