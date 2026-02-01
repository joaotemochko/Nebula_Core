`timescale 1ns/1ps
`default_nettype none

/**
 * @module reservation_set
 * @brief Gerenciador de Reservas para LR/SC (Load-Reserved / Store-Conditional)
 *
 * @details Implementa o conjunto de reservas necessário para instruções atômicas
 * LR.W, LR.D, SC.W, SC.D conforme especificação RISC-V.
 *
 * Regras de reserva:
 * - LR cria uma reserva no endereço especificado
 * - SC só sucede se a reserva ainda for válida
 * - Reserva é invalidada por: SC (sucesso ou falha), store de outro hart,
 *   interrupção/exceção, ou SFENCE.VMA
 *
 * Suporta múltiplos harts com invalidação cruzada.
 */
module reservation_set #(
    parameter int NUM_HARTS = 4,
    parameter int ADDR_WIDTH = 64,
    parameter int RESERVATION_GRANULE = 64  // Bytes - mínimo segundo spec
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // Interface de Reserva (por hart)
    input  wire [$clog2(NUM_HARTS)-1:0] hart_id,
    
    // LR - Load Reserved
    input  wire                     lr_valid,
    input  wire [ADDR_WIDTH-1:0]    lr_addr,
    input  wire                     lr_is_word,  // 1=LR.W, 0=LR.D
    
    // SC - Store Conditional
    input  wire                     sc_valid,
    input  wire [ADDR_WIDTH-1:0]    sc_addr,
    input  wire                     sc_is_word,  // 1=SC.W, 0=SC.D
    output logic                    sc_success,  // 1=sucesso (rd=0), 0=falha (rd=1)
    
    // Invalidação externa
    input  wire                     invalidate_all,        // SFENCE.VMA ou interrupção
    input  wire                     store_from_other_hart, // Store de outro hart
    input  wire [ADDR_WIDTH-1:0]    other_store_addr,
    input  wire [$clog2(NUM_HARTS)-1:0] other_hart_id,
    
    // Status
    output logic [NUM_HARTS-1:0]    reservation_valid
);

    // Estrutura de reserva por hart
    typedef struct packed {
        logic                   valid;
        logic [ADDR_WIDTH-1:0]  addr;
        logic                   is_word;  // Tamanho da reserva
    } reservation_t;
    
    reservation_t reservations [NUM_HARTS];
    
    // Máscara de endereço para granularidade de reserva
    // A spec permite reservas maiores que o acesso, usamos RESERVATION_GRANULE bytes
    localparam ADDR_MASK_BITS = $clog2(RESERVATION_GRANULE);
    
    function automatic logic addr_match(
        input [ADDR_WIDTH-1:0] addr1,
        input [ADDR_WIDTH-1:0] addr2
    );
        // Compara endereços ignorando bits de offset dentro do granule
        return (addr1[ADDR_WIDTH-1:ADDR_MASK_BITS] == addr2[ADDR_WIDTH-1:ADDR_MASK_BITS]);
    endfunction
    
    // Status de saída
    always_comb begin
        for (int i = 0; i < NUM_HARTS; i++) begin
            reservation_valid[i] = reservations[i].valid;
        end
    end
    
    // Lógica de SC success
    always_comb begin
        sc_success = 1'b0;
        
        if (sc_valid) begin
            // SC sucede se:
            // 1. Hart tem reserva válida
            // 2. Endereço corresponde à reserva
            // 3. Tamanho corresponde (W ou D)
            if (reservations[hart_id].valid &&
                addr_match(reservations[hart_id].addr, sc_addr) &&
                reservations[hart_id].is_word == sc_is_word) begin
                sc_success = 1'b1;
            end
        end
    end
    
    // Lógica Sequencial
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_HARTS; i++) begin
                reservations[i].valid <= 1'b0;
                reservations[i].addr <= '0;
                reservations[i].is_word <= 1'b0;
            end
        end else begin
            // Invalidação global (SFENCE.VMA, interrupção)
            if (invalidate_all) begin
                for (int i = 0; i < NUM_HARTS; i++) begin
                    reservations[i].valid <= 1'b0;
                end
            end else begin
                // LR - Criar nova reserva
                if (lr_valid) begin
                    reservations[hart_id].valid <= 1'b1;
                    reservations[hart_id].addr <= lr_addr;
                    reservations[hart_id].is_word <= lr_is_word;
                end
                
                // SC - Sempre invalida a reserva do hart (sucesso ou falha)
                if (sc_valid) begin
                    reservations[hart_id].valid <= 1'b0;
                end
                
                // Store de outro hart - Invalida reservas que colidem
                if (store_from_other_hart) begin
                    for (int i = 0; i < NUM_HARTS; i++) begin
                        if (i != other_hart_id &&
                            reservations[i].valid &&
                            addr_match(reservations[i].addr, other_store_addr)) begin
                            reservations[i].valid <= 1'b0;
                        end
                    end
                end
            end
        end
    end

endmodule
