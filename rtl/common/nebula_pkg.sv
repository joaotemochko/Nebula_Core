`ifndef NEBULA_PKG_SV
`define NEBULA_PKG_SV

/**
 * @package nebula_pkg
 * @brief Definições globais para o Nebula Cluster
 * 
 * ISA: RV64GC (RV64IMAFDC + Zicsr + Zifencei)
 * Para: Ubuntu Linux RV64
 */
package nebula_pkg;

    // =========================================================================
    // Parâmetros do Cluster
    // =========================================================================
    
    parameter int NUM_CORES = 4;
    parameter int CLUSTER_ID_WIDTH = 4;
    
    // =========================================================================
    // Parâmetros de Arquitetura
    // =========================================================================
    
    parameter int XLEN = 64;
    parameter int FLEN = 64;                 // Double-precision FP
    parameter int ILEN = 32;                 // Instrução não-comprimida
    parameter int CLEN = 16;                 // Instrução comprimida
    parameter int PADDR_WIDTH = 56;
    parameter int VADDR_WIDTH = 39;          // Sv39
    parameter int VPN_WIDTH = 27;
    parameter int PPN_WIDTH = 44;
    parameter int ASID_WIDTH = 16;
    parameter int NUM_INT_REGS = 32;
    parameter int NUM_FP_REGS = 32;
    
    // =========================================================================
    // Parâmetros de Cache L1
    // =========================================================================
    
    parameter int L1I_SIZE_KB = 32;
    parameter int L1I_WAYS = 4;
    parameter int L1I_LINE_SIZE = 64;
    parameter int L1I_SETS = (L1I_SIZE_KB * 1024) / (L1I_WAYS * L1I_LINE_SIZE);
    
    parameter int L1D_SIZE_KB = 32;
    parameter int L1D_WAYS = 4;
    parameter int L1D_LINE_SIZE = 64;
    parameter int L1D_SETS = (L1D_SIZE_KB * 1024) / (L1D_WAYS * L1D_LINE_SIZE);
    
    // =========================================================================
    // Parâmetros de Cache L2 (Compartilhado)
    // =========================================================================
    
    parameter int L2_SIZE_KB = 512;          // 512KB compartilhado
    parameter int L2_WAYS = 8;
    parameter int L2_LINE_SIZE = 64;
    parameter int L2_SETS = (L2_SIZE_KB * 1024) / (L2_WAYS * L2_LINE_SIZE);
    parameter int L2_BANKS = 4;              // 4 bancos para paralelismo
    
    // =========================================================================
    // Parâmetros de TLB
    // =========================================================================
    
    parameter int ITLB_ENTRIES = 32;
    parameter int DTLB_ENTRIES = 32;
    parameter int L2_TLB_ENTRIES = 512;      // Shared L2 TLB
    
    // =========================================================================
    // Parâmetros do Branch Predictor
    // =========================================================================
    
    parameter int BTB_ENTRIES = 256;         // Branch Target Buffer
    parameter int BHT_ENTRIES = 1024;        // Branch History Table
    parameter int RAS_DEPTH = 8;             // Return Address Stack
    parameter int GSHARE_HISTORY_LEN = 10;   // Global history bits
    
    // =========================================================================
    // Tipos de Dados Base
    // =========================================================================
    
    typedef logic [XLEN-1:0]        xreg_t;      // Integer register
    typedef logic [FLEN-1:0]        freg_t;      // FP register
    typedef logic [PADDR_WIDTH-1:0] paddr_t;
    typedef logic [VADDR_WIDTH-1:0] vaddr_t;
    typedef logic [ILEN-1:0]        instr_t;
    typedef logic [CLEN-1:0]        cinstr_t;    // Compressed instruction
    typedef logic [4:0]             reg_addr_t;
    typedef logic [11:0]            csr_addr_t;
    typedef logic [$clog2(NUM_CORES)-1:0] core_id_t;
    
    // =========================================================================
    // Níveis de Privilégio
    // =========================================================================
    
    typedef enum logic [1:0] {
        PRIV_USER       = 2'b00,
        PRIV_SUPERVISOR = 2'b01,
        PRIV_RESERVED   = 2'b10,
        PRIV_MACHINE    = 2'b11
    } priv_t;
    
    // =========================================================================
    // Rounding Modes (FPU)
    // =========================================================================
    
    typedef enum logic [2:0] {
        RM_RNE = 3'b000,  // Round to Nearest, ties to Even
        RM_RTZ = 3'b001,  // Round towards Zero
        RM_RDN = 3'b010,  // Round Down (towards -∞)
        RM_RUP = 3'b011,  // Round Up (towards +∞)
        RM_RMM = 3'b100,  // Round to Nearest, ties to Max Magnitude
        RM_DYN = 3'b111   // Dynamic (use frm CSR)
    } rounding_mode_t;
    
    // =========================================================================
    // FPU Exception Flags
    // =========================================================================
    
    typedef struct packed {
        logic NV;  // Invalid Operation
        logic DZ;  // Divide by Zero
        logic OF;  // Overflow
        logic UF;  // Underflow
        logic NX;  // Inexact
    } fflags_t;
    
    // =========================================================================
    // FPU Operation Types
    // =========================================================================
    
    typedef enum logic [4:0] {
        FPU_ADD,
        FPU_SUB,
        FPU_MUL,
        FPU_DIV,
        FPU_SQRT,
        FPU_SGNJ,       // Sign injection
        FPU_SGNJN,
        FPU_SGNJX,
        FPU_MIN,
        FPU_MAX,
        FPU_CVT_W,      // Convert to int32
        FPU_CVT_WU,     // Convert to uint32
        FPU_CVT_L,      // Convert to int64
        FPU_CVT_LU,     // Convert to uint64
        FPU_CVT_S,      // Convert to single
        FPU_CVT_D,      // Convert to double
        FPU_CVT_INT,    // Convert from int
        FPU_MV_X_W,     // Move to integer reg
        FPU_MV_W_X,     // Move from integer reg
        FPU_CLASS,      // Classify
        FPU_CMP_EQ,     // Compare equal
        FPU_CMP_LT,     // Compare less than
        FPU_CMP_LE,     // Compare less or equal
        FPU_MADD,       // Fused multiply-add
        FPU_MSUB,       // Fused multiply-sub
        FPU_NMADD,      // Negated fused multiply-add
        FPU_NMSUB       // Negated fused multiply-sub
    } fpu_op_t;
    
    // =========================================================================
    // Causas de Exceção
    // =========================================================================
    
    typedef enum logic [5:0] {
        EXC_INSTR_MISALIGNED    = 6'd0,
        EXC_INSTR_ACCESS_FAULT  = 6'd1,
        EXC_ILLEGAL_INSTR       = 6'd2,
        EXC_BREAKPOINT          = 6'd3,
        EXC_LOAD_MISALIGNED     = 6'd4,
        EXC_LOAD_ACCESS_FAULT   = 6'd5,
        EXC_STORE_MISALIGNED    = 6'd6,
        EXC_STORE_ACCESS_FAULT  = 6'd7,
        EXC_ECALL_U             = 6'd8,
        EXC_ECALL_S             = 6'd9,
        EXC_ECALL_M             = 6'd11,
        EXC_INSTR_PAGE_FAULT    = 6'd12,
        EXC_LOAD_PAGE_FAULT     = 6'd13,
        EXC_STORE_PAGE_FAULT    = 6'd15
    } exception_cause_t;
    
    // =========================================================================
    // Branch Prediction Types
    // =========================================================================
    
    typedef struct packed {
        logic           valid;
        logic           taken;
        vaddr_t         target;
        logic           is_call;
        logic           is_ret;
        logic [1:0]     confidence;    // 2-bit saturating counter
    } bp_prediction_t;
    
    typedef struct packed {
        logic           valid;
        logic           taken;
        logic           mispredicted;
        vaddr_t         pc;
        vaddr_t         target;
        logic           is_call;
        logic           is_ret;
    } bp_update_t;
    
    // =========================================================================
    // Estrutura de Instrução Decodificada
    // =========================================================================
    
    typedef struct packed {
        logic [6:0]     opcode;
        reg_addr_t      rd;
        logic [2:0]     funct3;
        reg_addr_t      rs1;
        reg_addr_t      rs2;
        reg_addr_t      rs3;           // Para FMA
        logic [6:0]     funct7;
        xreg_t          imm;
        csr_addr_t      csr_addr;
        logic           valid;
        logic           is_compressed;
        // Integer
        logic           is_alu;
        logic           is_alu_w;
        logic           is_branch;
        logic           is_jal;
        logic           is_jalr;
        logic           is_load;
        logic           is_store;
        logic           is_amo;
        logic           is_lr;
        logic           is_sc;
        logic           is_mdu;
        // FP
        logic           is_fp;
        logic           is_fp_load;
        logic           is_fp_store;
        logic           is_fma;        // Fused multiply-add
        logic           is_fp_single;  // Single precision
        logic           is_fp_double;  // Double precision
        fpu_op_t        fpu_op;
        rounding_mode_t rm;
        // System
        logic           is_csr;
        logic           is_fence;
        logic           is_fence_i;
        logic           is_sfence_vma;
        logic           is_ecall;
        logic           is_ebreak;
        logic           is_mret;
        logic           is_sret;
        logic           is_wfi;
        // PC
        vaddr_t         pc;
        // Branch prediction
        bp_prediction_t bp_pred;
    } decoded_instr_t;
    
    // =========================================================================
    // Resultado de Execução
    // =========================================================================
    
    typedef struct packed {
        xreg_t          result;
        freg_t          fp_result;
        vaddr_t         mem_addr;
        xreg_t          store_data;
        freg_t          fp_store_data;
        reg_addr_t      rd;
        logic [7:0]     mem_wstrb;
        logic           int_reg_we;
        logic           fp_reg_we;
        logic           mem_we;
        logic           mem_re;
        logic           is_fp_mem;     // FP load/store
        logic           branch_taken;
        vaddr_t         branch_target;
        logic           is_amo;
        logic           is_lr;
        logic           is_sc;
        logic [4:0]     amo_op;
        logic           amo_aq;
        logic           amo_rl;
        logic [2:0]     mem_size;
        logic           trap;
        exception_cause_t trap_cause;
        xreg_t          trap_value;
        vaddr_t         pc;
        fflags_t        fflags;        // FPU exception flags
        bp_prediction_t bp_pred;       // For update
    } exec_result_t;
    
    // =========================================================================
    // Interface Frontend -> Backend
    // =========================================================================
    
    typedef struct packed {
        decoded_instr_t instr0;
        decoded_instr_t instr1;
        logic           instr0_valid;
        logic           instr1_valid;
        logic           dual_issue;
    } frontend_packet_t;
    
    // =========================================================================
    // Interface Backend -> Frontend
    // =========================================================================
    
    typedef struct packed {
        logic           stall;
        logic           flush;
        logic           redirect;
        vaddr_t         redirect_pc;
        bp_update_t     bp_update;
    } backend_ctrl_t;
    
    // =========================================================================
    // Cache Coherence (MESI-like)
    // =========================================================================
    
    typedef enum logic [1:0] {
        CACHE_INVALID   = 2'b00,
        CACHE_SHARED    = 2'b01,
        CACHE_EXCLUSIVE = 2'b10,
        CACHE_MODIFIED  = 2'b11
    } cache_state_t;
    
    typedef enum logic [2:0] {
        COH_NONE,
        COH_READ,
        COH_READ_EX,
        COH_UPGRADE,
        COH_WRITEBACK,
        COH_INVALIDATE
    } coherence_op_t;
    
    // =========================================================================
    // L2 Request/Response
    // =========================================================================
    
    typedef struct packed {
        logic           valid;
        core_id_t       core_id;
        logic           is_ifetch;
        logic           is_write;
        logic           is_amo;
        logic [4:0]     amo_op;
        paddr_t         addr;
        logic [L1D_LINE_SIZE*8-1:0] wdata;
        logic           upgrade;       // Shared -> Modified
    } l2_req_t;
    
    typedef struct packed {
        logic           valid;
        core_id_t       core_id;
        logic           is_ifetch;
        logic [L2_LINE_SIZE*8-1:0] rdata;
        logic           error;
        cache_state_t   state;
    } l2_resp_t;
    
    // =========================================================================
    // Snoop Interface (Coherence)
    // =========================================================================
    
    typedef struct packed {
        logic           valid;
        coherence_op_t  op;
        paddr_t         addr;
        core_id_t       requester;
    } snoop_req_t;
    
    typedef struct packed {
        logic           valid;
        logic           has_data;
        logic [L1D_LINE_SIZE*8-1:0] data;
        cache_state_t   state;
    } snoop_resp_t;
    
    // =========================================================================
    // Funções Auxiliares
    // =========================================================================
    
    function automatic xreg_t sign_extend_load(
        input xreg_t data,
        input logic [2:0] size
    );
        case (size)
            3'b000: return {{56{data[7]}}, data[7:0]};
            3'b001: return {{48{data[15]}}, data[15:0]};
            3'b010: return {{32{data[31]}}, data[31:0]};
            3'b011: return data;
            3'b100: return {56'b0, data[7:0]};
            3'b101: return {48'b0, data[15:0]};
            3'b110: return {32'b0, data[31:0]};
            default: return data;
        endcase
    endfunction
    
    // Check if instruction is compressed (16-bit)
    function automatic logic is_compressed_instr(input logic [15:0] instr);
        return (instr[1:0] != 2'b11);
    endfunction
    
endpackage

`endif
