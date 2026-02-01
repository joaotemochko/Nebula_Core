`ifndef STU_PKG_SV
`define STU_PKG_SV

/**
 * @package stu_pkg
 * @brief Define os tipos de dados e parâmetros globais para a 
 * STU (Speculative Threading Unit) Adaptativa.
 */
package stu_pkg;

  // --- Parâmetros de Configuração da Arquitetura ---
  parameter int NUM_CORES   = 4;      // Total de núcleos no cluster
  parameter int ADDR_WIDTH  = 64;     // Arquitetura 64 bits
  parameter int INSTR_WIDTH = 32;     // RISC-V (padrão)
  parameter int REG_WIDTH   = 64;     // Largura do registrador
  parameter int NUM_REGS    = 32;     // 32 registradores

  // --- Definições de Tipo ---

  // Tipo para endereços de memória e Program Counter
  typedef logic [ADDR_WIDTH-1:0]   addr_t;

  // Tipo para uma única instrução RISC-V
  typedef logic [INSTR_WIDTH-1:0]  instr_t;

  // Tipo para identificar um núcleo de processador
  typedef logic [$clog2(NUM_CORES)-1:0] core_id_t;

  /**
   * @brief Define os 3 Níveis da Arquitetura Adaptativa.
   */
  typedef enum logic [1:0] {
    SPEC_LEVEL_0_BYPASS,       // Nível 0: Serializar (Inseguro: FENCE, AMO)
    SPEC_LEVEL_1_CONSERVATIVE, // Nível 1: Paralelismo de Bloco (Seguro: LOAD/COMPUTE)
    SPEC_LEVEL_2_OPTIMISTIC    // Nível 2: Especulação de Thread (Arriscado: STORE/BRANCH)
  } spec_level_t;

endpackage : stu_pkg

`endif
