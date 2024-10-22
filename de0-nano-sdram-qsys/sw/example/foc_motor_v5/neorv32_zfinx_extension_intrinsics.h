#ifndef neorv32_zfinx_extension_intrinsics_h
#define neorv32_zfinx_extension_intrinsics_h

#include <float.h>
#include <math.h>
#include <stdint.h>

// Sanity check
#if defined __riscv_f || (__riscv_flen == 32)
  #error Application programs using the Zfinx intrinsic library have to be compiled WITHOUT the <F> MARCH ISA attribute!
#endif

// Custom data type to access floating-point values as native floats and in binary representation
typedef union
{
  uint32_t binary_value; /**< Access as native float */
  float    float_value;  /**< Access in binary representation */
} float_conv_t;

// Helper functions
float subnormal_flush(float tmp);

// "Intrinsics" function declarations
float riscv_intrinsic_fadds(float rs1, float rs2);
float riscv_intrinsic_fsubs(float rs1, float rs2);
float riscv_intrinsic_fmuls(float rs1, float rs2);
float riscv_intrinsic_fmins(float rs1, float rs2);
float riscv_intrinsic_fmaxs(float rs1, float rs2);
uint32_t riscv_intrinsic_fcvt_wus(float rs1);
int32_t riscv_intrinsic_fcvt_ws(float rs1);
float riscv_intrinsic_fcvt_swu(uint32_t rs1);
float riscv_intrinsic_fcvt_sw(int32_t rs1);
uint32_t riscv_intrinsic_feqs(float rs1, float rs2);
uint32_t riscv_intrinsic_flts(float rs1, float rs2);
uint32_t riscv_intrinsic_fles(float rs1, float rs2);
float riscv_intrinsic_fsgnjs(float rs1, float rs2);
float riscv_intrinsic_fsgnjns(float rs1, float rs2);
float riscv_intrinsic_fsgnjxs(float rs1, float rs2);
uint32_t riscv_intrinsic_fclasss(float rs1);
float riscv_intrinsic_fdivs(float rs1, float rs2);
float riscv_intrinsic_fsqrts(float rs1);
float riscv_intrinsic_fmadds(float rs1, float rs2, float rs3);
float riscv_intrinsic_fmsubs(float rs1, float rs2, float rs3);
float riscv_intrinsic_fnmsubs(float rs1, float rs2, float rs3);
float riscv_intrinsic_fnmadds(float rs1, float rs2, float rs3);

// Emulation functions declarations
float riscv_emulate_fadds(float rs1, float rs2);
float riscv_emulate_fsubs(float rs1, float rs2);
float riscv_emulate_fmuls(float rs1, float rs2);
float riscv_emulate_fmins(float rs1, float rs2);
float riscv_emulate_fmaxs(float rs1, float rs2);
uint32_t riscv_emulate_fcvt_wus(float rs1);
int32_t riscv_emulate_fcvt_ws(float rs1);
float riscv_emulate_fcvt_swu(uint32_t rs1);
float riscv_emulate_fcvt_sw(int32_t rs1);
uint32_t riscv_emulate_feqs(float rs1, float rs2);
uint32_t riscv_emulate_flts(float rs1, float rs2);
uint32_t riscv_emulate_fles(float rs1, float rs2);
float riscv_emulate_fsgnjs(float rs1, float rs2);
float riscv_emulate_fsgnjns(float rs1, float rs2);
float riscv_emulate_fsgnjxs(float rs1, float rs2);
uint32_t riscv_emulate_fclasss(float rs1);
float riscv_emulate_fdivs(float rs1, float rs2);
float riscv_emulate_fsqrts(float rs1);
float riscv_emulate_fmadds(float rs1, float rs2, float rs3);
float riscv_emulate_fmsubs(float rs1, float rs2, float rs3);
float riscv_emulate_fnmsubs(float rs1, float rs2, float rs3);
float riscv_emulate_fnmadds(float rs1, float rs2, float rs3);

#endif // neorv32_zfinx_extension_intrinsics_h
