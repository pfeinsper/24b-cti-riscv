#include "neorv32_zfinx_extension_intrinsics.h"
#include "../../../../submodules/neorv32/sw/lib/include/neorv32_intrinsics.h"


// Helper function to flush subnormal numbers to zero
float subnormal_flush(float tmp) {
  float res = tmp;
  if (fpclassify(tmp) == FP_SUBNORMAL) {
    if (signbit(tmp)) {
      res = -0.0f;
    } else {
      res = +0.0f;
    }
  }
  return res;
}

// "Intrinsics" function definitions
float riscv_intrinsic_fadds(float rs1, float rs2) {
  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  res.binary_value = CUSTOM_INSTR_R3_TYPE(0b0000000, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}

float riscv_intrinsic_fsubs(float rs1, float rs2) {
  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  res.binary_value = CUSTOM_INSTR_R3_TYPE(0b0000100, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}

float riscv_intrinsic_fmuls(float rs1, float rs2) {
  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  res.binary_value = CUSTOM_INSTR_R3_TYPE(0b0001000, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}

float riscv_intrinsic_fmins(float rs1, float rs2) {
  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  res.binary_value = CUSTOM_INSTR_R3_TYPE(0b0010100, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}

float riscv_intrinsic_fmaxs(float rs1, float rs2) {
  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  res.binary_value = CUSTOM_INSTR_R3_TYPE(0b0010100, opb.binary_value, opa.binary_value, 0b001, 0b1010011);
  return res.float_value;
}

uint32_t riscv_intrinsic_fcvt_wus(float rs1) {
  float_conv_t opa;
  opa.float_value = rs1;
  return CUSTOM_INSTR_R2_TYPE(0b1100000, 0b00001, opa.binary_value, 0b000, 0b1010011);
}

int32_t riscv_intrinsic_fcvt_ws(float rs1) {
  float_conv_t opa;
  opa.float_value = rs1;
  return (int32_t)CUSTOM_INSTR_R2_TYPE(0b1100000, 0b00000, opa.binary_value, 0b000, 0b1010011);
}

float riscv_intrinsic_fcvt_swu(uint32_t rs1) {
  float_conv_t res;
  res.binary_value = CUSTOM_INSTR_R2_TYPE(0b1101000, 0b00001, rs1, 0b000, 0b1010011);
  return res.float_value;
}

float riscv_intrinsic_fcvt_sw(int32_t rs1) {
  float_conv_t res;
  res.binary_value = CUSTOM_INSTR_R2_TYPE(0b1101000, 0b00000, rs1, 0b000, 0b1010011);
  return res.float_value;
}

uint32_t riscv_intrinsic_feqs(float rs1, float rs2) {
  float_conv_t opa, opb;
  opa.float_value = rs1;
  opb.float_value = rs2;
  return CUSTOM_INSTR_R3_TYPE(0b1010000, opb.binary_value, opa.binary_value, 0b010, 0b1010011);
}

uint32_t riscv_intrinsic_flts(float rs1, float rs2) {
  float_conv_t opa, opb;
  opa.float_value = rs1;
  opb.float_value = rs2;
  return CUSTOM_INSTR_R3_TYPE(0b1010000, opb.binary_value, opa.binary_value, 0b001, 0b1010011);
}

uint32_t riscv_intrinsic_fles(float rs1, float rs2) {
  float_conv_t opa, opb;
  opa.float_value = rs1;
  opb.float_value = rs2;
  return CUSTOM_INSTR_R3_TYPE(0b1010000, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
}

float riscv_intrinsic_fsgnjs(float rs1, float rs2) {
  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  res.binary_value = CUSTOM_INSTR_R3_TYPE(0b0010000, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}

float riscv_intrinsic_fsgnjns(float rs1, float rs2) {
  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  res.binary_value = CUSTOM_INSTR_R3_TYPE(0b0010000, opb.binary_value, opa.binary_value, 0b001, 0b1010011);
  return res.float_value;
}

float riscv_intrinsic_fsgnjxs(float rs1, float rs2) {
  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  res.binary_value = CUSTOM_INSTR_R3_TYPE(0b0010000, opb.binary_value, opa.binary_value, 0b010, 0b1010011);
  return res.float_value;
}

uint32_t riscv_intrinsic_fclasss(float rs1) {
  float_conv_t opa;
  opa.float_value = rs1;
  return CUSTOM_INSTR_R2_TYPE(0b1110000, 0b00000, opa.binary_value, 0b001, 0b1010011);
}

float riscv_intrinsic_fdivs(float rs1, float rs2) {
  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  res.binary_value = CUSTOM_INSTR_R3_TYPE(0b0001100, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}

float riscv_intrinsic_fsqrts(float rs1) {
  float_conv_t opa, res;
  opa.float_value = rs1;
  res.binary_value = CUSTOM_INSTR_R2_TYPE(0b0101100, 0b00000, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}

float riscv_intrinsic_fmadds(float rs1, float rs2, float rs3) {
  float_conv_t opa, opb, opc, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  opc.float_value = rs3;
  res.binary_value = CUSTOM_INSTR_R4_TYPE(opc.binary_value, opb.binary_value, opa.binary_value, 0b000, 0b1000011);
  return res.float_value;
}

float riscv_intrinsic_fmsubs(float rs1, float rs2, float rs3) {
  float_conv_t opa, opb, opc, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  opc.float_value = rs3;
  res.binary_value = CUSTOM_INSTR_R4_TYPE(opc.binary_value, opb.binary_value, opa.binary_value, 0b000, 0b1000111);
  return res.float_value;
}

float riscv_intrinsic_fnmsubs(float rs1, float rs2, float rs3) {
  float_conv_t opa, opb, opc, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  opc.float_value = rs3;
  res.binary_value = CUSTOM_INSTR_R4_TYPE(opc.binary_value, opb.binary_value, opa.binary_value, 0b000, 0b1001011);
  return res.float_value;
}

float riscv_intrinsic_fnmadds(float rs1, float rs2, float rs3) {
  float_conv_t opa, opb, opc, res;
  opa.float_value = rs1;
  opb.float_value = rs2;
  opc.float_value = rs3;
  res.binary_value = CUSTOM_INSTR_R4_TYPE(opc.binary_value, opb.binary_value, opa.binary_value, 0b000, 0b1001111);
  return res.float_value;
}

// Emulation functions definitions
float riscv_emulate_fadds(float rs1, float rs2) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float res = opa + opb;
  if (fpclassify(res) == FP_NAN) {
    res = NAN;
  }
  return subnormal_flush(res);
}

float riscv_emulate_fsubs(float rs1, float rs2) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float res = opa - opb;
  if (fpclassify(res) == FP_NAN) {
    res = NAN;
  }
  return subnormal_flush(res);
}

float riscv_emulate_fmuls(float rs1, float rs2) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float res = opa * opb;
  return subnormal_flush(res);
}

float riscv_emulate_fmins(float rs1, float rs2) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  if ((fpclassify(opa) == FP_NAN) && (fpclassify(opb) == FP_NAN)) {
    return nanf("");
  }
  if (fpclassify(opa) == FP_NAN) {
    return opb;
  }
  if (fpclassify(opb) == FP_NAN) {
    return opa;
  }
  return fmin(opa, opb);
}

float riscv_emulate_fmaxs(float rs1, float rs2) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  if ((fpclassify(opa) == FP_NAN) && (fpclassify(opb) == FP_NAN)) {
    return nanf("");
  }
  if (fpclassify(opa) == FP_NAN) {
    return opb;
  }
  if (fpclassify(opb) == FP_NAN) {
    return opa;
  }
  return fmax(opa, opb);
}

uint32_t riscv_emulate_fcvt_wus(float rs1) {
  float opa = subnormal_flush(rs1);
  return (uint32_t)rint(opa);
}

int32_t riscv_emulate_fcvt_ws(float rs1) {
  float opa = subnormal_flush(rs1);
  return (int32_t)rint(opa);
}

float riscv_emulate_fcvt_swu(uint32_t rs1) {
  return (float)rs1;
}

float riscv_emulate_fcvt_sw(int32_t rs1) {
  return (float)rs1;
}

uint32_t riscv_emulate_feqs(float rs1, float rs2) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  if ((fpclassify(opa) == FP_NAN) || (fpclassify(opb) == FP_NAN)) {
    return 0;
  }
  if (isless(opa, opb)) {
    return 0;
  } else if (isgreater(opa, opb)) {
    return 0;
  } else {
    return 1;
  }
}

uint32_t riscv_emulate_flts(float rs1, float rs2) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  if ((fpclassify(opa) == FP_NAN) || (fpclassify(opb) == FP_NAN)) {
    return 0;
  }
  return isless(opa, opb) ? 1 : 0;
}

uint32_t riscv_emulate_fles(float rs1, float rs2) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  if ((fpclassify(opa) == FP_NAN) || (fpclassify(opb) == FP_NAN)) {
    return 0;
  }
  return islessequal(opa, opb) ? 1 : 0;
}

float riscv_emulate_fsgnjs(float rs1, float rs2) {
  int sign_1 = signbit(rs1);
  int sign_2 = signbit(rs2);
  if (sign_2 != 0) {
    return sign_1 == 0 ? -rs1 : rs1;
  } else {
    return sign_1 == 0 ? rs1 : -rs1;
  }
}

float riscv_emulate_fsgnjns(float rs1, float rs2) {
  int sign_1 = signbit(rs1);
  int sign_2 = signbit(rs2);
  if (sign_2 != 0) {
    return sign_1 == 0 ? rs1 : -rs1;
  } else {
    return sign_1 == 0 ? -rs1 : rs1;
  }
}

float riscv_emulate_fsgnjxs(float rs1, float rs2) {
  int sign_1 = signbit(rs1);
  int sign_2 = signbit(rs2);
  if (((sign_1 == 0) && (sign_2 != 0)) || ((sign_1 != 0) && (sign_2 == 0))) {
    return -rs1;
  } else {
    return rs1;
  }
}

uint32_t riscv_emulate_fclasss(float rs1) {
  float opa = rs1;

  union {
    uint32_t binary_value; /**< Access as native float */
    float    float_value;  /**< Access in binary representation */
  } aux;

  // RISC-V classify result layout
  const uint32_t CLASS_NEG_INF    = 1 << 0; // negative infinity
  const uint32_t CLASS_NEG_NORM   = 1 << 1; // negative normal number
  const uint32_t CLASS_NEG_DENORM = 1 << 2; // negative subnormal number
  const uint32_t CLASS_NEG_ZERO   = 1 << 3; // negative zero
  const uint32_t CLASS_POS_ZERO   = 1 << 4; // positive zero
  const uint32_t CLASS_POS_DENORM = 1 << 5; // positive subnormal number
  const uint32_t CLASS_POS_NORM   = 1 << 6; // positive normal number
  const uint32_t CLASS_POS_INF    = 1 << 7; // positive infinity
  const uint32_t CLASS_SNAN       = 1 << 8; // signaling NaN (sNaN)
  const uint32_t CLASS_QNAN       = 1 << 9; // quiet NaN (qNaN)

  int tmp = fpclassify(opa);
  int sgn = (int)signbit(opa);

  uint32_t res = 0;

  // infinity
  if (tmp == FP_INFINITE) {
    if (sgn) { res |= CLASS_NEG_INF; }
    else     { res |= CLASS_POS_INF; }
  }

  // zero
  if (tmp == FP_ZERO) {
    if (sgn) { res |= CLASS_NEG_ZERO; }
    else     { res |= CLASS_POS_ZERO; }
  }

  // normal
  if (tmp == FP_NORMAL) {
    if (sgn) { res |= CLASS_NEG_NORM; }
    else     { res |= CLASS_POS_NORM; }
  }

  // subnormal
  if (tmp == FP_SUBNORMAL) {
    if (sgn) { res |= CLASS_NEG_DENORM; }
    else     { res |= CLASS_POS_DENORM; }
  }

  // NaN
  if (tmp == FP_NAN) {
    aux.float_value = opa;
    if ((aux.binary_value >> 22) & 0b1) { // bit 22 (mantissa's MSB) is set -> canonical (quiet) NAN
      res |= CLASS_QNAN;
    }
    else {
      res |= CLASS_SNAN;
    }
  }

  return res;
}

float riscv_emulate_fdivs(float rs1, float rs2) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  return subnormal_flush(opa / opb);
}

float riscv_emulate_fsqrts(float rs1) {
  float opa = subnormal_flush(rs1);
  return subnormal_flush(sqrtf(opa));
}

float riscv_emulate_fmadds(float rs1, float rs2, float rs3) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float opc = subnormal_flush(rs3);
  return subnormal_flush((opa * opb) + opc);
}

float riscv_emulate_fmsubs(float rs1, float rs2, float rs3) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float opc = subnormal_flush(rs3);
  return subnormal_flush((opa * opb) - opc);
}

float riscv_emulate_fnmsubs(float rs1, float rs2, float rs3) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float opc = subnormal_flush(rs3);
  return subnormal_flush(-(opa * opb) + opc);
}

float riscv_emulate_fnmadds(float rs1, float rs2, float rs3) {
  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float opc = subnormal_flush(rs3);
  return subnormal_flush(-(opa * opb) - opc);
}
