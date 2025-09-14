#pragma once
#include <Arduino.h>
#include "Positions.h" // shared Positions struct

// Error codes
enum PSGCompileResult : uint8_t {
  PSG_OK = 0,
  PSG_ERR_EMPTY,
  PSG_ERR_SYNTAX,
  PSG_ERR_UNK_IDENT,
  PSG_ERR_FUNC_ARGS,
  PSG_ERR_STACK_OVER,
  PSG_ERR_TOO_LONG,
  PSG_ERR_READONLY_ASSIGN
};

// Output slot bit masks
static const uint8_t PSG_MASK_NEXT_RADIUS    = 0x01;
static const uint8_t PSG_MASK_NEXT_ANGLE     = 0x02;
static const uint8_t PSG_MASK_DELTA_RADIUS   = 0x04;
static const uint8_t PSG_MASK_DELTA_ANGLE    = 0x08;

// Limits (align with plan)
static const uint16_t PSG_MAX_SCRIPT_CHARS   = 512;
static const uint8_t  PSG_MAX_TOKENS         = 128;
static const uint8_t  PSG_MAX_STACK_DEPTH    = 16;
static const uint8_t  PSG_MAX_EXPR_BYTES     = 64; // per assignment expression

// Opcode enumeration
enum PSGOp : uint8_t {
  PSG_OP_CONST = 0x01,
  PSG_OP_LOAD  = 0x02,
  PSG_OP_ADD   = 0x10,
  PSG_OP_SUB   = 0x11,
  PSG_OP_MUL   = 0x12,
  PSG_OP_DIV   = 0x13,
  PSG_OP_NEG   = 0x14,
  PSG_OP_SIN   = 0x15,
  PSG_OP_COS   = 0x16,
  PSG_OP_ABS   = 0x17,
  PSG_OP_CLAMP = 0x18,
  PSG_OP_END   = 0xFF
};

// Variable slots (read-only first)
// Order important for LOAD operand mapping.
// These correspond to runtime indices 0..N
enum PSGVar : uint8_t {
  PSG_VAR_RADIUS = 0,      // input radius
  PSG_VAR_ANGLE = 1,       // input angle
  PSG_VAR_START = 2,       // 1 or 0 on restart
  // output / writable (only after expression evaluation)
  PSG_VAR_NEXT_RADIUS = 3,
  PSG_VAR_NEXT_ANGLE = 4,
  PSG_VAR_DELTA_RADIUS = 5,
  PSG_VAR_DELTA_ANGLE = 6,
  PSG_VAR_COUNT
};

struct PSGExpr {
  uint8_t used;                 // 1 if expression present
  uint8_t bytecode[PSG_MAX_EXPR_BYTES];
};

struct PatternScript {
  uint8_t usedMask = 0;         // bitmask of NEXT_/DELTA_ assignments present
  PSGExpr exprNextRadius;       // compiled bytecode if any
  PSGExpr exprNextAngle;
  PSGExpr exprDeltaRadius;
  PSGExpr exprDeltaAngle;
};

// Compilation entry point
PSGCompileResult compilePatternScript(const char* src, PatternScript &out, String *errMsg = nullptr);

// Evaluation entry point
Positions evalPatternScript(const PatternScript &ps, const Positions &current, bool startFlag);

// Utility: convert error to const char* (optional)
const char* psgErrorToString(PSGCompileResult code);
