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
  PSG_ERR_READONLY_ASSIGN,
  PSG_ERR_LOCAL_LIMIT,
  PSG_ERR_ASSIGN_LIMIT
};

// Output slot bit masks
static const uint8_t PSG_MASK_NEXT_RADIUS    = 0x01;
static const uint8_t PSG_MASK_NEXT_ANGLE     = 0x02;
static const uint8_t PSG_MASK_DELTA_RADIUS   = 0x04;
static const uint8_t PSG_MASK_DELTA_ANGLE    = 0x08;

// Limits (align with implementation plan)
static const uint16_t PSG_MAX_SCRIPT_CHARS   = 1024;
static const uint8_t  PSG_MAX_TOKENS         = 360;
static const uint8_t  PSG_MAX_STACK_DEPTH    = 32;
static const uint8_t  PSG_MAX_EXPR_BYTES     = 192; // per assignment expression (bytecode payload)
static const uint8_t  PSG_MAX_ASSIGNMENTS    = 32; // outputs + locals combined
static const uint8_t  PSG_MAX_LOCALS         = 16;

// Opcode enumeration
enum PSGOp : uint8_t {
  PSG_OP_CONST = 0x01,
  PSG_OP_LOAD  = 0x02,
  PSG_OP_ADD   = 0x10,
  PSG_OP_SUB   = 0x11,
  PSG_OP_MUL   = 0x12,
  PSG_OP_DIV   = 0x13,
  PSG_OP_MOD   = 0x14,
  PSG_OP_NEG   = 0x15,
  PSG_OP_SIN   = 0x16,
  PSG_OP_COS   = 0x17,
  PSG_OP_ABS   = 0x18,
  PSG_OP_CLAMP = 0x19,
  PSG_OP_PINGPONG = 0x1B,
  PSG_OP_SIGN  = 0x1A,
  PSG_OP_END   = 0xFF
};

// Variable slots (read-only first)
// Order important for LOAD operand mapping.
// These correspond to runtime indices 0..N
enum PSGVar : uint8_t {
  PSG_VAR_RADIUS = 0,      // input radius (cm)
  PSG_VAR_ANGLE = 1,       // input angle (deg, wrapped)
  PSG_VAR_START = 2,       // 1 or 0 on restart
  PSG_VAR_REV = 3,         // continuous revolutions (float)
  PSG_VAR_STEPS = 4,       // evaluation counter
  PSG_VAR_TIME = 5,        // milliseconds since start
  // Output / writable slots (only after evaluation)
  PSG_VAR_NEXT_RADIUS = 6,
  PSG_VAR_NEXT_ANGLE = 7,
  PSG_VAR_DELTA_RADIUS = 8,
  PSG_VAR_DELTA_ANGLE = 9,
  PSG_VAR_LOCAL_BASE = 10, // locals start here
  PSG_VAR_MAX = PSG_VAR_LOCAL_BASE + PSG_MAX_LOCALS
};

struct PSGExpr {
  uint8_t used = 0;             // 1 if expression present
  uint8_t opCount = 0;          // number of encoded opcodes (excluding END sentinel)
  uint8_t bytecode[PSG_MAX_EXPR_BYTES];
};

struct PSGAssignment {
  uint8_t target;               // PSGVar index for assignment target
  PSGExpr expr;                 // compiled expression
};

struct PatternScript {
  uint8_t usedMask = 0;         // bitmask of NEXT_/DELTA_ assignments present
  uint8_t localCount = 0;       // number of locals compiled (0..PSG_MAX_LOCALS)
  uint8_t assignmentCount = 0;  // total assignment expressions in evaluation order
  PSGAssignment assignments[PSG_MAX_ASSIGNMENTS];
};

// Compilation entry point
PSGCompileResult compilePatternScript(const char* src, PatternScript &out, String *errMsg = nullptr);

struct PatternScriptRuntime {
  bool initialized = false;
  float prevAngleDeg = 0.0f;
  float unwrappedAngleDeg = 0.0f;
  uint32_t stepCounter = 0;
  uint32_t startMillis = 0;
};

struct PatternScriptUnits {
  float stepsPerCm = 700.0f;     // default fallback (override via configure function)
  float stepsPerDeg = 11.377f;   // steps per degree for angular axis
  float maxRadiusCm = 10.0f;     // clamp radius (cm)
};

void configurePatternScriptUnits(const PatternScriptUnits &units);

// Evaluation entry point
Positions evalPatternScript(const PatternScript &ps, PatternScriptRuntime &rt, const Positions &current, bool startFlag);

// Utility: convert error to const char* (optional)
const char* psgErrorToString(PSGCompileResult code);
