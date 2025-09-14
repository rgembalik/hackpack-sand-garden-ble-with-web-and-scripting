#include "PatternScript.h"
#include <math.h>
// Positions struct now provided by Positions.h via PatternScript.h

static float readF32(const uint8_t *p) { float v; memcpy(&v, p, sizeof(float)); return v; }
static void writeF32(uint8_t *p, float v) { memcpy(p, &v, sizeof(float)); }

static const char *ERR_PREFIX = "PSG:";

const char* psgErrorToString(PSGCompileResult code) {
  switch(code) {
    case PSG_OK: return "OK";
    case PSG_ERR_EMPTY: return "EMPTY";
    case PSG_ERR_SYNTAX: return "SYNTAX";
    case PSG_ERR_UNK_IDENT: return "UNKNOWN_IDENT";
    case PSG_ERR_FUNC_ARGS: return "FUNC_ARGS";
    case PSG_ERR_STACK_OVER: return "STACK_OVER";
    case PSG_ERR_TOO_LONG: return "TOO_LONG";
    case PSG_ERR_READONLY_ASSIGN: return "READONLY_ASSIGN";
  }
  return "UNKNOWN";
}

// Very small tokenizer for v0 (placeholder / scaffolding)
// For now this produces a trivial compile that only detects empty script.
// Future steps: implement full tokenization & shunting-yard.
PSGCompileResult compilePatternScript(const char* src, PatternScript &out, String *errMsg) {
  if(!src) return PSG_ERR_SYNTAX;
  size_t len = strlen(src);
  if(len == 0) return PSG_ERR_EMPTY;
  if(len > PSG_MAX_SCRIPT_CHARS) return PSG_ERR_TOO_LONG;

  // TEMP STUB: detect simple presence of assignments and mark masks, no expression bytecode yet.
  // This lets us integrate calling pipeline before full compiler.
  const char *p = src;
  while(*p) {
    if(strncmp(p, "next_radius", 11)==0) { out.usedMask |= PSG_MASK_NEXT_RADIUS; }
    else if(strncmp(p, "next_angle", 10)==0) { out.usedMask |= PSG_MASK_NEXT_ANGLE; }
    else if(strncmp(p, "delta_radius", 12)==0) { out.usedMask |= PSG_MASK_DELTA_RADIUS; }
    else if(strncmp(p, "delta_angle", 11)==0) { out.usedMask |= PSG_MASK_DELTA_ANGLE; }
    ++p;
  }
  if(out.usedMask == 0) return PSG_ERR_EMPTY;

  // Mark dummy expressions as present (single END byte)
  auto mark = [](PSGExpr &e){ e.used = 1; e.bytecode[0] = PSG_OP_END; };
  if(out.usedMask & PSG_MASK_NEXT_RADIUS) mark(out.exprNextRadius);
  if(out.usedMask & PSG_MASK_NEXT_ANGLE) mark(out.exprNextAngle);
  if(out.usedMask & PSG_MASK_DELTA_RADIUS) mark(out.exprDeltaRadius);
  if(out.usedMask & PSG_MASK_DELTA_ANGLE) mark(out.exprDeltaAngle);

  return PSG_OK;
}

// Evaluate stub: just echoes current + simple constants (no real expression execution yet)
Positions evalPatternScript(const PatternScript &ps, const Positions &current, bool startFlag) {
  Positions out = current;
  // Placeholder behavior: if next_* assigned, add fixed offset; if delta_* assigned, add smaller offset.
  if(ps.usedMask & PSG_MASK_NEXT_RADIUS) out.radial = current.radial + 10; // TEMP
  else if(ps.usedMask & PSG_MASK_DELTA_RADIUS) out.radial = current.radial + 5; // TEMP

  if(ps.usedMask & PSG_MASK_NEXT_ANGLE) out.angular = current.angular + 50; // TEMP
  else if(ps.usedMask & PSG_MASK_DELTA_ANGLE) out.angular = current.angular + 20; // TEMP

  return out;
}
