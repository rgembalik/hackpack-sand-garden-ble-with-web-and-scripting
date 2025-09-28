# Sand Garden Pattern DSL Implementation Plan

Status: Working Draft v0.5.0 (web + firmware compilers aligned; zero-division guard unified)
Date: 2025-09-28
Owner: (You)

## Progress Snapshot

| Area / Task | Status | Notes |
|-------------|--------|-------|
| Requirements capture | ✅ Done | Clarified math‑expression DSL scope. |
| Concept model options | ✅ Done | Chose assignment-only + math functions. |
| Grammar & identifiers | ✅ Done | `next_/delta_` outputs fixed. |
| Web DSL compiler (JS) | ✅ Done | Tokenizer + shunting yard → op-list (bytecode-like) evaluation (CONST/LOAD/ADD/...). |
| Ordered output references | ✅ Done | Later outputs can reference earlier ones. |
| Multi-line parsing robustness | ✅ Done | Explicit EOL tokens implemented. |
| Degree-based trig | ✅ Done | `sin` / `cos` accept degrees (no rad variant now). |
| Physical units abstraction | ✅ Done | Script sees `radius` (cm) & `angle` (deg); internal conversion to steps. |
| Example scripts (base creative set) | ✅ Refreshed | Vibrant set incl. PingPongPetals, MirrorSpiral, EchoBloom, BounceWeave, StarPing, LaceHelix, RevCascade, StartBurst, PetalDrift, SelfRefWarp. |
| Firmware scaffold (headers & stubs) | ✅ Done | `PatternScript.h/.cpp` placeholders. |
| Real firmware compiler | ✅ Done | `PatternScript.cpp` implements tokenizer, parser, op encoder, and runtime with modulo/sign/time parity (DIV/MOD → 0 on zero denominator). |
| BLE script upload protocol | ✅ Done | `BLEConfigServer` exposes BEGIN/DATA/END pipeline with timeout + status callbacks. |
| Documentation (user-facing usage) | 🚧 In progress | README rewrite underway focusing on beginner quick-start + DSL primer. |
| Internal representation write-up | ⏳ Pending | Section to be expanded (todo). |
| Progressive enhancement roadmap | ⏳ Pending | Draft list exists; needs refinement to match new unit model. |
| `rev` synthetic input | ✅ Done | Continuous revolution counter (unwrappedAngleDeg / 360). |
| `steps` synthetic input | ✅ Done | Evaluation counter (increments each DSL evaluation). |
| `time` synthetic input | ✅ Done | Milliseconds since script start (web: performance.now baseline). |
| Ephemeral local variables | ✅ Done | Any new identifier assigned becomes single-pass local. |
| `sign()` function | ✅ Done | Enables arithmetic mirror / ping-pong patterns. |
| Mirror arithmetic guidance | ✅ Done | Achieved via `dir = sign(cos(rev * 180))` pattern; no mode flag in DSL. |

Legend: ✅ Completed / ⏳ Pending / 🚧 In Progress

## 1. Purpose
Provide a **minimal, math-expression style DSL** to define motion patterns for the Sand Garden. The same script text:
- Runs in firmware (ESP32) using a tiny interpreter.
- Runs in the HTML client for preview & visualization.
- Can be embedded as built-in patterns (flash constants) or uploaded via BLE (chunked if needed).

## 2. Current Scope (v0 Requirements – Updated v0.5.0)
| Feature | Included v0 | Notes |
|---------|-------------|-------|
| Absolute outputs | Yes | Provide next absolute polar target. |
| Relative outputs | Optional (dr/da) | If provided, applied after input state. |
| Inputs (read-only) | radius (cm), angle (deg), start, rev, steps, time | `angle` wrapped 0–360; `rev = unwrappedAngleDeg / 360` continuous; `steps` increments each evaluation; `time` ms since first evaluation (web uses high-res timer). |
| Time / counters | Incorporated (`steps`, `time`) | Early inclusion to enable richer temporal patterns; firmware must add before parity release. |
| Variables / params | Ephemeral locals | Any non-reserved identifier assigned becomes a one-pass local (not persisted). |
| Math operators | + - * / % unary -, parentheses | `%` remainder (b==0 → 0). |
| Trig | sin(), cos() (degree-based) | Degree arguments; radians removed for now. |
| Other math | abs(), clamp(x,a,b), sign(x) | `sign` returns -1,0,1 enabling mirror / ping-pong. |
| Power / sqrt | Optional (defer) | Add in v1 if needed. |
| Modulus operator | Yes | Remainder operator `%` (b==0 → 0). |
| Conditionals / ternary | No | Defer. |
| Comments | `# ...` end-of-line | For readability. |
| Whitespace | Ignored | Standard. |
| Errors | Fail fast with error code | Firmware rejects invalid script. |
| Bytecode | Removed in web | Web evaluates RPN directly; firmware will still compile to compact op list. |
| BLE transfer | Yes | Raw ASCII script; firmware compiles on receipt. |

Rationale: Keep the first version absolutely minimal to ensure rapid integration with existing pattern callback style.

## 3. Pattern Contract Alignment
Existing firmware pattern signature (conceptual):
```cpp
Positions pattern_X(Positions current, bool restart);
```
DSL runtime wrapper will expose:

Unit abstraction (new):
- Script receives `radius` (float cm) derived from `current.radial` steps via `stepsPerCm` constant.
- Script receives `angle` (float degrees) derived from `current.angular` steps via `degToSteps` conversion.
- `start = restart ? 1 : 0` unchanged.

Firmware will maintain conversion constants (e.g. `PSG_MAX_RADIUS_CM`, `PSG_STEPS_PER_REV`, `PSG_MAX_R_STEPS`) as internal configurable values—not exposed in script.

Outputs defined by the script (still in cm / degrees):
- `next_radius = <expr>` sets absolute next radial (cm) before conversion.
- `next_angle = <expr>` sets absolute next angular (deg).
- Optionally `delta_radius = <expr>` and / or `delta_angle = <expr>` for relative increments from current units.

Resolution priority per axis:
1. If `next_radius` assigned in script -> use that.
2. Else if `delta_radius` assigned -> `next_radius = radius + delta_radius`.
3. Else -> `next_radius = radius`.
(Same for `next_angle` / `delta_angle`.)

All values are evaluated as floats in script units, then clamped / wrapped, converted, and integer-truncated to step units for motor commands.

Ordered output references: A later assignment may reference a previously defined output symbol (e.g. define `next_radius` then use it inside the expression for `next_angle`). Referencing an output before it is assigned is a compile error.

## 4. Source Form (v0 DSL Examples – Updated Units)
```
# Simple outward spiral (cm / deg)
next_radius = clamp(radius + 0.25, 0, 15)
next_angle = angle + 40
```
```
# Relative increments (triangle cadence)
delta_radius = 0.30
delta_angle = 120
```
```
# Start pulse (jump 5 cm once)
next_radius = radius + start * 5
next_angle = angle + 60
```
```
# Wavy spiral (angle-based modulation)
next_radius = clamp(radius + 0.2 + sin(angle * 2) * 0.6, 0, 15)
next_angle = angle + 34
```
```
# Ordered output reference
next_radius = radius + 0.5
next_angle = angle + 15 * sin(next_radius * 12)
```

## 5. Grammar (EBNF v0.4.2)
```
program    := (statement | comment)*
statement  := assignment
assignment := ident '=' expr
ident      := outputIdent | inputIdent | localIdent
outputIdent:= 'next_radius' | 'next_angle' | 'delta_radius' | 'delta_angle'
inputIdent := 'radius' | 'angle' | 'start' | 'rev' | 'steps' | 'time'
localIdent := /[a-zA-Z_][a-zA-Z0-9_]*/  (but not colliding with reserved output/input)
expr       := term (('+'|'-') term)*
term       := factor (('*'|'/'|'%') factor)*
factor     := unary
unary      := ('-' unary) | primary
primary    := NUMBER | ident | func | '(' expr ')'
func       := funcName '(' argList? ')'
argList    := expr (',' expr)*
funcName   := 'sin' | 'cos' | 'abs' | 'clamp' | 'sign'
comment    := '#' <any chars until EOL>
NUMBER     := [0-9]+ ('.' [0-9]+)?
```
Notes:
- Left-associative +,-,*,/,% ( * / % share precedence ).
- No exponent operator in v0.
- `clamp(x,a,b)` returns min(max(x,a),b).
- User-defined identifiers now allowed as ephemeral locals (single-pass) if not reserved.
- Reassignment overwrites earlier value within the same evaluation pass.
- Forward reference to a local/output before assignment is a compile error.

## 6. Token Types (v0.4)
| Type | Example | Stored As |
|------|---------|-----------|
| NUMBER | 123 or 3.14 | float (32-bit) |
| IDENT | Outputs (next_radius/...) Inputs (radius/angle/start/rev) Locals (dynamic) | enum / index |
| OP | + - * / = ( ) , | char / enum |
| FUNC | sin | enum |
| COMMENT | #... | skipped |

Read-only identifiers: `radius`, `angle`, `start`, `rev`, `steps`, `time`.
Assignable identifiers:
- Outputs: `next_radius`, `next_angle`, `delta_radius`, `delta_angle`
- Locals: any other valid identifier (ephemeral; resets each eval call)

## 7. Internal Representation & Evaluation (RPN v0.3.1)
The web client evaluates *direct RPN* (no bytecode array). Firmware will store a compact op list per assignment; semantics are identical.

### 7.1 RPN Logical Form (Web)
For each assignment:
1. Tokenize → Shunting-yard → RPN token list
2. Convert to array of op objects `{op:OPCODE, v?:number|string}`
3. Evaluate sequentially using a float stack

### 7.2 Firmware Compact Op Encoding (Updated for steps/time inputs & modulo v0.4.2)
Define opcodes (1 byte) – proposed values:
| Opcode | Hex | Stack Effect | Extra Bytes | Description |
|--------|-----|--------------|-------------|-------------|
| OP_CONST | 0x01 | push | 4 (f32) | Push literal |
| OP_LOAD  | 0x02 | push | 1 (u8 index) | Load variable index (input/output/local) |
| OP_ADD   | 0x03 | pop2→push | 0 | a+b |
| OP_SUB   | 0x04 | pop2→push | 0 | a-b |
| OP_MUL   | 0x05 | pop2→push | 0 | a*b |
| OP_DIV   | 0x06 | pop2→push | 0 | a/b (if |b|<eps -> 0) |
| OP_MOD   | 0x07 | pop2→push | 0 | fmod(a,b) (if |b|<eps -> 0) |
| OP_NEG   | 0x08 | pop→push | 0 | -a |
| OP_SIN   | 0x09 | pop→push | 0 | sin(deg) |
| OP_COS   | 0x0A | pop→push | 0 | cos(deg) |
| OP_ABS   | 0x0B | pop→push | 0 | |a| |
| OP_CLAMP | 0x0C | pop3→push | 0 | clamp(value,min,max) (stack order value,min,max) |
| OP_SIGN  | 0x0D | pop→push | 0 | sign(a) (-1,0,1) |
| OP_END   | 0xFF | (flush) | 0 | End sentinel (optional if length stored) |

Variable index mapping (u8) (proposed updated ordering keeps legacy first four, appends new temporal inputs before outputs to simplify LOAD reuse while compiling outputs sequentially — alternative ordering acceptable if documented):
| Index | Symbol |
|-------|--------|
| 0 | radius (cm) |
| 1 | angle (deg wrapped 0–360) |
| 2 | start (0/1) |
| 3 | rev (continuous revolutions) |
| 4 | steps (evaluation counter) |
| 5 | time (ms since script start) |
| 6 | next_radius (if already assigned) |
| 7 | next_angle |
| 8 | delta_radius |
| 9 | delta_angle |
| 10..(10+L-1) | locals in discovery order |

Rationale: placing `steps` & `time` before outputs ensures any script referencing them during the assignment of first output uses a consistent low index. If firmware has already reserved indices 4..7 for outputs (legacy design), an alternate mapping preserving that order SHOULD be implemented and this document updated accordingly. Parity tests must assert mapping.

Max locals L ≤ 8 → final highest index ≤ 15 (fits in u8 easily). If more locals required → ERR_LOCAL_LIMIT.

### 7.3 Firmware Data Structures (Proposed – updated indices)
```c
struct PSG_Op {
  uint8_t opcode; // OP_CONST, OP_LOAD, ...
  union { float f; uint8_t idx; } u; // Only valid for OP_CONST (f) or OP_LOAD (idx)
};

struct PSG_Assignment {
  uint8_t targetVarIndex;    // 4..7 or local index if we unify; store which output/local this sequence computes
  uint8_t opCount;           // number of ops (excluding OP_END if omitted)
  PSG_Op ops[PSG_MAX_OPS];   // packed ops
};

struct PatternScript {
  uint8_t outputsMask;       // bit per output present (bit0 next_radius, bit1 next_angle, bit2 delta_radius, bit3 delta_angle)
  uint8_t localCount;        // number of locals (for indexing base = 8)
  uint8_t assignmentCount;   // total compiled assignments (outputs + locals)
  PSG_Assignment assignments[PSG_MAX_ASSIGNMENTS];
};

struct PSG_Runtime {
  float radius, angle, rev;  // inputs each call (angle wrapped, rev continuous)
  float stepsF;              // copy of evaluation counter as float (index 4)
  float timeMs;              // elapsed ms since start (index 5)
  float stack[PSG_STACK_DEPTH];
  float locals[8];
  float outputs[4];
  float unwrappedAngleDeg;   // persisted between calls for rev computation
};
```

### 7.4 Continuous Revolution & Temporal Tracking (Firmware)
Maintain `unwrappedAngleDeg` across evaluations (reset on restart):
```
delta = angleDegWrapped - prevAngleDegWrapped;
if (delta > 180) delta -= 360; else if (delta < -180) delta += 360;
unwrappedAngleDeg += delta;
rev = unwrappedAngleDeg / 360.0f;
```
This matches web logic ensuring mirror / ping‑pong scripts behave identically.

Additional temporal inputs:
```
steps: uint32_t evaluation counter (reset to 0 on restart BEFORE first eval; first call sees steps=0).
time:  milliseconds since restart baseline (first call sees time≈0).
```
Both are exposed as floats to the VM (LOAD indices 4 and 5). Firmware implementation: maintain `uint32_t stepCounter; uint32_t startMillis;` and update prior to evaluation of each script step.

### 7.5 Evaluation Algorithm (Firmware)
```
for each assignment in source order:
  evaluate ops:
    switch(opcode):
      LOAD -> push(varTable[idx])
      CONST -> push(f)
  ADD/SUB/MUL/DIV/MOD/NEG/SIN/COS/ABS/CLAMP/SIGN -> manipulate stack
  result = pop();
  store in target (output or local array)
Resolve next_radius / next_angle precedence vs delta_*
Clamp radius to [0,MAX_R_CM]; wrap angle to [0,360)
Convert units to steps
```

### 7.6 Compile-time Checks
- Unknown identifier → ERR_UNK_IDENT
- Output/local forward reference → ERR_FWD_REF
- >8 locals → ERR_LOCAL_LIMIT
- Stack usage > PSG_STACK_DEPTH → ERR_STACK_OVER
- Script length / tokens / ops exceed limits → ERR_TOO_LONG

### 7.7 Execution Edge Cases
- Division by zero (DIV or MOD): return 0 (push 0) to avoid Inf/NaN.
- `sin`/`cos` expect degrees; use fast degree→rad conversion: `rad = deg * (PI/180)`.
- `clamp`: assume caller supplies sensible a≤b; if a>b treat min/max swapped.
- `sign(±0)` returns 0; NaN input yields 0 (avoid propagating NaN to motion).

### 7.8 Determinism & Temporal Parity
No RNG in v0. Temporal variables introduce non-determinism relative to wall-clock; for parity tests, inject a deterministic `time` progression (e.g. fixed dt per eval). Web harness should allow overriding `time` injection for golden tests. All math uses single-precision on MCU; differences between web (double) and MCU (float) acceptable (<1 step) but can be reduced by applying `Math.fround` when generating expected comparisons.

### 7.9 Memory Footprint Targets
Derive per-assignment op storage: If max 48 ops * (1 opcode + 4 bytes const worst case) ~ 240B per heavy assignment; with 4 outputs + locals typical < 1 KB.

### 7.10 Optional Future Optimization
Pack small consts via 16-bit index to shared constant pool; not needed in v0.

## 8. Memory / Limits (Firmware) (revisit for locals/sign/temporal)
| Item | Limit |
|------|-------|
| Max script length (chars) | 512 (fits small BLE upload, adjustable) |
| Max tokens | 128 |
| Max ops per assignment | 48 (tunable) |
| Stack depth | 16 (verify still sufficient with clamp(sign()) nesting; adjust if temporal scripts push limits) |
| Number of assignments | Up to 4 outputs + locals (locals counted toward op & token limits) |
| Max locals | 8 |

If limits exceeded → error code (INVALID_TOO_LARGE).

## 9. Error Codes (Minimal v0.4)
| Code | Meaning |
|------|---------|
| OK | Success |
| ERR_EMPTY | No assignments found |
| ERR_SYNTAX | Generic parse error |
| ERR_UNK_IDENT | Unknown identifier |
| ERR_FUNC_ARGS | Wrong number of args to function |
| ERR_LOCAL_LIMIT | Too many locals |
| ERR_FWD_REF | Forward reference before assignment |
| ERR_STACK_OVER | Expression nesting too deep |
| ERR_TOO_LONG | Script exceeds character / token / op limits |
| ERR_READONLY_ASSIGN | Attempt to assign radius/angle/start |

Return numeric codes to caller; optionally map to short message for BLE/status log.

## 10. Firmware Integration Plan
1. **Header**: `PatternScript.h` declares structs in Section 7.3 plus constants: `PSG_MAX_SCRIPT_CHARS`, `PSG_MAX_TOKENS`, `PSG_MAX_OPS`, `PSG_STACK_DEPTH`.
2. **Compile Function**: `int psgCompile(const char* src, PatternScript& out);`
  - Performs tokenize → parse/assignments → RPN → op emission → validation.
3. **Eval Function**: `Positions psgEval(PatternScript& ps, PSG_Runtime& rt, const Positions& current, bool restart);`
  - Handles unit conversion, continuous rev update, environment seeding, sequential assignment evaluation, resolution & clamping.
4. **Wrapper Pattern Functions**: Embedded scripts compiled at startup into a static `PatternScript`; runtime keeps a `PSG_Runtime` instance (persists `unwrappedAngleDeg`).
4. **BLE Upload Path**:
   - Command sequence: client sends `SCRIPT_START <length>`, then `SCRIPT_CHUNK` packets, then `SCRIPT_END`.
   - Accumulate into buffer, compile, respond with status.
   - Provide ephemeral `currentScript` slot or assign to pattern index.
5. **Safety**: If eval returns NaN or INF, replace output with prior position.

## 11. Web (HTML) Client Plan (RPN only)
Components:
- Text area / code editor (Monaco optional later).
- JS tokenizer & compiler mirroring firmware grammar.
- RPN evaluator (no separate bytecode layer) producing successive points.
- Preview renderer: radial plot or path accumulation (simulate N steps).
- Upload button: splits raw text into BLE chunks (MTU - header).

Simulator loop (JS):
```js
let pos = { r: 0, a: 0 };
let first = true;
for (let i=0;i<steps;i++) {
  pos = evalPatternScript(compiled, pos, first);
  first = false;
  draw(pos);
}
```

## 12. BLE Chunking Protocol (Draft)
- Characteristic: `pattern_script`
- Packet formats (ASCII or small binary header):
  - `PS:BEGIN:<len>` (ASCII) → firmware allocates buffer if len <= MAX.
  - `PS:DATA:<seq>:<raw...>` (raw script fragment; `<raw...>` may contain newlines).
  - `PS:END` → compile and respond `PS:OK` or `PS:ERR:<code>`.
Alternative: binary framing:
  `[0xA5][type][seq][payloadLen][payload...]` (optional later for robustness).

## 13. Security / Robustness
- Reject scripts with non-printable characters except `\t \n \r`.
- Enforce single pass compile before activation.
- Keep previous valid script to fallback on failed new compile.

## 14. Development Phases
| Phase | Goal | Deliverables |
|-------|------|--------------|
| P1 | Core grammar & compiler (firmware + JS) | Tokenizer, shunting-yard, RPN → op encoder, tests (parity vs web) |
| P2 | Firmware integration | Wrapper pattern function, error reporting, LED status on compile result |
| P3 | Web simulator | JS VM + basic canvas preview + upload UI |
| P4 | Enhancements (optional) | More math funcs (tan, sqrt, pow), modulus, time `t`, counter `k` |

## 15. Test Scripts (v0 Acceptance – Updated Units & Continuous rev)
A. Minimal single assignment keeping radius:
```
next_angle = angle + 45
```
B. Relative only radius (cm):
```
delta_radius = 0.25
```
C. Absolute overrides relative precedence:
```
delta_radius = 0.30
next_radius  = radius + 1.00
next_angle   = angle + 60
```
D. Start flag usage (jump out 5 cm):
```
next_radius = radius + start * 5
next_angle  = angle + 120
```
E. Trig mixture (degree-based trig, radius modulation by angle):
```
next_angle  = angle + 90 + 20 * sin(angle * 0.5)
next_radius = radius + 0.4 * abs(cos(angle * 3))
```
F. Clamp usage (stay within inner band 3–12 cm):
```
next_radius = clamp(radius + 0.8, 3, 12)
next_angle  = angle + 45
```
G. Ordered output reference:
```
next_radius = radius + 0.5
next_angle  = angle + 10 * sin(next_radius * 8)
```

H. Mirror Spiral (continuous rev driven radius):
```
phase = (sin(rev*360) + 1) * 0.5
target = phase * 15
next_radius = clamp(radius + (target - radius)*0.30, 0, 15)
next_angle  = angle + 6
```

## 16. RPN Example Encoding (unchanged semantics)
Expression: `angle + 90 + 20 * sin(radius * 0.01)`
Tokens → RPN: `angle 90 + 20 radius 0.01 * sin * +`
Firmware compact ops (illustrative):
```
LOAD(angle)
CONST(90)
ADD
CONST(20)
LOAD(radius)
CONST(0.01)
MUL
SIN
MUL
ADD
```

## 17. MCU Resource Estimate
- Tokenizer: ~1.5 KB flash
- Parser + shunting yard: ~2.5 KB
- VM: ~1 KB
- Total < 6 KB (excluding trig functions from libm)
- RAM: script buffer (512B) + token array (~512B) + bytecode (~256B) + stack (64B) ≈ < 1.5 KB transient.

## 18. Future (v1+) Extensions (Not in v0 – updated for v0.4.2)
| Feature | Notes |
|---------|-------|
| Additional functions | tan, sqrt, pow, noise, rand1. |
| Ternary | Adds jump ops; keep simple for now. |
| User params | `param speed = 0.5` pre-eval once. |
| Variables | Introduce `let x = ...` persistent between eval calls. |
| Color outputs | `h = ..., b = ...` for LED. |
| Multi-phase patterns | `phase { ... }` with time bounds. |

## 19. Open Decisions (Updated)
| Decision | Options |
|----------|---------|
| Include tan()/sqrt()/pow() in v0? | Default no; add when requested. |
| Output reference policy | Ordered references allowed (already implemented). |
| Unit abstraction permanence | Lock cm/deg unless strong reason to revert. |
| Add constants (pi, tau) | Deferred – reintroduce only if needed. |
| `rev` input permanence | Locked in (proved useful). |
| sign() permanence | Keep (tiny & enables mirror). |
| Local variable limit | Tentatively 8 (review after MCU memory test). |
| BLE framing ASCII vs Binary first? | ASCII for debugging. |

## 20. Next Immediate Actions (v0.5.0)


## 21. Deferred / Nice-to-haves
1. Explore constant-pool compression or 16-bit literal packing to shrink large scripts.
2. Add additional math functions (wrapClamp? other trigs?)

**End of Plan v0.5.0 Draft**
