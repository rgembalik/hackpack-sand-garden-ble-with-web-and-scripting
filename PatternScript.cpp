#include <Arduino.h>
#include "PatternScript.h"
#include <math.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <cctype>
#include <cstring>
#ifdef ARDUINO_ARCH_ESP32
#include <esp_random.h>
#endif

static PatternScriptUnits g_units = {};

static inline float degToRad(float deg) {
  return deg * (float)M_PI / 180.0f;
}

static inline uint32_t psgRandomNext(uint32_t state) {
  return state * 1664525u + 1013904223u;
}

void configurePatternScriptUnits(const PatternScriptUnits &units) {
  g_units = units;
  if (g_units.stepsPerCm <= 0.0f) g_units.stepsPerCm = 700.0f;
  if (g_units.stepsPerDeg <= 0.0f) g_units.stepsPerDeg = 11.377f;
  if (g_units.maxRadiusCm <= 0.0f) g_units.maxRadiusCm = 10.0f;
}

static const char *ERR_PREFIX = "PSG:";

typedef PSGCompileResult CompileResult;

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
    case PSG_ERR_LOCAL_LIMIT: return "LOCAL_LIMIT";
    case PSG_ERR_ASSIGN_LIMIT: return "ASSIGN_LIMIT";
  }
  return "UNKNOWN";
}

namespace {
struct SymbolInfo {
  uint8_t index;
  bool readOnly;
  bool ready;
  bool isOutput;
  bool isLocal;
};

enum class TokenType : uint8_t {
  Number,
  Identifier,
  Operator,
  LParen,
  RParen,
  Comma
};

struct ExprToken {
  TokenType type;
  float number = 0.0f;
  std::string text;
  bool isFunction = false;
  bool isUnary = false;
  PSGOp op = PSG_OP_END;
  uint8_t varIndex = 0;
};

enum class RPNType : uint8_t {
  Const,
  Load,
  Op
};

struct RPNToken {
  RPNType type;
  float value = 0.0f;
  uint8_t varIndex = 0;
  PSGOp op = PSG_OP_END;
  uint8_t argCount = 0;
};

struct OperatorInfo {
  PSGOp op;
  int precedence;
  bool rightAssoc;
  uint8_t arity;
};

static OperatorInfo getOperatorInfo(const ExprToken &tok) {
  if (tok.isUnary) {
    return {PSG_OP_NEG, 3, true, 1};
  }
  if (tok.text == "+") return {PSG_OP_ADD, 1, false, 2};
  if (tok.text == "-") return {PSG_OP_SUB, 1, false, 2};
  if (tok.text == "*") return {PSG_OP_MUL, 2, false, 2};
  if (tok.text == "/") return {PSG_OP_DIV, 2, false, 2};
  if (tok.text == "%") return {PSG_OP_MOD, 2, false, 2};
  return {PSG_OP_END, 0, false, 0};
}

static bool isAlpha(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}

static bool isAlnum(char c) {
  return isAlpha(c) || (c >= '0' && c <= '9');
}

static std::string toLower(const std::string &s) {
  std::string out = s;
  for (char &c : out) c = (char)tolower((unsigned char)c);
  return out;
}

static std::string trim(const std::string &s) {
  size_t start = 0;
  while (start < s.size() && isspace((unsigned char)s[start])) start++;
  size_t end = s.size();
  while (end > start && isspace((unsigned char)s[end - 1])) end--;
  return s.substr(start, end - start);
}

static bool isDigitOrDot(char c) {
  return (c >= '0' && c <= '9') || c == '.';
}

static bool parseNumber(const std::string &expr, size_t &pos, float &out) {
  size_t start = pos;
  bool hasDot = false;
  while (pos < expr.size()) {
    char ch = expr[pos];
    if (ch == '.') {
      if (hasDot) break;
      hasDot = true;
      pos++;
    } else if ((ch >= '0' && ch <= '9')) {
      pos++;
    } else {
      break;
    }
  }
  if (pos < expr.size() && (expr[pos] == 'e' || expr[pos] == 'E')) {
    pos++;
    if (pos < expr.size() && (expr[pos] == '+' || expr[pos] == '-')) pos++;
    while (pos < expr.size() && (expr[pos] >= '0' && expr[pos] <= '9')) pos++;
  }
  if (pos == start) return false;
  out = atof(expr.substr(start, pos - start).c_str());
  return true;
}

static bool expectIdentifier(const std::string &id) {
  if (id.empty()) return false;
  if (!isAlpha(id[0])) return false;
  for (size_t i = 1; i < id.size(); ++i) {
    if (!isAlnum(id[i])) return false;
  }
  return true;
}

static bool isFunctionName(const std::string &name) {
  return name == "sin" || name == "cos" || name == "tan" || name == "abs" || name == "clamp" || name == "sign" || name == "pingpong" ||
         name == "min" || name == "max" || name == "pow" || name == "sqrt" || name == "exp" || name == "random" ||
         name == "floor" || name == "ceil" || name == "round";
}

static uint8_t maskForOutput(const std::string &name) {
  if (name == "next_radius") return PSG_MASK_NEXT_RADIUS;
  if (name == "next_angle") return PSG_MASK_NEXT_ANGLE;
  if (name == "delta_radius") return PSG_MASK_DELTA_RADIUS;
  if (name == "delta_angle") return PSG_MASK_DELTA_ANGLE;
  return 0;
}

static PSGOp functionToOp(const std::string &fn) {
  if (fn == "sin") return PSG_OP_SIN;
  if (fn == "cos") return PSG_OP_COS;
  if (fn == "tan") return PSG_OP_TAN;
  if (fn == "abs") return PSG_OP_ABS;
  if (fn == "clamp") return PSG_OP_CLAMP;
  if (fn == "sign") return PSG_OP_SIGN;
  if (fn == "pingpong") return PSG_OP_PINGPONG;
  if (fn == "min") return PSG_OP_MIN;
  if (fn == "max") return PSG_OP_MAX;
  if (fn == "pow") return PSG_OP_POW;
  if (fn == "sqrt") return PSG_OP_SQRT;
  if (fn == "exp") return PSG_OP_EXP;
  if (fn == "random") return PSG_OP_RANDOM;
  if (fn == "floor") return PSG_OP_FLOOR;
  if (fn == "ceil") return PSG_OP_CEIL;
  if (fn == "round") return PSG_OP_ROUND;
  return PSG_OP_END;
}

static uint8_t expectedArgs(const std::string &fn) {
  if (fn == "clamp") return 3;
  if (fn == "pingpong") return 2;
  if (fn == "min") return 2;
  if (fn == "max") return 2;
  if (fn == "pow") return 2;
  if (fn == "random") return 0;
  return 1;
}

static CompileResult emitError(CompileResult code, String *errMsg, const std::string &msg, int line) {
  if (errMsg) {
    String out = ERR_PREFIX;
    out += " ";
    out += msg.c_str();
    if (line >= 0) {
      out += " (line ";
      out += String(line);
      out += ")";
    }
    *errMsg = out;
  }
  return code;
}

static CompileResult tokenizeExpression(const std::string &expr,
                                        const std::unordered_map<std::string, SymbolInfo> &symbols,
                                        std::vector<ExprToken> &tokens,
                                        size_t &tokenBudget,
                                        String *errMsg,
                                        int line) {
  tokens.clear();
  size_t pos = 0;
  bool expectUnary = true;
  while (pos < expr.size()) {
    char ch = expr[pos];
    if (isspace((unsigned char)ch)) {
      pos++;
      continue;
    }
    if (tokenBudget == 0) {
      return emitError(PSG_ERR_TOO_LONG, errMsg, "token budget exceeded", line);
    }
    if (isDigitOrDot(ch)) {
      float num = 0.0f;
      if (!parseNumber(expr, pos, num)) {
        return emitError(PSG_ERR_SYNTAX, errMsg, "invalid number", line);
      }
      ExprToken tok;
      tok.type = TokenType::Number;
      tok.number = num;
      tokens.push_back(tok);
      expectUnary = false;
      tokenBudget--;
      continue;
    }
    if (isAlpha(ch)) {
      size_t start = pos;
      while (pos < expr.size() && isAlnum(expr[pos])) pos++;
      std::string ident = toLower(expr.substr(start, pos - start));
      if (!expectIdentifier(ident)) {
        return emitError(PSG_ERR_SYNTAX, errMsg, "invalid identifier", line);
      }
      ExprToken tok;
      tok.text = ident;
      if (isFunctionName(ident)) {
        tok.type = TokenType::Identifier;
        tok.isFunction = true;
      } else {
        auto it = symbols.find(ident);
        if (it == symbols.end() || !it->second.ready) {
          return emitError(PSG_ERR_UNK_IDENT, errMsg, "unknown identifier " + ident, line);
        }
        tok.type = TokenType::Identifier;
        tok.isFunction = false;
        tok.varIndex = it->second.index;
      }
      tokens.push_back(tok);
      expectUnary = false;
      tokenBudget--;
      continue;
    }
    if (ch == '+' || ch == '-' || ch == '*' || ch == '/' || ch == '%') {
      ExprToken tok;
      tok.type = TokenType::Operator;
      tok.text = std::string(1, ch);
      tok.isUnary = (ch == '-' && expectUnary);
      tokens.push_back(tok);
      expectUnary = true;
      pos++;
      tokenBudget--;
      continue;
    }
    if (ch == '(') {
      ExprToken tok;
      tok.type = TokenType::LParen;
      tokens.push_back(tok);
      pos++;
      tokenBudget--;
      expectUnary = true;
      continue;
    }
    if (ch == ')') {
      ExprToken tok;
      tok.type = TokenType::RParen;
      tokens.push_back(tok);
      pos++;
      tokenBudget--;
      expectUnary = false;
      continue;
    }
    if (ch == ',') {
      ExprToken tok;
      tok.type = TokenType::Comma;
      tokens.push_back(tok);
      pos++;
      tokenBudget--;
      expectUnary = true;
      continue;
    }
    return emitError(PSG_ERR_SYNTAX, errMsg, std::string("unexpected character '") + ch + "'", line);
  }
  return PSG_OK;
}

static CompileResult toRPN(const std::vector<ExprToken> &tokens,
                           std::vector<RPNToken> &output,
                           String *errMsg,
                           int line) {
  output.clear();
  struct StackEntry {
    enum Type { Operator, Function, LParen } type;
    OperatorInfo info;
    std::string func;
  };
  std::vector<StackEntry> stack;
  std::vector<uint8_t> funcArgCounts;
  for (size_t i = 0; i < tokens.size(); ++i) {
    const ExprToken &tok = tokens[i];
    switch (tok.type) {
      case TokenType::Number: {
        RPNToken rt;
        rt.type = RPNType::Const;
        rt.value = tok.number;
        output.push_back(rt);
        break;
      }
      case TokenType::Identifier: {
        if (tok.isFunction) {
          StackEntry entry;
          entry.type = StackEntry::Function;
          entry.info.op = PSG_OP_END;
          entry.info.precedence = 0;
          entry.info.rightAssoc = false;
          entry.info.arity = 0;
          entry.func = tok.text;
          stack.push_back(entry);
          funcArgCounts.push_back(0);
        } else {
          RPNToken rt;
          rt.type = RPNType::Load;
          rt.varIndex = tok.varIndex;
          output.push_back(rt);
        }
        break;
      }
      case TokenType::Operator: {
        OperatorInfo info = getOperatorInfo(tok);
        if (info.op == PSG_OP_END) {
          return emitError(PSG_ERR_SYNTAX, errMsg, "unknown operator", line);
        }
        while (!stack.empty()) {
          const StackEntry &top = stack.back();
          if (top.type != StackEntry::Operator) break;
          int precTop = top.info.precedence;
          int precCur = info.precedence;
          if ((info.rightAssoc && precCur < precTop) || (!info.rightAssoc && precCur <= precTop)) {
            RPNToken rt;
            rt.type = RPNType::Op;
            rt.op = top.info.op;
            rt.argCount = top.info.arity;
            output.push_back(rt);
            stack.pop_back();
          } else {
            break;
          }
        }
        StackEntry entry;
        entry.type = StackEntry::Operator;
        entry.info = info;
        entry.func.clear();
        stack.push_back(entry);
        break;
      }
      case TokenType::LParen: {
        StackEntry entry;
        entry.type = StackEntry::LParen;
        entry.info = {};
        entry.func.clear();
        stack.push_back(entry);
        break;
      }
      case TokenType::RParen: {
        bool foundParen = false;
        while (!stack.empty()) {
          StackEntry entry = stack.back();
          stack.pop_back();
          if (entry.type == StackEntry::LParen) {
            foundParen = true;
            break;
          }
          RPNToken rt;
          rt.type = RPNType::Op;
          rt.op = entry.info.op;
          rt.argCount = entry.info.arity;
          output.push_back(rt);
        }
        if (!foundParen) {
          return emitError(PSG_ERR_SYNTAX, errMsg, "mismatched parentheses", line);
        }
        if (!stack.empty() && stack.back().type == StackEntry::Function) {
          StackEntry fnEntry = stack.back();
          stack.pop_back();
          uint8_t argCount = funcArgCounts.back();
          funcArgCounts.pop_back();
          bool hadExplicitArgs = argCount > 0;
          if (!hadExplicitArgs) {
            bool hadValue = (i >= 1 && (tokens[i-1].type == TokenType::Number ||
                                        (tokens[i-1].type == TokenType::Identifier && !tokens[i-1].isFunction) ||
                                        tokens[i-1].type == TokenType::RParen));
            argCount = hadValue ? 1 : 0;
          } else {
            argCount += 1;
          }
          uint8_t expected = expectedArgs(fnEntry.func);
          bool ok = false;
          if (fnEntry.func == "pingpong") {
            ok = (argCount == 1 || argCount == 2);
          } else {
            ok = (argCount == expected);
          }
          if (!ok) {
            return emitError(PSG_ERR_FUNC_ARGS, errMsg, "function argument mismatch", line);
          }
          RPNToken rt;
          rt.type = RPNType::Op;
          rt.op = functionToOp(fnEntry.func);
          rt.argCount = argCount;
          output.push_back(rt);
        }
        break;
      }
      case TokenType::Comma: {
        bool foundParen = false;
        while (!stack.empty()) {
          if (stack.back().type == StackEntry::LParen) {
            foundParen = true;
            break;
          }
          RPNToken rt;
          rt.type = RPNType::Op;
          rt.op = stack.back().info.op;
          rt.argCount = stack.back().info.arity;
          output.push_back(rt);
          stack.pop_back();
        }
        if (!foundParen) {
          return emitError(PSG_ERR_SYNTAX, errMsg, "misplaced comma", line);
        }
        if (!funcArgCounts.empty()) funcArgCounts.back() += 1;
        break;
      }
    }
  }
  while (!stack.empty()) {
    StackEntry entry = stack.back();
    stack.pop_back();
    if (entry.type == StackEntry::LParen) {
      return emitError(PSG_ERR_SYNTAX, errMsg, "mismatched parentheses", line);
    }
    if (entry.type == StackEntry::Function) {
      uint8_t argCount = funcArgCounts.empty() ? 0 : funcArgCounts.back() + 1;
      if (!funcArgCounts.empty()) funcArgCounts.pop_back();
      bool ok = false;
      if (entry.func == "pingpong") {
        ok = (argCount == 1 || argCount == 2);
      } else {
        uint8_t expected = expectedArgs(entry.func);
        ok = (argCount == expected);
      }
      if (!ok) {
        return emitError(PSG_ERR_FUNC_ARGS, errMsg, "function argument mismatch", line);
      }
      RPNToken rt;
      rt.type = RPNType::Op;
      rt.op = functionToOp(entry.func);
      rt.argCount = argCount;
      output.push_back(rt);
    } else {
      RPNToken rt;
      rt.type = RPNType::Op;
      rt.op = entry.info.op;
      rt.argCount = entry.info.arity;
      output.push_back(rt);
    }
  }
  return PSG_OK;
}

static CompileResult encodeBytecode(const std::vector<RPNToken> &rpn,
                                    PSGExpr &expr,
                                    String *errMsg,
                                    int line) {
  expr.used = 1;
  expr.opCount = 0;
  size_t cursor = 0;
  int stackDepth = 0;
  for (const RPNToken &rt : rpn) {
    switch (rt.type) {
      case RPNType::Const: {
        if (cursor + 1 + sizeof(float) > PSG_MAX_EXPR_BYTES) {
          return emitError(PSG_ERR_TOO_LONG, errMsg, "expression too long", line);
        }
        expr.bytecode[cursor++] = PSG_OP_CONST;
        memcpy(expr.bytecode + cursor, &rt.value, sizeof(float));
        cursor += sizeof(float);
        expr.opCount++;
        stackDepth++;
        if (stackDepth > PSG_MAX_STACK_DEPTH) {
          return emitError(PSG_ERR_STACK_OVER, errMsg, "expression stack overflow", line);
        }
        break;
      }
      case RPNType::Load: {
        if (cursor + 2 > PSG_MAX_EXPR_BYTES) {
          return emitError(PSG_ERR_TOO_LONG, errMsg, "expression too long", line);
        }
        expr.bytecode[cursor++] = PSG_OP_LOAD;
        expr.bytecode[cursor++] = rt.varIndex;
        expr.opCount++;
        stackDepth++;
        if (stackDepth > PSG_MAX_STACK_DEPTH) {
          return emitError(PSG_ERR_STACK_OVER, errMsg, "expression stack overflow", line);
        }
        break;
      }
      case RPNType::Op: {
        if (cursor + 1 > PSG_MAX_EXPR_BYTES) {
          return emitError(PSG_ERR_TOO_LONG, errMsg, "expression too long", line);
        }
        expr.bytecode[cursor++] = rt.op;
        expr.opCount++;
        uint8_t arity = rt.argCount;
        if (stackDepth < (int)arity) {
          return emitError(PSG_ERR_SYNTAX, errMsg, "stack underflow", line);
        }
        stackDepth -= (int)arity;
        stackDepth += 1;
        if (stackDepth > PSG_MAX_STACK_DEPTH) {
          return emitError(PSG_ERR_STACK_OVER, errMsg, "expression stack overflow", line);
        }
        break;
      }
    }
  }
  if (stackDepth != 1) {
    return emitError(PSG_ERR_SYNTAX, errMsg, "expression did not resolve to single value", line);
  }
  return PSG_OK;
}

} // namespace

PSGCompileResult compilePatternScript(const char* src, PatternScript &out, String *errMsg) {
  memset(&out, 0, sizeof(out));
  if (!src) return PSG_ERR_SYNTAX;
  size_t len = strlen(src);
  if (len == 0) return PSG_ERR_EMPTY;
  if (len > PSG_MAX_SCRIPT_CHARS) return PSG_ERR_TOO_LONG;
  std::string script(src);
  if (!script.empty() && script.back() == '\r') script.pop_back();

  std::unordered_map<std::string, SymbolInfo> symbols;
  auto addSymbol = [&](const std::string &name, uint8_t index, bool readOnly, bool ready, bool isOutput, bool isLocal) {
    symbols[name] = {index, readOnly, ready, isOutput, isLocal};
  };
  addSymbol("radius", PSG_VAR_RADIUS, true, true, false, false);
  addSymbol("angle", PSG_VAR_ANGLE, true, true, false, false);
  addSymbol("start", PSG_VAR_START, true, true, false, false);
  addSymbol("rev", PSG_VAR_REV, true, true, false, false);
  addSymbol("steps", PSG_VAR_STEPS, true, true, false, false);
  addSymbol("time", PSG_VAR_TIME, true, true, false, false);
  addSymbol("next_radius", PSG_VAR_NEXT_RADIUS, false, false, true, false);
  addSymbol("next_angle", PSG_VAR_NEXT_ANGLE, false, false, true, false);
  addSymbol("delta_radius", PSG_VAR_DELTA_RADIUS, false, false, true, false);
  addSymbol("delta_angle", PSG_VAR_DELTA_ANGLE, false, false, true, false);

  size_t tokenBudget = PSG_MAX_TOKENS;
  size_t lineNo = 0;
  out.assignmentCount = 0;
  out.localCount = 0;
  out.usedMask = 0;

  size_t start = 0;
  while (start < script.size()) {
    size_t end = script.find('\n', start);
    if (end == std::string::npos) end = script.size();
    std::string rawLine = script.substr(start, end - start);
    if (!rawLine.empty() && rawLine.back() == '\r') rawLine.pop_back();
    lineNo++;
    start = end + 1;

    size_t commentPos = rawLine.find('#');
    if (commentPos != std::string::npos) rawLine = rawLine.substr(0, commentPos);
    std::string line = trim(rawLine);
    if (line.empty()) continue;

    size_t eqPos = line.find('=');
    if (eqPos == std::string::npos) {
      return emitError(PSG_ERR_SYNTAX, errMsg, "missing '='", (int)lineNo);
    }
    std::string lhs = trim(line.substr(0, eqPos));
    std::string rhs = trim(line.substr(eqPos + 1));
    if (lhs.empty() || rhs.empty()) {
      return emitError(PSG_ERR_SYNTAX, errMsg, "invalid assignment", (int)lineNo);
    }
    lhs = toLower(lhs);
    if (!expectIdentifier(lhs)) {
      return emitError(PSG_ERR_SYNTAX, errMsg, "invalid identifier", (int)lineNo);
    }

    SymbolInfo targetInfo{};
    auto it = symbols.find(lhs);
    if (it == symbols.end()) {
      if (out.localCount >= PSG_MAX_LOCALS) {
        return emitError(PSG_ERR_LOCAL_LIMIT, errMsg, "too many locals", (int)lineNo);
      }
      uint8_t idx = PSG_VAR_LOCAL_BASE + out.localCount;
      out.localCount++;
      targetInfo = {idx, false, false, false, true};
      symbols[lhs] = targetInfo;
    } else {
      targetInfo = it->second;
    }

    if (targetInfo.readOnly) {
      return emitError(PSG_ERR_READONLY_ASSIGN, errMsg, "cannot assign read-only identifier", (int)lineNo);
    }

    std::vector<ExprToken> tokens;
    CompileResult res = tokenizeExpression(rhs, symbols, tokens, tokenBudget, errMsg, (int)lineNo);
    if (res != PSG_OK) return res;

    std::vector<RPNToken> rpn;
    res = toRPN(tokens, rpn, errMsg, (int)lineNo);
    if (res != PSG_OK) return res;

    if (out.assignmentCount >= PSG_MAX_ASSIGNMENTS) {
      return emitError(PSG_ERR_ASSIGN_LIMIT, errMsg, "too many assignments", (int)lineNo);
    }
    PSGAssignment &assign = out.assignments[out.assignmentCount++];
    assign.target = targetInfo.index;
    res = encodeBytecode(rpn, assign.expr, errMsg, (int)lineNo);
    if (res != PSG_OK) return res;

    SymbolInfo &symRef = symbols[lhs];
    symRef.ready = true;

    if (symRef.isOutput) {
      out.usedMask |= maskForOutput(lhs);
    }
  }

  if (out.usedMask == 0) {
    return PSG_ERR_EMPTY;
  }

  return PSG_OK;
}

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float wrapDeg(float deg) {
  float out = fmodf(deg, 360.0f);
  if (out < 0.0f) out += 360.0f;
  return out;
}

Positions evalPatternScript(const PatternScript &ps, PatternScriptRuntime &rt, const Positions &current, bool startFlag) {
  Positions result = current;
  if (ps.assignmentCount == 0) {
    return result;
  }

  rt.faulted = false;
  rt.faultMask = 0;

  float stepsPerCm = g_units.stepsPerCm > 0 ? g_units.stepsPerCm : 700.0f;
  float stepsPerDeg = g_units.stepsPerDeg > 0 ? g_units.stepsPerDeg : 11.377f;
  float maxRadiusCm = g_units.maxRadiusCm > 0 ? g_units.maxRadiusCm : 10.0f;

  float radiusCm = current.radial / stepsPerCm;
  float angleDeg = current.angular / stepsPerDeg;
  angleDeg = wrapDeg(angleDeg);

  uint32_t nowMs = millis();
  if (!rt.initialized || startFlag) {
    rt.initialized = true;
    rt.prevAngleDeg = angleDeg;
    rt.unwrappedAngleDeg = angleDeg;
    rt.stepCounter = 0;
    rt.startMillis = nowMs;
    rt.randomInitialized = false;
  } else {
    float deltaDeg = angleDeg - rt.prevAngleDeg;
    if (deltaDeg > 180.0f) deltaDeg -= 360.0f;
    else if (deltaDeg < -180.0f) deltaDeg += 360.0f;
    rt.unwrappedAngleDeg += deltaDeg;
    rt.prevAngleDeg = angleDeg;
  }

  float vars[PSG_VAR_MAX] = {0.0f};
  vars[PSG_VAR_RADIUS] = radiusCm;
  vars[PSG_VAR_ANGLE] = angleDeg;
  vars[PSG_VAR_START] = startFlag ? 1.0f : 0.0f;
  vars[PSG_VAR_REV] = rt.unwrappedAngleDeg / 360.0f;
  vars[PSG_VAR_STEPS] = (float)rt.stepCounter;
  vars[PSG_VAR_TIME] = (float)((nowMs >= rt.startMillis) ? (nowMs - rt.startMillis) : 0U);

  float stack[PSG_MAX_STACK_DEPTH];

  for (uint8_t i = 0; i < ps.assignmentCount; ++i) {
    const PSGAssignment &assign = ps.assignments[i];
    const PSGExpr &expr = assign.expr;
    int sp = 0;
    size_t cursor = 0;
    for (uint8_t opIndex = 0; opIndex < expr.opCount; ++opIndex) {
      PSGOp opcode = (PSGOp)expr.bytecode[cursor++];
      switch (opcode) {
        case PSG_OP_CONST: {
          float val;
          memcpy(&val, expr.bytecode + cursor, sizeof(float));
          cursor += sizeof(float);
          stack[sp++] = val;
          break;
        }
        case PSG_OP_LOAD: {
          uint8_t idx = expr.bytecode[cursor++];
          if (idx >= PSG_VAR_MAX) idx = PSG_VAR_MAX - 1;
          stack[sp++] = vars[idx];
          break;
        }
        case PSG_OP_ADD: {
          float b = stack[--sp];
          float a = stack[--sp];
          stack[sp++] = a + b;
          break;
        }
        case PSG_OP_SUB: {
          float b = stack[--sp];
          float a = stack[--sp];
          stack[sp++] = a - b;
          break;
        }
        case PSG_OP_MUL: {
          float b = stack[--sp];
          float a = stack[--sp];
          stack[sp++] = a * b;
          break;
        }
        case PSG_OP_DIV: {
          float b = stack[--sp];
          float a = stack[--sp];
          stack[sp++] = a / b;
          break;
        }
        case PSG_OP_MOD: {
          float b = stack[--sp];
          float a = stack[--sp];
          stack[sp++] = fmodf(a, b);
          break;
        }
        case PSG_OP_NEG: {
          float a = stack[--sp];
          stack[sp++] = -a;
          break;
        }
        case PSG_OP_SIN: {
          float a = stack[--sp];
          stack[sp++] = sinf(degToRad(a));
          break;
        }
        case PSG_OP_COS: {
          float a = stack[--sp];
          stack[sp++] = cosf(degToRad(a));
          break;
        }
        case PSG_OP_TAN: {
          float a = stack[--sp];
          stack[sp++] = tanf(degToRad(a));
          break;
        }
        case PSG_OP_ABS: {
          float a = stack[--sp];
          stack[sp++] = fabsf(a);
          break;
        }
        case PSG_OP_PINGPONG: {
          // expects two args on stack: value, max -> returns triangular/pingpong between 0 and max
          float maxV = stack[--sp];
          float v = stack[--sp];
          if (!isfinite(maxV) || maxV <= 0.0f) {
            stack[sp++] = 0.0f;
            break;
          }
          float period = 2.0f * maxV;
          float t = fmodf(v, period);
          if (t < 0.0f) t += period;
          float out = (t <= maxV) ? t : (2.0f * maxV - t);
          stack[sp++] = out;
          break;
        }
        case PSG_OP_CLAMP: {
          float maxV = stack[--sp];
          float minV = stack[--sp];
          float val = stack[--sp];
          stack[sp++] = clampf(val, minV, maxV);
          break;
        }
        case PSG_OP_SIGN: {
          float a = stack[--sp];
          float outVal = 0.0f;
          if (a > 1e-6f) outVal = 1.0f;
          else if (a < -1e-6f) outVal = -1.0f;
          stack[sp++] = outVal;
          break;
        }
        case PSG_OP_MIN: {
          float b = stack[--sp];
          float a = stack[--sp];
          stack[sp++] = fminf(a, b);
          break;
        }
        case PSG_OP_MAX: {
          float b = stack[--sp];
          float a = stack[--sp];
          stack[sp++] = fmaxf(a, b);
          break;
        }
        case PSG_OP_POW: {
          float b = stack[--sp];
          float a = stack[--sp];
          stack[sp++] = powf(a, b);
          break;
        }
        case PSG_OP_SQRT: {
          float a = stack[--sp];
          stack[sp++] = sqrtf(a);
          break;
        }
        case PSG_OP_EXP: {
          float a = stack[--sp];
          stack[sp++] = expf(a);
          break;
        }
        case PSG_OP_FLOOR: {
          float a = stack[--sp];
          stack[sp++] = floorf(a);
          break;
        }
        case PSG_OP_CEIL: {
          float a = stack[--sp];
          stack[sp++] = ceilf(a);
          break;
        }
        case PSG_OP_ROUND: {
          float a = stack[--sp];
          stack[sp++] = roundf(a);
          break;
        }
        case PSG_OP_RANDOM: {
          if (!rt.randomInitialized) {
#if defined(ARDUINO_ARCH_ESP32)
            uint32_t seed = esp_random();
#else
            uint32_t seed = (uint32_t)random();
#endif
            if (seed == 0) {
              seed = ((uint32_t)millis() << 16) ^ (uint32_t)current.radial ^ 0xA5A5A5A5u;
            }
            rt.randomState = seed;
            rt.randomInitialized = true;
          }
          rt.randomState = psgRandomNext(rt.randomState);
          float out = (rt.randomState >> 8) * (1.0f / 16777216.0f);
          stack[sp++] = out;
          break;
        }
        default:
          break;
      }
    }
    float value = (sp > 0) ? stack[sp - 1] : 0.0f;
    vars[assign.target] = value;
  }

  uint8_t faultMask = 0;
  if (ps.usedMask & PSG_MASK_NEXT_RADIUS) {
    if (!isfinite(vars[PSG_VAR_NEXT_RADIUS])) faultMask |= PSG_MASK_NEXT_RADIUS;
  }
  if (ps.usedMask & PSG_MASK_DELTA_RADIUS) {
    if (!isfinite(vars[PSG_VAR_DELTA_RADIUS])) faultMask |= PSG_MASK_DELTA_RADIUS;
  }
  if (ps.usedMask & PSG_MASK_NEXT_ANGLE) {
    if (!isfinite(vars[PSG_VAR_NEXT_ANGLE])) faultMask |= PSG_MASK_NEXT_ANGLE;
  }
  if (ps.usedMask & PSG_MASK_DELTA_ANGLE) {
    if (!isfinite(vars[PSG_VAR_DELTA_ANGLE])) faultMask |= PSG_MASK_DELTA_ANGLE;
  }

  float outRadius = radiusCm;
  if (ps.usedMask & PSG_MASK_NEXT_RADIUS) {
    outRadius = vars[PSG_VAR_NEXT_RADIUS];
  } else if (ps.usedMask & PSG_MASK_DELTA_RADIUS) {
    outRadius = radiusCm + vars[PSG_VAR_DELTA_RADIUS];
  }
  float outAngle = angleDeg;
  if (ps.usedMask & PSG_MASK_NEXT_ANGLE) {
    outAngle = vars[PSG_VAR_NEXT_ANGLE];
  } else if (ps.usedMask & PSG_MASK_DELTA_ANGLE) {
    outAngle = angleDeg + vars[PSG_VAR_DELTA_ANGLE];
  }

  if (!isfinite(outRadius)) {
    if (ps.usedMask & PSG_MASK_NEXT_RADIUS) faultMask |= PSG_MASK_NEXT_RADIUS;
    else if (ps.usedMask & PSG_MASK_DELTA_RADIUS) faultMask |= PSG_MASK_DELTA_RADIUS;
  }
  if (!isfinite(outAngle)) {
    if (ps.usedMask & PSG_MASK_NEXT_ANGLE) faultMask |= PSG_MASK_NEXT_ANGLE;
    else if (ps.usedMask & PSG_MASK_DELTA_ANGLE) faultMask |= PSG_MASK_DELTA_ANGLE;
  }

  if (faultMask != 0) {
    rt.faulted = true;
    rt.faultMask = faultMask;
    return result;
  }

  rt.stepCounter++;

  outRadius = clampf(outRadius, 0.0f, maxRadiusCm);
  outAngle = wrapDeg(outAngle);

  float radialSteps = outRadius * stepsPerCm;
  float angularSteps = outAngle * stepsPerDeg;
  if (!isfinite(radialSteps)) {
    rt.faulted = true;
    rt.faultMask = (ps.usedMask & PSG_MASK_NEXT_RADIUS) ? (uint8_t)PSG_MASK_NEXT_RADIUS : (ps.usedMask & PSG_MASK_DELTA_RADIUS) ? (uint8_t)PSG_MASK_DELTA_RADIUS : 0;
    return result;
  }
  if (!isfinite(angularSteps)) {
    rt.faulted = true;
    rt.faultMask = (ps.usedMask & PSG_MASK_NEXT_ANGLE) ? (uint8_t)PSG_MASK_NEXT_ANGLE : (ps.usedMask & PSG_MASK_DELTA_ANGLE) ? (uint8_t)PSG_MASK_DELTA_ANGLE : rt.faultMask;
    return result;
  }

  result.radial = (int)lroundf(radialSteps);
  result.angular = (int)lroundf(angularSteps);
  return result;
}
